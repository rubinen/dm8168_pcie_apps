#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>


// #include <pthread.h>
// #include <pcie_server_lib.h>
#include "pcie_common.h"


#define APP_NAME     "pcie_sever"

#define dbg(level, debug, fmt, arg...)                \
  do {                                                \
    if (debug >= (level))                             \
      printf(APP_NAME": " fmt , ## arg); \
  } while (0)


typedef struct pcie_channel_s {
  int             handler;    // handle of the file to write
  int             mode;       //
  char            filename[128];   // filename
  int             file_fd;    // file descriptor
  pid_t           pid;        // pid of the child process
  sem_t          *sem;        // semaphore
  int             pipe_fd[2]; // pipe

} pcie_channel_t;


static int debug = 0;


int process_cnt = 0;
pcie_channel_t pcie_channel_table[10] = {-1,};

void init_channels(void)
{
  int i = 0;
  memset(pcie_channel_table, 0, sizeof(pcie_channel_table));

  for (i = 0; i < 10; i++)
  {
    pcie_channel_table[i].handler = -1;
  }
}

int find_free_channel(void)
{
  int i = 0;
  int free_idx = -1;
  for (i = 0; i < 10; i++)
  {
    if (pcie_channel_table[i].handler == -1)
    {
      free_idx = i;
      break;
    }
  }
  return free_idx;
}

/*
  child process will respond to commands:
  - open
  - write, read
  - close
*/
int forked_channel_process(pcie_channel_t *channel)
{
  int len = 0;
  errno = 0;
  channel->file_fd = open(channel->filename, O_RDWR | O_CREAT, 00666);
  printf("[%s] handler:%d open(%s) file_fd: %d errno:%d (%s)\n", __FUNCTION__,
      channel->handler, channel->filename, channel->file_fd, errno, strerror(errno));

  close(channel->pipe_fd[1]); // close the write-end of the pipe, I'm not going to use it

  #warning send RESPONSE to open file

  while(1)
  {
    int cmd = -1;
    printf("[%s] wait handler:%d.... \n", __FUNCTION__, channel->handler);
    sem_wait(channel->sem);
    printf("[%s] go handler:%d \n", __FUNCTION__, channel->handler);

    len = read(channel->pipe_fd[0], &cmd, sizeof(cmd));
    if (len != sizeof(cmd))
    {
      printf("[%s] %s pipe read len:%d\n", __FUNCTION__, channel->filename, len);
      continue;
    }
    printf("[%s] %s pipe cmd:%d\n", __FUNCTION__, channel->filename, cmd);
    if (cmd == PCIE_CMD_CLOSE)
    {
      break;
    }
    if (cmd == PCIE_CMD_WRITE)
    {
      printf("[%s] %s WRITE\n", __FUNCTION__, channel->filename);
      write(channel->file_fd, channel->filename, strlen(channel->filename));
    }
    else if (cmd == PCIE_CMD_READ)
    {
      printf("[%s] %s READ\n", __FUNCTION__, channel->filename);
    }

    #warning send RESPONSE with CMD result

  }
  printf("[%s] filename:%s bye\n", __FUNCTION__, channel->filename);
  close(channel->file_fd);
  close(channel->pipe_fd[0]);
  #warning send RESPONSE to close file
}

int main(int argc, char *argv[])
{
  int rv = 0;
  dbg(0, debug, "welcome!\n");
  // int command = -1;

  init_channels();

  while (1)
  {
    int command = -1;
    int handler = -1;
    // pcie_cmd_t cmd;
    // rv = pcie_srv_cmd_wait();

    printf("enter command: ");
    scanf("%d", &command);
    printf("command: %d\n", command);

    switch (command)
    {
      /* allocate resources for child process and fork */
      case PCIE_CMD_OPEN:
      {
        int channel_idx = find_free_channel();
        printf("free channel idx: %d\n", channel_idx);
        if (channel_idx >= 0)
        {
          pcie_channel_t *channel_ptr = &pcie_channel_table[channel_idx];
          int pid = 0;

          channel_ptr->handler = channel_idx;

          memset(channel_ptr->filename, 0, sizeof(channel_ptr->filename));
          printf("enter filename: ");
          scanf("%s", channel_ptr->filename);
          printf("filename: %s\n", channel_ptr->filename);

          channel_ptr->mode    = PCIE_MODE_WRITE;
          channel_ptr->pid = 0;

          /* name of semaphore is "pSem", semaphore is reached using this name */
          channel_ptr->sem = sem_open ("pSem", O_CREAT | O_EXCL, 0644, 0);
          /* unlink prevents the semaphore existing forever */
          sem_unlink ("pSem");
          pipe(channel_ptr->pipe_fd);

          pid = fork();
          if (pid == 0)
          {
            forked_channel_process(channel_ptr);
          }
          else
          {
            // close the read-end of the pipe, I'm not going to use it;
            close(channel_ptr->pipe_fd[0]);
            channel_ptr->pid = pid;
          }
        }
        break;
      }

      default:
      {
        printf("enter handler: ");
        scanf("%d", &handler);
        printf("channel[%d].handler:%d\n", handler, pcie_channel_table[handler].handler);
        if (handler >= 0 && handler < 10 && pcie_channel_table[handler].handler > -1)
        {
          pcie_channel_t *channel_ptr = &pcie_channel_table[handler];

          printf("handler: %d wakeup!!\n", handler);
          write(channel_ptr->pipe_fd[1], &command, sizeof(command));
          sem_post(channel_ptr->sem);

          /* close command arrived */
          if (command == PCIE_CMD_CLOSE)
          {
            close(channel_ptr->pipe_fd[1]);
            channel_ptr->handler = -1;
            sem_destroy(channel_ptr->sem);
          }
        }

        break;
      }
    }
#if 0
    if (command == PCIE_CMD_OPEN)
    {
      int channel_idx = find_free_channel();
      printf("free channel idx: %d\n", channel_idx);
      if (channel_idx >= 0)
      {
        int pid = 0;
        printf("enter filename: ");
        memset(filename, 0, sizeof(filename));
        scanf("%s", filename);
        printf("filename: %s\n", filename);
        channel[channel_idx].handler = channel_idx;
        channel[channel_idx].mode    = PCIE_MODE_WRITE;
        strncpy(channel[channel_idx].filename, filename, sizeof(channel[channel_idx].filename));

        channel[channel_idx].pid = 0;
        channel[channel_idx].sem = sem_open ("pSem", O_CREAT | O_EXCL, 0644, 0);
        /* name of semaphore is "pSem", semaphore is reached using this name */
        sem_unlink ("pSem");
        /* unlink prevents the semaphore existing forever */

        pipe(channel[channel_idx].pipe_fd);

        pid = fork();
        if (pid == 0)
        {
          forked_channeless(&channel[channel_idx]);
        }
        else
        {
          close(channel[channel_idx].pipe_fd[0]); // close the read-end of the pipe, I'm not going to use it;
          channel[channel_idx].pid = pid;
        }
      }
    }
    else
    {
      /* other command
      printf("enter handler: ");
      scanf("%d", &handler);
      printf("handler: %d channel[%d].handler:%d\n", handler, handler, channel[handler].handler);
      if (channel[handler].handler > -1)
      {
        printf("handler: %d wakeup!!\n", handler);
        write(channel[handler].pipe_fd[1], &command, sizeof(command));
        sem_post(channel[handler].sem);

        /* close command arrived */
        if (command == PCIE_CMD_CLOSE)
        {
          close(channel[handler].pipe_fd[1]);
          channel->handler = -1;
          sem_destroy(channel->sem);
        }
      }
    }
#endif
  }

  dbg(0, debug, "bye!\n");
}


