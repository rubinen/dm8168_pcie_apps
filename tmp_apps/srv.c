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
    if (debug >= (level)) {                           \
      printf("\n"APP_NAME" [%d] %s():%d ", getpid(), __FUNCTION__, __LINE__); \
      printf(fmt , ## arg); \
    } \
  } while (0)

#define dbgi(fmt, arg...)                \
  do {                                                \
      printf(APP_NAME" [%d] %s():%d ", getpid(), __FUNCTION__, __LINE__); \
      printf(fmt , ## arg); \
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

static void pcie_channel_table_init(void)
{
  int i = 0;
  memset(pcie_channel_table, 0, sizeof(pcie_channel_table));

  for (i = 0; i < 10; i++)
  {
    pcie_channel_table[i].handler = -1;
  }
}

/******************************************************************************\

\******************************************************************************/
static pcie_channel_t* pcie_get_free_channel(void)
{
  pcie_channel_t *channel_ptr = NULL;
  int idx = 0;

  for (idx = 0; idx < 10; idx++)
  {
    if (pcie_channel_table[idx].handler == -1)
    {
      channel_ptr = &pcie_channel_table[idx];
      break;
    }

  }
  if (channel_ptr)
  {
    channel_ptr->handler = idx;

    memset(channel_ptr->filename, 0, sizeof(channel_ptr->filename));
    dbgi("enter filename: ");
    scanf("%s", channel_ptr->filename);
    dbgi("filename: %s\n", channel_ptr->filename);

    channel_ptr->mode    = PCIE_MODE_WRITE;
    channel_ptr->pid = 0;

    /* name of semaphore is "pSem", semaphore is reached using this name */
    channel_ptr->sem = sem_open ("pSem", O_CREAT | O_EXCL, 0644, 0);
    /* unlink prevents the semaphore existing forever */
    sem_unlink ("pSem");
    pipe(channel_ptr->pipe_fd);
  }
  return channel_ptr;
}

/******************************************************************************\
  Channel open command creates a process,
  which:
  - opens a file to read/write
  - handles read, write, append commands
  - closes file upon close command
\******************************************************************************/
static int pcie_channel_process(pcie_channel_t *channel)
{
  int fd = -1;
  int len = 0;
  pcie_cmd_t pcie_cmd = {-1, };
  pcie_response_t response =
  {
    .command = PCIE_CMD_OPEN,
    .result = -1
  };

  if (channel && strlen(channel->filename) && (channel->handler >= 0))
  {
    errno = 0;
    fd = open(channel->filename, O_RDWR | O_CREAT, 00666);
    dbg(0, debug, "handler:%d open(%s) file_fd: %d errno:%d (%s)\n",
          channel->handler, channel->filename, fd, errno, strerror(errno));
    if (fd > 0)
    {
      response.handler = channel->handler;
      response.result = 0;
    }
  }

  /* response to OPEN command */
  // pcie_srv_cmd_response(&response);

  /* If OPEN failed then return */
  if (response.result != 0)
  {
    close(channel->pipe_fd[1]);
    close(channel->pipe_fd[0]);
    return;
  }

  // close the write-end of the pipe, I'm not going to use it
  close(channel->pipe_fd[1]);

  while (pcie_cmd.command != PCIE_CMD_CLOSE)
  {
    pcie_cmd.command = -1;

    sem_wait(channel->sem);

    len = read(channel->pipe_fd[0], &pcie_cmd.command, sizeof(pcie_cmd.command));
    response.command = pcie_cmd.command;

    if ((pcie_cmd.command < PCIE_CMD_CLOSE) &&
        (pcie_cmd.command >= PCIE_CMD_MAX))
    {
      dbgi("[%s] %s unknown command received :%d\n", __FUNCTION__, channel->filename, pcie_cmd.command);
      response.result = -1;
    }
    else
    {
      len = read(channel->pipe_fd[0], &pcie_cmd.len, sizeof(pcie_cmd.len));
      len = read(channel->pipe_fd[0], pcie_cmd.data, pcie_cmd.len);

      dbgi("[%s] %s pipe cmd:%d\n", __FUNCTION__, channel->filename, pcie_cmd.command);
      if (pcie_cmd.command == PCIE_CMD_CLOSE)
      {
        response.result = 0;
        pcie_cmd_close_t *cmd_close = (pcie_cmd_close_t *)pcie_cmd.data;
        dbgi("[%s] %s pipe cmd close handler:%d\n", __FUNCTION__, channel->filename, cmd_close->handler);
      }
      else if (pcie_cmd.command == PCIE_CMD_WRITE)
      {
        unsigned char c = 0;
        pcie_cmd_write_t *cmd_write = (pcie_cmd_write_t *)pcie_cmd.data;

        // response.result = pcie_server_receive(channel->handler, buf, cmd_write->wr_len);

        dbgi("[%s] %s pipe cmd WRITE handler:%d wr_len:%d \n", __FUNCTION__,
            channel->filename, cmd_write->handler, cmd_write->wr_len);
#if 0
        len = 0;
        // int pcie_server_receive(handler, void *buf, int size);
        while (len < cmd_write->wr_len)
        {
          write(fd, &c, sizeof(c));
          c++;
          len++;
        }
#endif
      }
      else if (pcie_cmd.command == PCIE_CMD_READ)
      {
        pcie_cmd_read_t *cmd_read = (pcie_cmd_read_t *)pcie_cmd.data;

        // response.result = pcie_server_send(channel->handler, buf, cmd_write->rd_len);

        dbgi("[%s] %s pipe cmd READ handler:%d rd_len:%d \n", __FUNCTION__,
            channel->filename, cmd_read->handler, cmd_read->rd_len);
      }
      else if (pcie_cmd.command == PCIE_CMD_LSEEK)
      {
        pcie_cmd_lseek_t *cmd_lseek = (pcie_cmd_lseek_t *)pcie_cmd.data;
        response.result = lseek(fd, cmd_lseek->offset, SEEK_SET);
      }
    }
    /* response to command command */
    // pcie_srv_cmd_response(&response);
  }
  dbgi("[%s] filename:%s bye\n", __FUNCTION__, channel->filename);
  close(fd);
  close(channel->pipe_fd[0]);
}



/******************************************************************************\

\******************************************************************************/
int main(int argc, char *argv[])
{
  int rv = 0;
  dbgi("welcome!\n");
  // int command = -1;

  pcie_channel_table_init();

  while (1)
  {
    int handler = -1;
    pcie_cmd_t pcie_cmd;
    pcie_channel_t *channel_ptr = NULL;
    // rv = pcie_srv_cmd_wait();

    dbgi("enter command: ");
    scanf("%d", &pcie_cmd.command);
    dbgi("command: %d\n", pcie_cmd.command);

    switch (pcie_cmd.command)
    {
      /* allocate resources for child process and fork */
      case PCIE_CMD_OPEN:
      {
        int pid = 0;
        channel_ptr = pcie_get_free_channel();
        dbgi("channel_ptr: %p\n", channel_ptr);

        pid = fork();
        if (pid == 0) // child process
        {
          // Child process sends responses to command.
          // Even if channel open failed, create child process
          // anyway, it will send OPEN fail result and terminate.
          pcie_channel_process(channel_ptr);
        }
        else // parent process
        {
          // check if channel initialized
          if (channel_ptr)
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
        dbgi("enter handler: ");
        scanf("%d", &handler);
        dbgi("handler:%d\n", handler);
        if (handler >= 0 && handler < 10 && pcie_channel_table[handler].handler > -1)
        {
          pcie_channel_t *channel_ptr = &pcie_channel_table[handler];

          dbgi("channel[%d].handler:%d\n", handler, pcie_channel_table[handler].handler);
          dbgi("handler: %d wakeup!!\n", handler);

          if (pcie_cmd.command == PCIE_CMD_CLOSE)
          {
            pcie_cmd_close_t *cmd_close = (pcie_cmd_close_t *)pcie_cmd.data;
            pcie_cmd.len = sizeof(pcie_cmd_close_t);
            cmd_close->handler = handler;
          }
          else if (pcie_cmd.command == PCIE_CMD_WRITE)
          {
            pcie_cmd_write_t *cmd_write = (pcie_cmd_write_t *)pcie_cmd.data;
            pcie_cmd.len = sizeof(pcie_cmd_write_t);
            dbgi("pcie_cmd len:%d!!\n", pcie_cmd.len);

            cmd_write->handler = handler;
            cmd_write->wr_len = (handler+1)*100;
            dbgi("pcie_cmd wr handler:%d wr_len:%d !!\n", cmd_write->handler, cmd_write->wr_len);
          }
          else if (pcie_cmd.command == PCIE_CMD_READ)
          {
            pcie_cmd_read_t *cmd_read = (pcie_cmd_read_t *)pcie_cmd.data;
            pcie_cmd.len = sizeof(pcie_cmd_read_t);
            cmd_read->handler = handler;
            cmd_read->rd_len = (handler+1)*100;
            dbgi("pcie_cmd rd handler:%d rr_len:%d !!\n", cmd_read->handler, cmd_read->rd_len);
          }

          write(channel_ptr->pipe_fd[1], &pcie_cmd.command, sizeof(pcie_cmd.command));
          write(channel_ptr->pipe_fd[1], &pcie_cmd.len, sizeof(pcie_cmd.len));
          write(channel_ptr->pipe_fd[1], pcie_cmd.data, pcie_cmd.len);
          sem_post(channel_ptr->sem);

          /* close command arrived */
          if (pcie_cmd.command == PCIE_CMD_CLOSE)
          {
            close(channel_ptr->pipe_fd[1]);
            channel_ptr->handler = -1;
            sem_destroy(channel_ptr->sem);
          }
        }

        break;
      }
    }
  }

  dbg(0, debug, "bye!\n");
}


