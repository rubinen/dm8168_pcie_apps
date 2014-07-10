#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>

#include <pcie_server_lib.h>
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


#define PCIE_CHANNELS_CNT     10

typedef struct pcie_channel_s {
  int             pcie_chan_handler; // pcie channel handler
  void *          ep_ptr;            // ep
  int             mode;              //
  char            filename[128];     // filename
  pid_t           pid;        // pid of the child process
  sem_t          *sem;        // semaphore
  int             pipe_fd[2]; // pipe
} pcie_channel_t;


static int debug = 0;

static int process_cnt = 0;
static pcie_channel_t pcie_channel_table[PCIE_CHANNELS_CNT] = {-1,};

/******************************************************************************\

\******************************************************************************/
static void pcie_channel_table_init(void)
{
  int i = 0;
  memset(pcie_channel_table, 0, sizeof(pcie_channel_table));

  for (i = 0; i < PCIE_CHANNELS_CNT; i++)
  {
    pcie_channel_table[i].pcie_chan_handler = -1;
  }
}

/******************************************************************************\

\******************************************************************************/
static pcie_channel_t* pcie_get_free_channel(pcie_cmd_t *pcie_cmd_ptr)
{
  pcie_channel_t *channel_ptr = NULL;
  int idx = 0;

  for (idx = 0; idx < PCIE_CHANNELS_CNT; idx++)
  {
    if (pcie_channel_table[idx].pcie_chan_handler == -1)
    {
      channel_ptr = &pcie_channel_table[idx];
      break;
    }

  }

  if (channel_ptr)
  {
    channel_ptr->pcie_chan_handler = idx;

    memset(channel_ptr->filename, 0, sizeof(channel_ptr->filename));
    dbgi("pcie_cmd ep_ptr:%p len:%d\n", pcie_cmd_ptr->ep_ptr, pcie_cmd_ptr->len);
    channel_ptr->ep_ptr = pcie_cmd_ptr->ep_ptr;
    if (pcie_cmd_ptr->len)
    {
      pcie_cmd_open_t *cmd_open = (pcie_cmd_open_t *) pcie_cmd_ptr->data;
      dbgi("cmd_open mode:%d name:%s\n", cmd_open->mode, cmd_open->name);
      memcpy(channel_ptr->filename, cmd_open->name, strlen(cmd_open->name));
    }
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
  pcie_cmd_t pcie_cmd = {NULL, -1};
  pcie_response_t response =
  {
    .command = PCIE_CMD_OPEN,
    .result = -1
  };
  void*  file_mapping_ptr = NULL; // file mapping
  int    file_offset      = 0;    // file offset
  int    file_mapping_len = 0;    // mapping length

  if (channel && strlen(channel->filename) && (channel->pcie_chan_handler >= 0))
  {
    errno = 0;
    fd = open(channel->filename, O_RDWR | O_CREAT, 00666);
    dbg(0, debug, "handler:%d open(%s) file_fd: %d errno:%d (%s)\n",
          channel->pcie_chan_handler, channel->filename, fd, errno, strerror(errno));
    if (fd > 0)
    {
      response.pcie_chan_handler = channel->pcie_chan_handler;
      response.result = 0;
    }
  }

  /* response to OPEN command */
  PCIE_SRV_CmdResponse(channel->ep_ptr, &response);

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

        close(fd);

        dbgi("[%s] %s pipe cmd close handler:%d\n", __FUNCTION__, channel->filename, cmd_close->pcie_chan_handler);
      }
      else if ((pcie_cmd.command == PCIE_CMD_WRITE) || (pcie_cmd.command == PCIE_CMD_READ))
      {
        pcie_cmd_rdwr_t *cmd_rdwr = (pcie_cmd_rdwr_t *)pcie_cmd.data;

        /* length must be aligned to page size */
        if (cmd_rdwr->len > 0 && ALIGNED(cmd_rdwr->len, PAGE_SIZE))
        {
          file_mapping_ptr = mmap(NULL, cmd_rdwr->len, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, file_offset);
          if (file_mapping_ptr)
          {
            response.result = 0;
            if (pcie_cmd.command == PCIE_CMD_WRITE)
            {
              response.result = PCIE_SRV_Receive(channel->ep_ptr, file_mapping_ptr, cmd_rdwr->len);
            }
            else
            {
              response.result = PCIE_SRV_Send(channel->ep_ptr, file_mapping_ptr, cmd_rdwr->len);
            }

            dbgi("[%s] %s pipe cmd:%d handler:%d rd_len:%d result:%d \n", __FUNCTION__,
                channel->filename, pcie_cmd.command, cmd_rdwr->pcie_chan_handler, cmd_rdwr->len, response.result);

            if (response.result == cmd_rdwr->len)
            {
              file_offset += response.result;
            }

            munmap(file_mapping_ptr, cmd_rdwr->len);
          }
        }


      }
      // else if (pcie_cmd.command == PCIE_CMD_READ)
      // {
      //   pcie_cmd_read_t *cmd_read = (pcie_cmd_read_t *)pcie_cmd.data;

      //   response.result = PCIE_SRV_Send(channel->ep_ptr, buf, cmd_write->rd_len);

      //   dbgi("[%s] %s pipe cmd READ handler:%d rd_len:%d \n", __FUNCTION__,
      //       channel->filename, cmd_read->pcie_chan_handler, cmd_read->rd_len);
      // }
      else if (pcie_cmd.command == PCIE_CMD_LSEEK)
      {
        pcie_cmd_lseek_t *cmd_lseek = (pcie_cmd_lseek_t *)pcie_cmd.data;
        response.result = lseek(fd, cmd_lseek->offset, SEEK_SET);
      }
    }
    /* response to command command */
    PCIE_SRV_CmdResponse(channel->ep_ptr, &response);
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
  rv = PCIE_SRV_Init();

  dbgi("PCIE_SRV_Init rv:%d\n", rv);

  pcie_channel_table_init();

  while (1)
  {
    int pcie_chan_handler = -1;
    pcie_cmd_t pcie_cmd;
    pcie_channel_t *channel_ptr = NULL;

    rv = PCIE_SRV_CmdWait(&pcie_cmd);

    // dbgi("enter command: ");
    // scanf("%d", &pcie_cmd.command);
    // dbgi("command: %d\n", pcie_cmd.command);

    switch (pcie_cmd.command)
    {
      /* allocate resources for child process and fork */
      case PCIE_CMD_OPEN:
      {
        int pid = 0;
        channel_ptr = pcie_get_free_channel(&pcie_cmd);
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
        /* Get PCIE channel handler from pcie command that arrived */
        if ((pcie_cmd.command >= PCIE_CMD_CLOSE) && (pcie_cmd.command < PCIE_CMD_MAX))
        {

          dbgi("channel[%d].handler:%d\n", pcie_chan_handler, pcie_channel_table[pcie_chan_handler].pcie_chan_handler);
          dbgi("handler: %d wakeup!!\n", pcie_chan_handler);

          if (pcie_cmd.command == PCIE_CMD_CLOSE)
          {
            pcie_cmd_close_t *cmd_close = (pcie_cmd_close_t *)pcie_cmd.data;
            pcie_chan_handler = cmd_close->pcie_chan_handler;
          }
          else if ((pcie_cmd.command == PCIE_CMD_WRITE) || (pcie_cmd.command == PCIE_CMD_READ))
          {
            pcie_cmd_rdwr_t *cmd_rdwr = (pcie_cmd_rdwr_t *)pcie_cmd.data;
            pcie_chan_handler = cmd_rdwr->pcie_chan_handler;
          }
          else if (pcie_cmd.command == PCIE_CMD_LSEEK)
          {
            pcie_cmd_lseek_t *cmd_lseek = (pcie_cmd_lseek_t *)pcie_cmd.data;
            pcie_chan_handler = cmd_lseek->pcie_chan_handler;
          }
          dbgi("pcie_cmd  cmd:%d handler:%d \n", pcie_cmd.command, pcie_chan_handler);

          /* check if channel handler does not exceed channel table */
          if ((pcie_chan_handler >= 0) && (pcie_chan_handler < PCIE_CHANNELS_CNT))
          {
            pcie_channel_t *channel_ptr = &pcie_channel_table[pcie_chan_handler];

            /* pass command to the channel process */
            write(channel_ptr->pipe_fd[1], &pcie_cmd.command, sizeof(pcie_cmd.command));
            write(channel_ptr->pipe_fd[1], &pcie_cmd.len, sizeof(pcie_cmd.len));
            write(channel_ptr->pipe_fd[1], pcie_cmd.data, pcie_cmd.len);
            sem_post(channel_ptr->sem);

            /* close command arrived */
            if (pcie_cmd.command == PCIE_CMD_CLOSE)
            {
              close(channel_ptr->pipe_fd[1]);
              channel_ptr->pcie_chan_handler = -1;
              sem_destroy(channel_ptr->sem);
            }
          }
        }

        break;
      }
    }
  }

  dbg(0, debug, "bye!\n");
}


