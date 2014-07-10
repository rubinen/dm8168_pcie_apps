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
#include <poll.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>

#include "pcie_server_lib.h"

#include "ti81xx_mgmt_lib.h"
#include "ti81xx_pci_info.h"
#include "ti81xx_trans.h"
#include <drivers/char/ti81xx_pcie_rcdrv.h>

/******************************************************************************\
 *** DEFINES
\******************************************************************************/
#define APP_NAME     "lib_pcie_server"

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


#define PAGE_SIZE_EP  4096
#define USED          1
#define INPROCESS     1
#define CPU           2

#define EP_COUNT      2

enum {
  EP_STATE_WAIT       = 0,             /* RC waits for command from EP */
  EP_STATE_RD_RUNNING = 1,             /* EP reads from RC */
  EP_STATE_WR_RUNNING = 2,             /* EP writes to RC */
  EP_STATE_MAX
};

/******************************************************************************\
 *** TYPES
\******************************************************************************/
struct fix_trans {
  struct dedicated_buf bufd;
  char *user_buf;
  unsigned int size_buf;
};

typedef struct ep_context_s {
  int                      idx;               /* Index of the context table entry */
  int                      id;                /* Endpoint unique Id */
  int                      cli_no;            /* number of clients using this EP */
  struct pci_sys_info     *sys_info;
  struct ti81xx_start_addr_area start_addr;
  unsigned int             bar0_addr;

  char                    *mapped_buffer;
  int                      bar_num;
  char                    *mapped_pci;

  struct ti81xx_ptrs       ptr;
  unsigned int             int_cap;           /* Received interrupt capability */
  unsigned int             size_buf;

  struct fix_trans         rd_buf;
  struct fix_trans         wr_buf;
  int                      state;             /* current state of EP - temporary */

  sem_t                    wr_sem;            /* wr */
  int                      rx_bytes_left;     /* number of bytes to receive from EP */
} ep_context_t;

/******************************************************************************\
 *** LOCAL VARIABLES
\******************************************************************************/
static int                  debug = 0;

static ep_context_t         ep_context_table[EP_COUNT] = {0, };
static int                  ep_found = 0;
static unsigned long long   byte_recv = 0;

struct ti81xx_mgmt_area     global_mgmt_area = {0, };
static int                  mgmt_area_set = 0;
static int                  fd;
static struct pci_sys_info *start;
static struct pollfd        fds[1];


static pthread_t            wait_int_thread;
static pthread_attr_t       wait_int_thread_attr;

static sem_t                command_sem;        // cmd parse signal
static pcie_cmd_t           pcie_cmd = {0,};

int debug_test = 1;

/******************************************************************************\
 *** FUNCTION DECLARATIONS
\******************************************************************************/
void *push_data_in_local(void *);
void *send_data(void *);
void *wait_for_data(void *arg);
void *pcie_wait_for_int(void *arg);


/******************************************************************************\

\******************************************************************************/
static void send_data_by_cpu(ep_context_t *ep_ctx_ptr, char *data, int data_len)
{
  int ret;
  unsigned int offset;

ACCESS_MGMT:
  while ((ret = access_mgmt_area((u32 *) ep_ctx_ptr->mapped_pci,
            global_mgmt_area.my_unique_id)) < 0) {
    debug_print("mgmt area access not granted\n");
    sleep(3);
    continue;
  }

  while ((ret = get_free_buffer((u32 *)ep_ctx_ptr->mapped_pci)) < 0) {
    debug_print("buffer not available\n");
    release_mgmt_area((u32 *)ep_ctx_ptr->mapped_pci);
    sleep(2);
    goto ACCESS_MGMT;
  }

  offset = offset_to_buffer((u32 *)ep_ctx_ptr->mapped_pci, ret);
  send_to_remote_buf_by_cpu((u32 *)ep_ctx_ptr->mapped_pci, offset, ret, data, data_len);
  debug_print("send by cpu success full\n");
  release_mgmt_area((u32 *)ep_ctx_ptr->mapped_pci);
}

/******************************************************************************\

\******************************************************************************/
static void pcie_send_response(ep_context_t *ep_ctx_ptr, pcie_response_t *rsp)
{
  if (ep_ctx_ptr && rsp)
  {
    send_data_by_cpu(ep_ctx_ptr, (char*)rsp, sizeof(pcie_response_t));

    if (ep_ctx_ptr->int_cap == 1)
    {
      ioctl(fd, TI81XX_RC_SEND_MSI, ep_ctx_ptr->bar0_addr/*bar 0 of ep2 */);
    }

    debug_print("INFO: response sent \n");
  }
}


/******************************************************************************\

\******************************************************************************/
static int pcie_get_cmd(ep_context_t *ep_ctx_ptr)
{
  int rv = -1;
  int bytes_received = 0;

  dbgi("ep_ctx_ptr:%p idx:%d ptr:%p\n", ep_ctx_ptr, ep_ctx_ptr->idx, ep_ctx_ptr->ptr);

  ti81xx_poll_for_data_in_block(ep_ctx_ptr->idx * 2, &ep_ctx_ptr->ptr, &global_mgmt_area/*ep_ctx_ptr->mgmt_area*/,
          ep_ctx_ptr->mapped_buffer, &pcie_cmd.command, sizeof(pcie_cmd) - sizeof(void*), &bytes_received);
  dbgi("bytes_received:%d\n", bytes_received);

  if (bytes_received > 10)
  {
    dbgi("pcie_cmd cmd:%d len:%d\n", pcie_cmd.command, pcie_cmd.len);
    if ((pcie_cmd.len > 0) && (pcie_cmd.command >= PCIE_CMD_OPEN) && (pcie_cmd.command < PCIE_CMD_MAX))
    {
      pcie_cmd.ep_ptr = (void*)ep_ctx_ptr;
      rv = 0;
    }
  }
  if (rv != 0)
  {
    memset(&pcie_cmd, 0, sizeof(pcie_cmd));
  }
  return rv;
}

/******************************************************************************\

\******************************************************************************/
static int pcie_get_data(ep_context_t *ep_ctx_ptr)
{
  int rv = -1;
  int bytes_received = 0;

  dbgi("ep_ctx_ptr:%p idx:%d ptr:%p\n", ep_ctx_ptr, ep_ctx_ptr->idx, ep_ctx_ptr->ptr);

  ti81xx_poll_for_data_in_block(ep_ctx_ptr->idx * 2, &ep_ctx_ptr->ptr, &global_mgmt_area/*ep_ctx_ptr->mgmt_area*/,
          ep_ctx_ptr->mapped_buffer, &pcie_cmd.command, sizeof(pcie_cmd) - sizeof(void*), &bytes_received);
  dbgi("bytes_received:%d\n", bytes_received);

  if (bytes_received > 10)
  {
    dbgi("pcie_cmd cmd:%d len:%d\n", pcie_cmd.command, pcie_cmd.len);
    if ((pcie_cmd.len > 0) && (pcie_cmd.command >= PCIE_CMD_OPEN) && (pcie_cmd.command < PCIE_CMD_MAX))
    {
      pcie_cmd.ep_ptr = (void*)ep_ctx_ptr;
      rv = 0;
    }
  }
  if (rv != 0)
  {
    memset(&pcie_cmd, 0, sizeof(pcie_cmd));
  }
  return rv;
}


/******************************************************************************\

\******************************************************************************/
static int pcie_endpoint_init(ep_context_t *ep_ctx_ptr)
{
  int rv = 0;
  int  i;

  do
  {
    struct ti81xx_mgmt_area *buffer_mgmt_area = NULL;
    struct ti81xx_mgmt_area *pci_mgmt_area = NULL;

    if (!ep_ctx_ptr)
    {
      rv = -EINVAL;
      break;
    }

    debug_print(" %s EP id:%d bar0_addr:%08x idx:%d\n", __FUNCTION__, ep_ctx_ptr->id, ep_ctx_ptr->bar0_addr, ep_ctx_ptr->idx);

    rv = sem_init(&ep_ctx_ptr->wr_sem, 0, 1);
    if (rv < 0)
    {
      err_print("sem_init failed rv : %d\n", rv);
      break;
    }

    ep_ctx_ptr->int_cap = 0;
    ep_ctx_ptr->state   = EP_STATE_WAIT;

    ep_ctx_ptr->mapped_buffer = mmap(0, 8 * 1024 * 1024,
                    PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, (off_t)ep_ctx_ptr->start_addr.start_addr_phy);
    debug_print("mapped_buffer:%08x\n", ep_ctx_ptr->mapped_buffer);

    if ((void *) ep_ctx_ptr->mapped_buffer == (void *)-1)
    {
      err_print("MMAP of dedicated memory fail : %d\n", errno);
      rv = -errno;
      break;
    }

    buffer_mgmt_area = (struct ti81xx_mgmt_area *) ep_ctx_ptr->mapped_buffer;

    ti81xx_set_mgmt_area(&global_mgmt_area, (unsigned int *)ep_ctx_ptr->mapped_buffer);
    buffer_mgmt_area->unique_id    = 0;
    buffer_mgmt_area->my_unique_id = RC_UNIQ_ID;

    dedicate_buffer((unsigned int *)ep_ctx_ptr->mapped_buffer,
              ep_ctx_ptr->id, global_mgmt_area.no_blk, ep_ctx_ptr->idx * 2, RD);

    print_mgmt_area(__FUNCTION__, __LINE__, (unsigned int *)ep_ctx_ptr->mapped_buffer);

    dedicate_buffer((unsigned int *)ep_ctx_ptr->mapped_buffer,
              ep_ctx_ptr->id, global_mgmt_area.no_blk, (ep_ctx_ptr->idx * 2) + 1, WR);

    print_mgmt_area(__FUNCTION__, __LINE__, (unsigned int *)ep_ctx_ptr->mapped_buffer);

    debug_print("initialization of management mgmt_area complete\n");

    ti81xx_calculate_ptr(&global_mgmt_area, ep_ctx_ptr->mapped_buffer, &ep_ctx_ptr->ptr);

    ep_ctx_ptr->mapped_pci = mmap(0, ep_ctx_ptr->sys_info->res_value[ep_ctx_ptr->bar_num + 1][1],
                    PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, (off_t) ep_ctx_ptr->sys_info->res_value[ep_ctx_ptr->bar_num + 1][0]);

    printf("PCI mapping:  BAR%d size:%08x at addr:%08x mapped at %08x\n",
          ep_ctx_ptr->bar_num,
          ep_ctx_ptr->sys_info->res_value[ep_ctx_ptr->bar_num + 1][1],
          ep_ctx_ptr->sys_info->res_value[ep_ctx_ptr->bar_num + 1][0],
          ep_ctx_ptr->mapped_pci);

    if ((void *) ep_ctx_ptr->mapped_pci == (void *)-1)
    {
      err_print("MMAP EP's BAR  memory fail - %d\n", errno);
      rv = -errno;
      break;
    }
    pci_mgmt_area = (struct ti81xx_mgmt_area *) ep_ctx_ptr->mapped_pci;

    /* Advertise Interrupt capability,
       you will later wait for INT_CAP from EP when it joins */
    pci_mgmt_area->int_cap = INT_CAP;

  } while (0);

  return rv;
}

/******************************************************************************\

\******************************************************************************/
static int pcie_endpoint_init_finish(ep_context_t *ep_ctx_ptr)
{
  int rv = 0;
  do
  {
    struct ti81xx_mgmt_area *buffer_mgmt_area = NULL;
    if (!ep_ctx_ptr)
    {
      rv = -EINVAL;
      break;
    }
    buffer_mgmt_area = (struct ti81xx_mgmt_area *) ep_ctx_ptr->mapped_buffer;

    debug_print("EP[%d] id:%d int_cap received:%d\n", ep_ctx_ptr->idx, ep_ctx_ptr->id, buffer_mgmt_area->int_cap);

    if (buffer_mgmt_area->int_cap == INT_CAP)
    {
      ep_ctx_ptr->int_cap = INT_CAP;
      /*
        check for dedicated buf when interrupt comes, it would mean that some Endpoint
        attempts to communicate
      */
#ifdef INTEGRITY
      ep_ctx_ptr->wr_buf.size_buf = ep_ctx_ptr->size_buf;
      ep_ctx_ptr->wr_buf.user_buf = malloc(ep_ctx_ptr->size_buf);
      debug_print("WR buf on RC side size:%u address:%p\n",
             ep_ctx_ptr->wr_buf.size_buf, ep_ctx_ptr->wr_buf.size_buf);

      if (ep_ctx_ptr->wr_buf.user_buf == NULL)
      {
        rv = -ENOMEM;
        printf("user buffer allocation failed\n");
        break;
      }

      if (find_dedicated_buffer((unsigned int *)ep_ctx_ptr->mapped_pci,
          global_mgmt_area.my_unique_id, &ep_ctx_ptr->wr_buf.bufd, RD) < 0)
      {
        rv = -EIO;
        err_print("no dedicated buffer for TX on remote peer\n");
        break;;
      }

      debug_print("offset of buffer is %x\n", ep_ctx_ptr->wr_buf.bufd.off_st);
#endif

      ep_ctx_ptr->rd_buf.size_buf = ep_ctx_ptr->size_buf;
      ep_ctx_ptr->rd_buf.user_buf = NULL;

      if (find_dedicated_buffer((unsigned int *)ep_ctx_ptr->mapped_buffer,
                ep_ctx_ptr->id, &ep_ctx_ptr->rd_buf.bufd, WR) < 0)
      {
        rv = -EIO;
        err_print("no dedicated buffer on local peer to be RX "
                  "by remote peer\n");
        break;
      }
    }
  }
  while (0);

  return rv;
}

/******************************************************************************\

\******************************************************************************/
void *pcie_wait_for_int(void * arg)
{
  int rv = 0;

  while (1)
  {
    rv = poll(fds, 1, 3000); /*3 sec wait time out*/
    dbgi("poll rv:%d POLLIN:%d\n", rv, POLLIN);
    if (rv == POLLIN)
    {
      int ep_idx = 0;
      ep_context_t *ep_ctx_ptr = NULL;

      for (ep_idx = 0; ep_idx < ep_found; ep_idx++)
      {
        ep_ctx_ptr = &ep_context_table[ep_idx];

        dbgi("ep_idx:%d ep_found:%d ep_ctx_ptr: int_cap:%d idx:%d\n", ep_idx, ep_found, ep_ctx_ptr->int_cap, ep_ctx_ptr->idx);

        rv = 0;
        /* Check for interrupt capability of EP */
        if (ep_ctx_ptr->int_cap == 0)
        {
          rv = pcie_endpoint_init_finish(ep_ctx_ptr);
          dbgi("pcie_endpoint_init_finish rv:%d \n", rv);
        }

        dbgi("rv:%d int_cap:%d \n", rv, ep_ctx_ptr->int_cap);
        if ((rv == 0) && (ep_ctx_ptr->int_cap == INT_CAP))
        {
          dbgi("ep_ctx_ptr->state:%d \n", ep_ctx_ptr->state);
          if (ep_ctx_ptr->state == EP_STATE_WAIT)
          {
            if (pcie_get_cmd(ep_ctx_ptr) == 0)
            {
              dbgi("sem_post command_sem\n");
              sem_post(&command_sem);
            }
          }
          else if (ep_ctx_ptr->state == EP_STATE_WR_RUNNING)
          {
            if (pcie_get_data(ep_ctx_ptr) == 0)
            {
              dbgi("pcie_get_data rv:0 rx_bytes_left:%d\n", ep_ctx_ptr->rx_bytes_left);
            }
            /*
              EP writes to RC - notify receive function if enough data received.
            */
            if (ep_ctx_ptr->rx_bytes_left == 0)
            {
              sem_post(&ep_ctx_ptr->wr_sem);
            }
          }
          else
          {

          }
        }
      }
    }
    else
    {
      debug_print("no notification from remote peer -- "
              "timed out byte_recv\n");
    }
  }
}



/******************************************************************************\

\******************************************************************************/
int PCIE_SRV_Init(void)
{
  int  i;
  int eps = 0;
  unsigned int size_buf = 1024 * 1024;
  struct ti81xx_start_addr_area start_addr;
  unsigned int *test;
  struct pci_sys_info *temp;
  ep_context_t *ep_ctx_ptr = NULL;

  if (sem_init(&command_sem, 0, 0) < 0)
  {
    perror("semaphore initilization failed");
    exit(0);
  }

  fds[0].events = POLLIN;
  start = NULL;
  // bar0_addr = 0;
  byte_recv = 0;
  int bar_chosen = -1;

  fd = open("/dev/ti81xx_ep_hlpr", O_RDWR);
  if (fd == -1)
  {
    err_print("device file open fail\n");
    return -1;
  }


  fds[0].fd = fd;

  if (ioctl(fd, TI81XX_RC_START_ADDR_AREA, &start_addr) < 0)
  {
    err_print("ioctl START_ADDR failed\n");
    goto ERROR;
  }

  if (ti81xx_prepare_mgmt_info(&global_mgmt_area, size_buf) < 0)
  {
    err_print("prepare_mgmt_info failed\n");
    goto FREELIST;
  }

  global_mgmt_area.my_unique_id = RC_UNIQ_ID;

  eps = get_devices(&start);
  if (eps < 0)
  {
    err_print("fetching pci sub system info on rc fails\n");
    goto ERROR;
  }

  dbgi("no of ep in system is %u\n", eps);

  dbgi("start address of global_mgmt_area is virt--%x  phy--%x\n",
        start_addr.start_addr_virt,
            start_addr.start_addr_phy);
  // dbgi(start, fd, eps, start_addr.start_addr_phy);

  propagate_system_info(start, fd, eps, start_addr.start_addr_phy);

  debug_print("pci subsystem info propagated\n");
  print_list(start);


  ep_ctx_ptr = &ep_context_table[0];
  ep_found = 0;

  for (temp = start; temp != NULL; temp = temp->next)
  {
    debug_print("temp->res_value[0][0]:%d [0][1]:%d\n", temp->res_value[0][0], temp->res_value[0][1]);

    if ((temp->res_value[0][1] > 0) && (temp->res_value[0][1] <= 6))
    {
      ep_ctx_ptr->id        = temp->res_value[0][0];
      ep_ctx_ptr->bar_num   = temp->res_value[0][1];
      ep_ctx_ptr->bar0_addr = temp->res_value[1][0];

      printf("BAR%d used by EP id:%d \n", ep_ctx_ptr->bar_num, ep_ctx_ptr->id);
      printf("BAR%d address of EP is %x size is %x\n", ep_ctx_ptr->bar_num,
            temp->res_value[ep_ctx_ptr->bar_num + 1][0],
            temp->res_value[ep_ctx_ptr->bar_num + 1][1]);

      printf("BAR0 address is %x\n", ep_ctx_ptr->bar0_addr);

      ep_ctx_ptr->sys_info   = temp;
      ep_ctx_ptr->size_buf   = size_buf;
      ep_ctx_ptr->start_addr = start_addr;
      ep_ctx_ptr->idx        = ep_found;

      pcie_endpoint_init(ep_ctx_ptr);
      ep_found++;
      ep_ctx_ptr++;
    }
  }

  pthread_attr_init(&wait_int_thread_attr);

  pthread_create(&wait_int_thread, &wait_int_thread_attr, pcie_wait_for_int, NULL);
  // pthread_join(t2, NULL);

  return 0;
FREELIST:
  free_list(start);

ERROR:
  close(fd);
  return -1;
}


/******************************************************************************\
  Wait for command:
  ie.
  - open file ()
  - close file
\******************************************************************************/
int PCIE_SRV_CmdWait(pcie_cmd_t *cmd)
{
  dbgi("sem_wait command_sem...\n");
  sem_wait(&command_sem);
  dbgi("wake up command_sem...copy cmd:%d (%d+%d+%d)\n", sizeof(pcie_cmd.command) + sizeof(pcie_cmd.len) + pcie_cmd.len,
      sizeof(pcie_cmd.command), sizeof(pcie_cmd.len), pcie_cmd.len);
  memcpy(cmd, &pcie_cmd, sizeof(pcie_cmd.command) + sizeof(pcie_cmd.len) + pcie_cmd.len);
  return 0;
}

/******************************************************************************\
  Send response to command:
  ie.
  - open file status
  - close file status
  - number of bytes written
\******************************************************************************/
int PCIE_SRV_CmdResponse(void* ep_ptr, pcie_response_t *rsp)
{
  ep_context_t *ep_ctx_ptr = (ep_context_t *)ep_ptr;

  dbgi(" ep_ctx_ptr:%p response:%p \n", ep_ctx_ptr, rsp);
  if (ep_ctx_ptr/* && ep_ctx_ptr->state*/)
  {
    /* command completed, wait for another... */
    ep_ctx_ptr->state = EP_STATE_WAIT;
  }

  pcie_send_response(ep_ctx_ptr, rsp);

  return 0;
}

/******************************************************************************\
  Receive data from client
  return number of bytes read
\******************************************************************************/
int PCIE_SRV_Receive(void* ep_ptr, void *buf, int size)
{
  ep_context_t *ep_ctx_ptr = (ep_context_t *)ep_ptr;
  if (ep_ctx_ptr/* && ep_ctx_ptr->state*/)
  {
    ep_ctx_ptr->state = EP_STATE_WR_RUNNING;
    ep_ctx_ptr->rx_bytes_left = size;
    sem_wait(&ep_ctx_ptr->wr_sem);
  }

}

/******************************************************************************\
  Send data to client
\******************************************************************************/
int PCIE_SRV_Send(void* ep_ptr, void *buf, int size)
{
  ep_context_t *ep_ctx_ptr = (ep_context_t *)ep_ptr;
  if (ep_ctx_ptr && buf && size)
  {
    ep_ctx_ptr->state = EP_STATE_RD_RUNNING;

    send_data_by_cpu(ep_ctx_ptr, (char*)buf, size);

    if (ep_ctx_ptr->int_cap == 1)
    {
      ioctl(fd, TI81XX_RC_SEND_MSI, ep_ctx_ptr->bar0_addr/*bar 0 of ep2 */);
    }

    debug_print("INFO: data sent \n");
  }
}
