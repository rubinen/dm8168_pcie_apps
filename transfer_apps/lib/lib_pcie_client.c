#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <pthread.h>
#include <semaphore.h>
#include "ti81xx_ep_lib.h"
#include "ti81xx_mgmt_lib.h"
#include "ti81xx_trans.h"
#include "debug_msg.h"

#include <pcie_client_lib.h>

/******************************************************************************\
 *** DEFINES
\******************************************************************************/
#define APP_NAME     "lib_pcie_client"

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


unsigned long long byte_recv; /* send by remote peer */
unsigned long long byte_rx; /* read from remote peer */

/* information for dedicated buffer */
struct fix_trans {
  struct dedicated_buf bufd;
  unsigned int out_phy;
  char *user_buf;
  unsigned int size_buf;
};

/*
 * global variable ( each bit of this 32 bit integer represents
 * status of corresponding outbound region.
 * if bit is set -  free
 * else -- in use
 */

unsigned int    status;

/* other globals*/
char      *mapped_buffer;
char      *mapped_pci;
char      *mapped_dma, *mapped_dma_recv;
unsigned int    *rm_info;
struct ti81xx_mgmt_area  mgmt_area;
struct pollfd   fds[1];
int     fd_dma;
int     fd;
struct ti81xx_ptrs  ptr;
pthread_mutex_t mutex_int = PTHREAD_MUTEX_INITIALIZER;
int     counter;
sem_t     mutex;
unsigned int    intr_cap; /* interrupt capability of this peer */
unsigned int fin_time1, cur_time1, fin_time2, cur_time2;
struct dma_cnt_conf dma_cnt;
struct dma_buf_info dma_b;
int integrity_test;

int debug_test = 0;
int sleep_dur = 4;
int bar_chosen = 2;
static int ep_id = 22;

/**
 * scan_mgmt_area(): search local management area for the information(start
 * address of dedicated memory)
 * of remote peer having unique id represented by muid.
 * And fill up outbound details in structure accordingly.
 * @rm_info: pointer on locla management area for remote peer system
 * @muid: uniqued id of remote peer
 * @ob: pointer to outbound region staructure
 */

int scan_mgmt_area(unsigned int *rm_info, unsigned int muid,
            struct ti81xx_outb_region *ob)
{
  unsigned int *ep_info = rm_info + 2;
  int j = 0, i = 0;
  int bar_map;
  if (muid == 1) {
    ob->ob_offset_hi = 0;
    /*start address of management area on RC*/
    ob->ob_offset_idx = rm_info[1];
    return 0;
  }
  for (i = 1; i <= rm_info[0] ; i++) {

    if (ep_info[j] == muid) {
      bar_map = ep_info[j + 13];
      ob->ob_offset_hi = 0;
      ob->ob_offset_idx = ep_info[j + bar_map * 2 + 1];
      return 0;
    }
    j += 14;
  }

  return -1;
}

/******************************************************************************\

\******************************************************************************/
static void send_data_by_cpu(char *map_pci, char *data, int data_len)
{
  int ret;
  unsigned int offset;

ACCESS_MGMT:
  while ((ret = access_mgmt_area((u32 *) map_pci,
            mgmt_area.my_unique_id)) < 0) {
    debug_print("mgmt area access not granted\n");
    sleep(3);
    continue;
  }

  while ((ret = get_free_buffer((u32 *)map_pci)) < 0) {
    debug_print("buffer not available\n");
    release_mgmt_area((u32 *)map_pci);
    sleep(2);
    goto ACCESS_MGMT;
  }

  offset = offset_to_buffer((u32 *)map_pci, ret);
  send_to_remote_buf_by_cpu((u32 *)map_pci, offset, ret, data, data_len);
  debug_print("send by cpu success full\n");
  release_mgmt_area((u32 *)map_pci);
}



void *send_to_dedicated_buf(void *arg)
{

  struct fix_trans *tx = (struct fix_trans *) arg;
  struct dma_info info;
  int count;

  unsigned int chunk_trans;
  int ret;
  info.size = tx->size_buf;
  info.user_buf = (unsigned char *)tx->user_buf;
  info.src = 0;
  info.dest = (unsigned int *)(tx->out_phy + (tx->bufd).off_st);
  /* take size_buf to be multiple of page size */
  chunk_trans = (1024 * 1024) / (tx->size_buf);

//#ifdef THPT
  /* 5BG devided by buffer size */
  chunk_trans = 5368709120llu / tx->size_buf;
  debug_print("total iteration to send data will be %u\n", chunk_trans);
  memset(mapped_dma, 78, tx->size_buf);

//#endif

  ioctl(fd, TI81XX_CUR_TIME, &cur_time1);

  for (count = 0; count < chunk_trans; count++) {


//#ifdef THPT
    while (*((tx->bufd).wr_ptr) != 0)
      /*debug_print("waiting for wr index to be zero\n");*/

    if (count == chunk_trans-1)
      memset(mapped_dma, 78, tx->size_buf);
    ret = ioctl(fd_dma, TI81XX_EDMA_WRITEM, &info);
    if (ret < 0) {
      err_print("edma ioctl failed, error in dma data "
                "transfer\n");
      pthread_exit(NULL);
    }
    *((tx->bufd).wr_ptr) = ret;
    ioctl(fd, TI81XX_SEND_MSI, 0);
// #endif
  }
  ioctl(fd, TI81XX_CUR_TIME, &fin_time1);
// #ifdef THPT
  debug_print("tx test case executed with tx %llu bytes in  %u jiffies\n",
          5368709120llu, fin_time1 - cur_time1);
  // printf("THPT calculated in TX direction is : %f MBPS\n",
  //     (float)((5.0 * 1024 * HZ) / (fin_time1 - cur_time1)));
// #endif

}

/* read data from a dedicated buffer on remote peer */
void *read_from_dedicated_buf(void *arg)
{
  struct fix_trans *rx = (struct fix_trans *)arg;
  struct dma_info info;
  int count;
  unsigned int chunk_trans;
  int ret;
  info.size = rx->size_buf;
  info.user_buf = (unsigned char *)rx->user_buf;
  info.dest = 0;
  info.src = (unsigned int *)(rx->out_phy + (rx->bufd).off_st);
  /* take size_buf to be multiple of page size */
  chunk_trans = (1024 * 1024) / (rx->size_buf);

// #ifdef THPT
  chunk_trans = 5368709120llu / rx->size_buf; /* 5GB devided by buf size*/
  debug_print("total iteration to read data will be %u\n", chunk_trans);
  byte_rx = 0;
// #endif

  ioctl(fd, TI81XX_CUR_TIME, &cur_time2);
  for (count = 0; count < chunk_trans; count++) {

// #ifdef THPT

    /*while (*((rx->bufd).wr_ptr) == 0);*/
    if (count == chunk_trans-1) {
      memset(rx->user_buf, 0, rx->size_buf);
      debug_print("last read\n");
    }
    ret = ioctl(fd_dma, TI81XX_EDMA_READM, &info);
    if (ret < 0) {
      err_print("edma ioctl failed, error in dma "
              "data transfer\n");
      pthread_exit(NULL);
    }
    /*ioctl(fd, TI81XX_SEND_MSI, 0);
      ((rx->bufd).wr_ptr) = 0;*/
    byte_rx += ret;
    if (byte_rx == 0x140000000llu) {
      debug_print("read from dedicated buffer completed\n");
      break;
    }

// #endif
  }

  ioctl(fd, TI81XX_CUR_TIME, &fin_time2);
  debug_print("RX test case executed with receving %llu bytes in "
        "%u jiffies\n", byte_rx, fin_time2-cur_time2);
}

int pcie_cli_init(int id)
{
  /*by default my_unique id has been set by driver and uniqueid is
  * set to lock management area at startup.
  */

  int ret;
  unsigned int ob_size;
  struct ti81xx_pcie_mem_info start_addr;
  struct ti81xx_inb_window in;
  struct ti81xx_outb_region ob;
  unsigned int *test;
  unsigned int *int_cap;
  unsigned int muid;
  unsigned int size_buf = 1024 * 1024;

  dma_cnt.acnt = 256;
  dma_cnt.bcnt = 4096;
  dma_cnt.ccnt = 1;
  dma_cnt.mode = 1;

  struct ti81xx_pciess_regs pcie_regs;

  struct fix_trans rd_buf, wr_buf;
  void *send_to_dedicated_buf(void *arg);
  void *read_frm_dedicate_buf(void *arg);
  void *cpu_utilize(void *arg);

  pthread_t t1, t2, t3, t4;

  void *wait_for_int(void *);
  void *send_data(void *);
  void *wait_for_data(void *arg);
  void *process_rmt_buffer(void *arg);
  void *process_local_buffer(void *arg);

  pthread_attr_t attr_t1, attr_t2, attr_t3, attr_t4;

  pthread_attr_init(&attr_t1);
  pthread_attr_init(&attr_t2);
  pthread_attr_init(&attr_t3);
  pthread_attr_init(&attr_t4);

  fds[0].events = POLLIN;
  byte_recv = 0;
  byte_rx = 0;
  status = 0;
  counter = 0;

  if (sem_init(&mutex, 0, 1) < 0)
  {
    perror("semaphore initilization failed");
    exit(0);
  }
  ep_id = id;
  printf("debugs: %d\n", debug_test);
  printf("EP id :%d\n", ep_id);
  printf("BAR%d used\n", bar_chosen);

  fd = open("/dev/ti81xx_pcie_ep", O_RDWR);
  if (fd == -1)
  {
    err_print("EP device file open fail\n");
    return -1;
  }

  /*local management area lock has been hold by EP itself.*/


  ret = ioctl(fd, TI81XX_GET_PCIE_MEM_INFO, &start_addr);

  if (ret < 0)
  {
    err_print("START_MGMT_AREA ioctl failed\n");
    close(fd);
    return -1;
  }

  if (!start_addr.size || (start_addr.size < SIZE_AREA))
  {
    if (!start_addr.size)
      err_print("No reserved memory available for PCIe "
          "transfers, quitting...\n");
    else
      err_print("Minimum %#x bytes required as reserved "
          "memory, quitting...\n", SIZE_AREA);

    close(fd);
    return -1;
  }

  dma_b.size = 0x100000;
  dma_b.send_buf = (unsigned int)start_addr.base + 0x6000000;
  dma_b.recv_buf = (unsigned int)dma_b.send_buf + dma_b.size;

  mapped_buffer = (char *)mmap(0, SIZE_AREA, PROT_READ|PROT_WRITE,
              MAP_SHARED, fd ,
          (off_t) start_addr.base);
  debug_print("start_addr.base:%08x (%08x) mapped_buffer:%08x\n", start_addr.base, SIZE_AREA, mapped_buffer);
  if ((void *)-1 == (void *) mapped_buffer)
  {
    err_print("mapping dedicated memory fail\n");
    close(fd);
    return -1;

  }

  memset(mapped_buffer, 0x0, SIZE_AREA);

  test = (unsigned int *)mapped_buffer;

  pcie_regs.offset = GPR0;
  pcie_regs.mode = GET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &pcie_regs) < 0)
  {
    err_print("GET_REGS mode ioctl failed\n unable to fetch "
            "unique id from GPR0\n");
    close(fd);
    return -1;
  }

  if (pcie_regs.value < 2)
  {
    err_print("Not a valid id assigned to it\n still continue by "
          "manually assigning valid id -- 2\n");
  }
  /******* by default uid 2 is assigned to This EP ****/

  test[0] = bar_chosen;
  test[1] = ep_id;
  fd_dma = open("/dev/ti81xx_edma_ep", O_RDWR);
  if (fd_dma == -1)
  {
    err_print("EP DMA device file open fail\n");
    close(fd);
    return -1;
  }

  ret = ioctl(fd_dma, TI81XX_EDMA_SET_CNT, &dma_cnt);
  if (ret < 0)
  {
    debug_print("dma count setting ioctl failed\n");
    goto ERROR;
  }
  ret = ioctl(fd_dma, TI81XX_EDMA_SET_BUF_INFO, &dma_b);
  if (ret < 0)
  {
    debug_print("dma buffer setting ioctl failed\n");
    goto ERROR;
  }

  fds[0].fd = fd;

  debug_print("INFO: start address of mgmt_area is virt--%x  phy--%x\n",
      start_addr.base, start_addr.base);


  /*inbound setup to be done for inbound to be enabled. by default BAR 2*/

  pcie_regs.offset = LOCAL_CONFIG_OFFSET + (BAR0 + 0x4 * bar_chosen);
  printf("PCIE_Regs  offset: %08x (BAR%d offs: %08x) \n", pcie_regs.offset, bar_chosen, (BAR0 + 0x4 * bar_chosen));
  pcie_regs.mode = GET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &pcie_regs) < 0)
  {
    err_print("GET_REGS mode ioctl failed\n");
    goto ERROR;
  }

  in.BAR_num = bar_chosen;
  /*by default BAR2 will be used to get
  * inbound access by RC.
  */
  in.internal_addr = start_addr.base;
  in.ib_start_hi = 0;
  in.ib_start_lo = pcie_regs.value;

  if (ti81xx_set_inbound(&in, fd) < 0)
  {
    err_print("setting in bound config failed\n");
    goto ERROR;
  }

  if (ti81xx_enable_in_translation(fd) < 0)
  {
    err_print("enable in bound failed\n");
    goto ERROR;
  }

  /*inbound setup complete.*/

  mapped_pci = (char *)mmap(0, PCIE_NON_PREFETCH_SIZE,
        PROT_READ|PROT_WRITE, MAP_SHARED,
            fd, (off_t)0x20000000);
  if ((void *)-1 == (void *) mapped_pci)
  {
    err_print("mapping PCIE_NON_PREFETCH memory fail\n");
    goto ERROR;
  }


  mapped_dma = (char *)mmap(0, dma_b.size,
        PROT_READ|PROT_WRITE, MAP_SHARED,
          fd, (off_t)dma_b.send_buf);
  if ((void *)-1 == (void *) mapped_dma)
  {
    err_print("mapping DMA tx memory fail\n");
    goto ERROR;
  }

  mapped_dma_recv = (char *)mmap(0, dma_b.size, PROT_READ|PROT_WRITE,
          MAP_SHARED, fd, (off_t)dma_b.recv_buf);
  if ((void *)-1 == (void *) mapped_dma)
  {
    err_print("mapping DMA rx memory fail\n");
    goto ERROR;
  }

  int_cap = (unsigned int *)mapped_pci;

  if (ti81xx_prepare_mgmt_info(&mgmt_area, size_buf) < 0)
  {
    err_print("prepare_mgmt_info failed\n");
    goto ERROR;
  }

  debug_print("mgmt_area.size:%08x\n", mgmt_area.size);

  rm_info = (u32 *)(mapped_buffer + mgmt_area.size);

  ti81xx_set_mgmt_area(&mgmt_area, (unsigned int *)mapped_buffer);
  debug_print("initialization of management mgmt_area complete\n");
#ifdef INTEGRITY
  dedicate_buffer((unsigned int *)mapped_buffer, RC_UNIQ_ID,
            mgmt_area.no_blk, 0, RD);
#endif

  print_mgmt_area(__FUNCTION__, __LINE__, (unsigned int *)mapped_buffer);

  test[0] = RC_UNIQ_ID; /*set lock to be accessed by RC*/
  mgmt_area.my_unique_id = test[1];

  while ((ret = access_mgmt_area((u32 *)mapped_buffer,
            test[1])) < 0)
  {
    /*trying to get access to it's own management area.*/
    debug_print("mgmt area access not granted\n");
    sleep(1);
    continue;
  }


  debug_print("total no of ep on system is %u\n", rm_info[0]);
  debug_print("Mgmt area's start address on RC is %08x\n", rm_info[1]);



  ti81xx_calculate_ptr(&mgmt_area, mapped_buffer, &ptr);
  ob_size = 0; /* by default assuming 1 MB outbound window */

  ret = ioctl(fd, TI81XX_SET_OUTBOUND_SIZE, ob_size);

  muid = 1;
  /* RC unique id -- assuming communcation have to be
  * done with remote peer having muid=1
  */

  if ((scan_mgmt_area(rm_info, muid, &ob) == -1))
  {
    /*this function will fillup outbound structure accordingly.*/
    debug_print("no remote peer of this muid exist\n");
    /*handle error free resource or continue without outbound.*/
  }

  debug_print("outbound config is %x %x\n",
          ob.ob_offset_hi, ob.ob_offset_idx);
  ob.size = 4194304;/* 4 MB*/




  ti81xx_set_outbound_region(&ob, fd); /*outbound will be setup here.*/


  if (ti81xx_enable_out_translation(fd) < 0)
  {
    err_print("enable outbound failed\n");
    goto ERROR;
  }


  if (ti81xx_enable_bus_master(fd) < 0)
  {
    err_print("enable bus master fail\n");
    goto ERROR;
  }

  mgmt_area.size = test[4]; /* updated by rc application */

  release_mgmt_area((u32 *)mapped_buffer); /*release management area */

  while (test[5] == 0)
  {
    debug_print("INFO: waiting for interrupt capability "
          "info from remote peer\n");
    sleep(1);
    continue;
  }

  printf("***Advertised interrupt capability, EP will generate MSI.\n");
  intr_cap = 1;


  int_cap[5] = intr_cap;


  /* user buffer allocation */
  rd_buf.size_buf = size_buf;
  rd_buf.out_phy = 0x20000000;
  wr_buf.size_buf = size_buf;
  wr_buf.out_phy = 0x20000000;

  rd_buf.user_buf = malloc(size_buf);
  if (rd_buf.user_buf == NULL)
  {
    printf("user buffer allocation failed\n");
    goto ERROR;
  }

  wr_buf.user_buf = malloc(size_buf);
  if (wr_buf.user_buf == NULL)
  {
    printf("user buffer allocation failed\n");
    free(rd_buf.user_buf);
    goto ERROR;
  }

  memset(rd_buf.user_buf, 66, size_buf);
  memset(wr_buf.user_buf, 66, size_buf);

  print_mgmt_area(__FUNCTION__, __LINE__, (unsigned int *)mapped_pci);

  /* find dedicated buffer for reading */
  if (find_dedicated_buffer((unsigned int *)mapped_pci,
        mgmt_area.my_unique_id, &rd_buf.bufd, WR) < 0)
  {
    err_print("no dedicated buffer for RX on remote peer\n");
    goto FREE_BUFFER;
  }
  debug_print("offset of buffer dedicated for RX on "
        "remote peer is %X\n", rd_buf.bufd.off_st);

  /* find dedicated buffer for transmitting data */
  if (find_dedicated_buffer((unsigned int *)mapped_pci,
        mgmt_area.my_unique_id, &wr_buf.bufd, RD) < 0)
  {
    err_print("no dedicated buffer for TX on remote peer\n");
    goto FREE_BUFFER;
  }

  debug_print("offset of buffer dedicated for TX on remote "
          "peer is  %X\n", wr_buf.bufd.off_st);


  return 0;
#if 0//def THPT

  printf("Executing test case THPT TX (write to RC from EP)\n");
  /*pthread_create(&t2, &attr_t2, cpu_utilize, NULL);*/
  pthread_create(&t1, &attr_t1, send_to_dedicated_buf, &wr_buf);
  pthread_join(t1, NULL);
  sleep(4);

  printf("Executing test case THPT RX (RC read from EP)\n");
  pthread_create(&t4, &attr_t4, read_from_dedicated_buf, &rd_buf);
  pthread_join(t4, NULL);
  sleep(4);

  printf("Executing test case THPT TX/RX "
      "(simultaneous read & write from EP)\n");
  pthread_create(&t4, &attr_t4, read_from_dedicated_buf, &rd_buf);
  pthread_create(&t1, &attr_t1, send_to_dedicated_buf, &wr_buf);
  pthread_join(t1, NULL);
  pthread_join(t4, NULL);

#endif

  goto CLEAR_MAP;

  if (test[5] == INT_CAP) {
    debug_print("INTERRUPT will be working\n");
    sleep(5);
    pthread_create(&t1, &attr_t1, send_data, &fd);
    pthread_create(&t2, &attr_t2, wait_for_int, NULL);
    pthread_create(&t3, &attr_t3, process_local_buffer, NULL);
    pthread_create(&t4, &attr_t4, process_rmt_buffer, &fd);
    /*send interrupt to indicate rmt buffer to recycle this buffer*/
  } else {
    debug_print("polling will be working\n");
    pthread_create(&t1, &attr_t1, send_data, &fd);
    pthread_create(&t2, &attr_t2, wait_for_data, NULL);
  }

  /* thread running infinite loops, in Demo mode they never joined*/

  pthread_join(t1, NULL);
  pthread_join(t2, NULL);
  pthread_join(t3, NULL);
  /* never executed  in demo mode*/
CLEAR_MAP:

FREE_BUFFER:
  free(wr_buf.user_buf);
  free(rd_buf.user_buf);

ERROR:
  close(fd);
  close(fd_dma);

  /* additional tests */
  return 0;
}

int pcie_cli_open(const char *name, int flags)
{
  int rv = 0;
  pcie_cmd_t cmd;
  pcie_cmd_open_t *cmd_open = NULL;
  dbgi("open file:%s flags:0x%x\n", name, flags);

  if (name)
  {
    cmd.ep_ptr = NULL;
    cmd.command = PCIE_CMD_OPEN;
    cmd.len = sizeof(pcie_cmd_open_t);
    cmd_open = (pcie_cmd_open_t *)cmd.data;

    cmd_open->mode = PCIE_MODE_WRITE;
    memset(cmd_open->name, 0, CMD_OPEN_LEN_MAX);
    memcpy(cmd_open->name, name, strlen(name));

    dbgi("send_data_by_cpu mapped_pci:%p data:%p len:%d\n", mapped_pci, &cmd, sizeof(pcie_cmd_t));
    send_data_by_cpu(mapped_pci, (char*)&cmd, sizeof(pcie_cmd_t));
    dbgi("ioctl TI81XX_SEND_MSI\n");
    rv = ioctl(fd, TI81XX_SEND_MSI, 0);
    dbgi("ioctl TI81XX_SEND_MSI rv:%d\n");

  }
  return 0;
}
int pcie_cli_read(int fd, void *buf, int size)
{

  return 0;
}

int pcie_cli_write(int fd, void *buf, int size)
{
  return 0;
}
int pcie_cli_close(int fd)
{
  return 0;
}
