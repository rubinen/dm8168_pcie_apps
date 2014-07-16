#include "ti81xx_mgmt_lib.h"
#include "ti81xx_trans.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <string.h>
#include "debug_msg.h"

extern char pattern[200];
/**
 * access_mgmt_area()-- this function try to get controll of management area.
 * @mgmt_area: value returned by mmap on that particular management area.
 * for local management area it will be pointer to corresponding management
 * area in local memory ( mapped_buffer).
 * for remote management area it will be pointer to a address
 * ( 0x20000000-0x2fffffff )(mapped_pci).
 * @unique_id: unique id field of caller.
 *
 * return 0 if access is granted.
 */

int access_mgmt_area(unsigned int *mgmt_area, unsigned int unique_id)
{
  unsigned int id;

  debug_print("reading unique id from management area\n");
  memcpy(&id, mgmt_area, sizeof(unsigned int));

  debug_print("unique id field in mgmt area is :  %u\n", id);

  if (id == 0) {
    debug_print("no one  is accessing this ,management area, "
              "try to get access\n");
  } else if (id == unique_id) {
    debug_print("you already have access to it\n");
    return 0;
  } else {
    debug_print("someone else is accessing this management area\n");
    return -1;
  }

  mgmt_area[0] = unique_id;

  if (mgmt_area[0] == unique_id) { /* checking access granted or not. */
    debug_print("access is granted to you\n");
    return 0;
  } else
    debug_print("some one else get access to management area\n");

  return -1;
}



inline void release_mgmt_area(unsigned int *mgmt_area)
{
  mgmt_area[0] = 0;
  debug_print("access to management area released\n");
}

/**
 * get_free_buffer()-- try to get a free buffer and if there is one,
 * put it's status to INPROCESS in free Q.
 * @mgmt_area:  value returned by mmap on that particular management area.
 * for local management area it will be pointer to corresponding management
 * area in local memory ( mapped_buffer).
 * for remote management area it will be pointer to a address
 * ( 0x20000000-0x2fffffff ).(mapped_pci).
 *
 * __NOTE__: This function is called after getting access to
 * management area by access_mgmt_area()
 *
 * return buffer_no on success. otherwise return -1 on failure.
 */
/* overhead malloc */
int get_free_buffer(unsigned int *mgmt_area)
{
  int i;
  struct ti81xx_mgmt_area  mgmt_area_info;
  debug_print("reading management fields from mgmt_area\n");
  memcpy(&mgmt_area_info, mgmt_area, GENERAL_INFO_SIZE);

  debug_print("unique id field in mgmt area is:  %u\n",
            mgmt_area_info.unique_id);
  debug_print("my unique id field in mgmt area is:  %u\n",
            mgmt_area_info.my_unique_id);
  debug_print("no of block on mgmt area is:  %u\n",
            mgmt_area_info.no_blk);
  debug_print("offset field on mgmt area is:  %u\n",
            mgmt_area_info.offset);
  debug_print("size of management mgmt_area is:  %u\n",
            mgmt_area_info.size);
  debug_print("int capability of remote end point is: %u\n",
            mgmt_area_info.int_cap);

  mgmt_area_info.free_Q =
    (unsigned int *) malloc(FREE_Q_SIZE(mgmt_area_info.no_blk));
  if (mgmt_area_info.free_Q == NULL) {
    err_print("free-Q-- malloc failed\n");
    return -1;
  }

  memcpy(mgmt_area_info.free_Q, mgmt_area + 6,
          FREE_Q_SIZE(mgmt_area_info.no_blk));

  for (i = 0; i < mgmt_area_info.no_blk; i++) {
    if (mgmt_area_info.free_Q[i] != INPROCESS) {
      debug_print("buffer %d is free\n", i);
      break;
    }
  }
  free(mgmt_area_info.free_Q);

  if (i == mgmt_area_info.no_blk) {
    debug_print("all buffers are in process try again\n");
    return -1;
  }

  mgmt_area[6 + i] = INPROCESS; /*update free-Q*/
  return i; /*this buffer is occupied*/
}


/**
 * offset_to_buffer()-- calculate offset of available buffer from start
 * of management area
 * @mgmt_area: value returned by mmap on that particular management area.
 * @i : buffer no whose offset to be calculated
 *
 * return offset of buffer from start of management area.
 */
/* processing overhead in offset by malloc*/
unsigned int offset_to_buffer(unsigned int *mgmt_area, int i)
{

  void *mgmt_blk;
  struct ti81xx_mgmt_area mgmt_area_info;
  unsigned int off_st;
  int j;

  memcpy(&mgmt_area_info, mgmt_area, GENERAL_INFO_SIZE);
  debug_print("mgmt blocks are %u\n", mgmt_area_info.no_blk);

  mgmt_blk = (mgmt_area + 6 + 2 * mgmt_area_info.no_blk);

  mgmt_area_info.mgmt_blk = (struct ti81xx_mgmt_blk *)
        malloc(MGMT_BLKS_SIZE(mgmt_area_info.no_blk));
  if (mgmt_area_info.mgmt_blk == NULL) {
    err_print("mgmt_blk malloc failed\n");
    return 0;
  }

  memcpy(mgmt_area_info.mgmt_blk, mgmt_blk,
        MGMT_BLKS_SIZE(mgmt_area_info.no_blk));

  off_st = mgmt_area_info.size + PAGE_SIZE_EP -
        (mgmt_area_info.size) % PAGE_SIZE_EP ;

  for (j = 0 ; j < i; j++)
    off_st += mgmt_area_info.mgmt_blk[j].size;

  return off_st;
}


/**
 * send_to_remote_buf_by_cpu()-- send data to a particular buffer by
 * application using CPU.
 * @mgmt_area: value returned by mmap on that particular management area.
 * @off_st: offset of buffer from start of management area as returned by
 * offset_to_buffer().
 * @i: buffer no to which data has to be send
 *
 * return 0.
 */

int send_to_remote_buf_by_cpu(unsigned int *mgmt_area,
            unsigned int off_st, int i)
{
  unsigned int no_blk = mgmt_area[2];
  char *buf = (char *)mgmt_area;
  /*
   * sending known pattern "BBBBBBB" of 510 bytes.
   */
  memset(buf + off_st, 67, 510);
  memcpy(buf + off_st + 30, pattern, strlen(pattern));

  /*
   * update write index of buffer
   */
  mgmt_area[6 + 2 * no_blk  +  i * 5 + 2] = 510;
  /* i*5 indicates 5 no of field in mgmt_blk */

  /*
   * update status in used Q as RD
   */

  mgmt_area[6 + no_blk + i] = RD;
  return 0;
}

/**
 * put_data_in_local_buffer()-- put data in one of the local buffer
 * to be read by remote peer.
 * @mgmt_area: value returned by mmap on that particular
 * management area.(mapped_buffer)
 * @off_st: offset of particular buffer from start of management area
 * @i: buffer no to which data have to be sent
 */


int put_data_in_local_buffer(unsigned int *mgmt_area,
            unsigned int off_st, int i)
{
  unsigned int no_blk = mgmt_area[2];
  char *buf = (char *)mgmt_area;
  mgmt_area[6 + i] = INPROCESS;
  /*
  * set known pattern in buffer
  */
  memset((buf + off_st), 76, 510);
  memcpy(buf + off_st + 30, pattern, strlen(pattern));
  /*
  * update write index of buffer
  */
  mgmt_area[6 + 2 * no_blk + i * 5 + 2] = 510;
  /*
  * update status in used Q as WR ( written for remote peer )
  */
  mgmt_area[6 + no_blk + i] = WR;
  return 0;
}

/*
 * read_from_remote_buf_by_cpu()-- read data from buffer on remote peer
 * @mgmt_area:  value returned by mmap on that particular
 * management area.(mapped_pci)
 * @off_st: offset from start of management area of that particular buffer.
 */


int read_from_remote_buf_by_cpu(char *mgmt_area, unsigned int off_st)
{

  #ifdef DISPLAY
  int i = 0;
  for (i = 0; i < 510; i++) {
    if ((i + 1) % 30 == 0)
      printf("\n");
    else
      printf("%c", mgmt_area[off_st + i]);
  }
  printf("\n");
  #endif

  return 0;
}

/**
 * read_from_remote_buf_by_dma()-- read data from buffer on remote peer using
 * dma on EP, this function can only be used on endpoint.
 * @outb_address: address in internal pci address space that to be used
 * as source address.
 * @off_st: offset of remote buffer from start of management area.
 * @fd_dma: file descriptor related to edma handling.
 *
 */


int read_from_remote_buf_by_dma(unsigned int outb_address,
            unsigned int off_st, int fd_dma)
{
  struct dma_info info;
  int ret;
  #ifdef DISPLAY
  int i;
  #endif
  info.size = 510;
  info.user_buf = malloc(510);
  if (info.user_buf == NULL) {
    debug_print("user buffer malloc failed for DMA\n");
    return -1;
  }

  memset(info.user_buf, 0, 510);
  info.dest = 0;
  info.src = (unsigned int *)(outb_address + off_st);
  ret = ioctl(fd_dma, TI81XX_EDMA_READ, &info);
  if (ret < 0) {
    err_print("edma ioctl failed\n");
    free(info.user_buf);
    return -1;
  }

  #ifdef DISPLAY
  for (i = 0; i < 510; i++) {
    if ((i + 1) % 30 == 0)
      printf("\n");
    else
      printf("%c", info.user_buf[i]);
  }
  printf("\n");
  #endif
  free(info.user_buf);
  return 0;
}



/**
 * process_remote_buffer_for_data()-- this function process buffer on
 * remote peer from any data meant to it and call read_from_remote_buf_by_dma()
 * or read_from_remote_buf_by_cpu() to display data
 * and update rd_idx of corresponding buffers.
 *
 * @mgmt_area: value returned by mmap on that particular
 * management area.(mapped_pci)
 * @choice : using EDMA or CPU ( in case running on RC choice
 * should always be CPU)
 * @fd_dma: file descriptor related to edma handling
 *
 * return -1 on failure.
 */
 /* process overhead malloc*/
int  process_remote_buffer_for_data(unsigned int *mgmt_area,
            int choice, int fd_dma)
{
  struct ti81xx_mgmt_area mgmt_area_info;

  unsigned int rd_idx, wr_idx, offset_buffer;
  int i;

  memcpy(&mgmt_area_info, mgmt_area, GENERAL_INFO_SIZE);

  mgmt_area_info.Used_Q =
    (unsigned int *)malloc(USED_Q_SIZE(mgmt_area_info.no_blk));

  if (mgmt_area_info.Used_Q == NULL) {
    err_print("Used-Q-- malloc failed\n");
    return -1;
  }

  mgmt_area_info.mgmt_blk = (struct ti81xx_mgmt_blk *)
        malloc(MGMT_BLKS_SIZE(mgmt_area_info.no_blk));
  if (mgmt_area_info.mgmt_blk == NULL) {
    err_print("MGMT-BLK-- malloc failed\n");
    free(mgmt_area_info.Used_Q);
    return -1;
  }


  memcpy(mgmt_area_info.Used_Q,
    mgmt_area + 6 + mgmt_area_info.no_blk,
        USED_Q_SIZE(mgmt_area_info.no_blk));
  memcpy(mgmt_area_info.mgmt_blk,
      mgmt_area + 6 + 2 * mgmt_area_info.no_blk,
        MGMT_BLKS_SIZE(mgmt_area_info.no_blk));

  debug_print("goimg through rd_idx and wr_idx of buffers\n");

  for (i = 0; i < mgmt_area_info.no_blk ; i++) {
    rd_idx = mgmt_area_info.mgmt_blk[i].rd_idx;
    wr_idx = mgmt_area_info.mgmt_blk[i].wr_idx;

    if ((mgmt_area_info.Used_Q[i] == WR) && (wr_idx != rd_idx)) {
      debug_print("buffer %d  on remote peer have "
          "%u bytes\n", i, wr_idx-rd_idx);
      offset_buffer = offset_to_buffer(mgmt_area, i);
      if (choice == EDMA) {
        debug_print("choice is EDMA offset is 0x%x\n",
                offset_buffer);
        read_from_remote_buf_by_dma(0x20000000,
              offset_buffer, fd_dma);
            /* mapped_pci<-> 0x20000000*/
        debug_print("update rd_idx of remote buf\n");
        mgmt_area[6 +
          2 * mgmt_area_info.no_blk +
              i * 5 + 1] = 510;
            /* 510 bytes has been read */
      } else if (choice == CPU) {
        read_from_remote_buf_by_cpu((char *)mgmt_area,
                offset_buffer);
        debug_print("update wr_idx of remote buf\n");
        mgmt_area[6 +
          2 * mgmt_area_info.no_blk +
              i * 5 + 1] = 510;
            /* 510 bytes has been read */
      }
    }

  }

  free(mgmt_area_info.mgmt_blk);
  free(mgmt_area_info.Used_Q);
  return 0;
}




/**
 * send_to_remote_buf_by_dma()-- send data to buffer on remote peer
 * using dma on EP, this function can only be used on End point
 * @mgmt_area:  value returned by mmap on that particular
 * management area.(mapped_pci)
 * @outb_address: address on internal pci address range to which
 * this transaction will be trageted.
 * @off_st: offset of buffer from start of management area.
 * @i: buffer no
 * @fd_dma: file descriptor corresponding to edma handling.
 *
 * __NOTE__: this function will only be used on end point.
 *
 */



int send_to_remote_buf_by_dma(unsigned int *mgmt_area,
  unsigned int outb_address, unsigned int off_st, int i, int fd_dma)
{
  unsigned int no_blk = mgmt_area[2];
  struct dma_info info;
  int ret;
  info.size = 510;
  info.user_buf = (unsigned char *)malloc(510);
  if (info.user_buf == NULL) {
    err_print("user buffer malloc failed for DMA\n");
    return -1;
  }

  /*
  *
  * sending known pattern to assure integrity of data transfer.
  */

  memset(info.user_buf, 68, 510);
  memcpy(info.user_buf + 30, pattern, strlen(pattern));

  info.dest = (unsigned int *)(outb_address + off_st);
  info.src = 0;
  ret = ioctl(fd_dma, TI81XX_EDMA_WRITE, &info);

  if (ret < 0) {
    err_print("edma ioctl failed\n");
    free(info.user_buf);
    return -1;
  }

  /*
   * update wr_index of corresponding buffer
   */

  mgmt_area[6 + 2 * no_blk + i * 5 + 2] = 510;

  /*
   *  update status in used Q as RD
   */

  mgmt_area[6 + no_blk + i] = RD;
  free(info.user_buf);
  return 0;
}

/**
 * find_dedicated_buffer() -- this function search a dedicated buffer
 * for given EP identified using my unique id.
 * @mgmt_area: pointer to management area
 * @muid: unique id of peer for which buffer is being searched
 * @dbuf: pointer to dedicated buf structure that will contain info
 * of dedicated buffer
 * @choice: RD/WR buffer is for reading or writing
 * return 0 on success
 */


int find_dedicated_buffer(unsigned int *mgmt_area, unsigned int muid,
      struct dedicated_buf *dbuf, unsigned int choice)
{
  void *pntr;
  unsigned int offset = 0;
  int i = 0;
  int j;
  struct ti81xx_mgmt_area mgmt_area_info;
  memcpy(&mgmt_area_info,  mgmt_area,  GENERAL_INFO_SIZE);
  debug_print("mgmt blocks are %u\n", mgmt_area_info.no_blk);
  pntr = (mgmt_area + 6 + 2 * mgmt_area_info.no_blk);

  mgmt_area_info.mgmt_blk = (struct ti81xx_mgmt_blk *)
        malloc(MGMT_BLKS_SIZE(mgmt_area_info.no_blk));
  if (mgmt_area_info.mgmt_blk == NULL) {
    err_print("mgmt_blk malloc failed\n");
    return -1;
  }



  memcpy(mgmt_area_info.mgmt_blk, pntr,
        MGMT_BLKS_SIZE(mgmt_area_info.no_blk));
  pntr = (mgmt_area + 6 + mgmt_area_info.no_blk);

  mgmt_area_info.Used_Q = (unsigned int *)
        malloc(USED_Q_SIZE(mgmt_area_info.no_blk));

  if (mgmt_area_info.Used_Q == NULL) {
    err_print("used Q malloc failed\n");
    free(mgmt_area_info.mgmt_blk);
    return -1;
  }

  memcpy(mgmt_area_info.Used_Q, pntr, USED_Q_SIZE(mgmt_area_info.no_blk));
  debug_print("searching for buffer having used_Q status %u [5-RD/6-WR] "
      "and dedicated for peer having id %u\n", choice, muid);
  for (j = 0; j < mgmt_area_info.no_blk; j++) {

    debug_print("status of %d buffer %u\n",
        j, mgmt_area_info.Used_Q[j]);

    if (mgmt_area_info.mgmt_blk[j].status == muid) {
      if (mgmt_area_info.Used_Q[j] == choice) {
        debug_print("buffer %d is dedicated for peer "
            " having ID %u and used_Q "
            "status %u[5-RD/6-WR]\n",
              j, muid, choice);
        dbuf->wr_ptr = mgmt_area +
            6 + 2 * mgmt_area_info.no_blk +
                j * 5  + 2;
        dbuf->rd_ptr = mgmt_area +
            6 + 2 * mgmt_area_info.no_blk +
                j * 5  + 1;
        break;
      }
    }
  }

  if (j == mgmt_area_info.no_blk) {
    debug_print("no buffer with used_Q status %u [5-RD/6-WR]is "
        "dedicated for peer having id %u",
              choice, muid);
    free(mgmt_area_info.mgmt_blk);
    free(mgmt_area_info.Used_Q);
    return -1;
  }

  offset = mgmt_area_info.size + PAGE_SIZE_EP -
          (mgmt_area_info.size) % PAGE_SIZE_EP ;

  for (i = 0 ; i < j; i++)
    offset += mgmt_area_info.mgmt_blk[i].size;

  dbuf->off_st = offset;
  free(mgmt_area_info.mgmt_blk);
  free(mgmt_area_info.Used_Q);
  return 0;
}
