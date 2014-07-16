
/*
 * This file contain function definitions related to management area( book
 * keeping information about buffers on dedicated memory). All the functions
 * are dealing with access/update management area information.
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *     Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the
 *     distribution.
 *
 *     Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ti81xx_mgmt_lib.h"
#include "debug_msg.h"

#define APP_NAME     "ti81xx_mgmt_lib"

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

static int debug = 1;


#define min(x, y)  (((x) < (y)) ? (x) : (y))

int print_mgmt_area(char *func, int line, unsigned int *mgmt_area)
{
  int no_blk = 6;
  int j = 0;
  printf("[%s:%d] == mgmt_area:%p \n", func, line, mgmt_area);
  printf("[%s:%d] == uid   :%d muid   :%d \n", func, line, mgmt_area[0], mgmt_area[1]);
  printf("[%s:%d] == no_blk:%d offs   :%d \n", func, line, mgmt_area[2], mgmt_area[3]);
  printf("[%s:%d] == size  :%d int_cap:%d \n", func, line, mgmt_area[4], mgmt_area[5]);

  printf("[%s:%d] == free_Q:", func, line);
  for (j = 0; j < no_blk; j++)
  {
    printf("%d ", mgmt_area[6 + j]);
  }
  printf("\n");
  printf("[%s:%d] == used_Q:", func, line);
  for (j = 0; j < no_blk; j++)
  {
    printf("%d ", mgmt_area[6 + no_blk + j]);
  }
  printf("\n");


  for (j = 0; j < no_blk; j++)
  {
    // debug_print("mgmt_area blk[j] %p mgmt_blk_ptr:%p (%d)\n", mgmt_area, pntr, pntr - mgmt_area);
    // printf("== mgmt_area:%p blk[%d] mgmt_area[status]:%p (%d) mgmt_area[choice]:%p (%d)\n",
    //          mgmt_area,
    //          j,
    //          &mgmt_area[6 + 2 * no_blk + j * 5 + 3], &mgmt_area[6 + 2 * no_blk + j * 5 + 3] - mgmt_area,
    //          &mgmt_area[6 + no_blk + j], &mgmt_area[6 + no_blk + j] - mgmt_area);
    printf("[%s:%d] == blk[%d] status:%d  choice:%d \n",
              func, line,
             j,
             j,
             mgmt_area[6 + 2 * no_blk + j * 5 + 3],
             mgmt_area[6 + no_blk + j]);
  }
}



/**
 * ti81xx_set_mgmt_area() -- This function initialize management area
 * @mgmt_area: points to management area related info (struct ti81xx_mgmt_area)
 * filled by ti81xx_prepare_mgmt_info()
 * @mapped_buffer: address returned by mmap (mmap from startof dedicated memory)
 * on dedicated memory
 *
 * This function sets up mamagement area related detail contained in struct
 * ti81xx_mgmt_area
 * (that is filled by ti81xx_prepare_mgmt_info()) on management area
 * (on dedicated physical memory)
 */


int ti81xx_set_mgmt_area(struct ti81xx_mgmt_area *mgmt_area,
            unsigned int *mapped_buffer)
{

  unsigned int *ptr = (unsigned int *)mgmt_area;
  memcpy(mapped_buffer + 2, ptr + 2,
      GENERAL_INFO_SIZE - 2 * sizeof(unsigned int));
  mapped_buffer += 6;
  /* 6 are no of elements in general info size (6 u32)*/
  // memcpy(mapped_buffer, mgmt_area->free_Q,
  //        FREE_Q_SIZE(mgmt_area->no_blk));
  // mapped_buffer += mgmt_area->no_blk;
  // memcpy(mapped_buffer, mgmt_area->Used_Q,
  //        USED_Q_SIZE(mgmt_area->no_blk));
  // mapped_buffer += mgmt_area->no_blk;
  // memcpy(mapped_buffer, mgmt_area->mgmt_blk,
  //        MGMT_BLKS_SIZE(mgmt_area->no_blk));
  return 0;
}


/**
 * ti81xx_get_mgmt_area() -- this function provides snapshot of management area
 * @mgmt_area: points to  struct ti81xx_mgmt_area that will contain details
 * @mapped_buffer: address returned by mmap (mmap from startof dedicated memory)
 * on dedicated memory
 *
 * __NOTE__: it is assumed that before calling this function, the structure that
 * is pointed by mgmt_area has been initialized properly.( EX: memory allocated
 * for free Q etc.)
 *
 */


int ti81xx_get_mgmt_area(struct ti81xx_mgmt_area *mgmt_area,
            unsigned int *mapped_buffer)
{
  memcpy(mgmt_area, mapped_buffer, GENERAL_INFO_SIZE);
  mapped_buffer += 6;
  memcpy(mgmt_area->free_Q, mapped_buffer,
          FREE_Q_SIZE(mgmt_area->no_blk));
  mapped_buffer += mgmt_area->no_blk;
  memcpy(mgmt_area->Used_Q, mapped_buffer,
          USED_Q_SIZE(mgmt_area->no_blk));
  mapped_buffer += mgmt_area->no_blk;
  memcpy(mgmt_area->mgmt_blk, mapped_buffer,
          MGMT_BLKS_SIZE(mgmt_area->no_blk));
  return 0;
}


/**
 * ti81xx_prepare_mgmt_info()-- this function prepare struct ti81xx_mgmt_area
 * by asking different details from user
 * @mgmt_area: pointer to struct ti81xx_mgmt_area that will be initialize
 * @start_addr: contain start addresses of dedicated memory
 *
 * This function fillup struct ti81xx_mgmt_area with different information that
 * have to be written on management area in order to initialize it
 */

int ti81xx_prepare_mgmt_info(struct ti81xx_mgmt_area *mgmt_area, unsigned int size_buffer)
{
  unsigned int i;
  /*unsigned int buffer_start_addr;*/
  /*printf("enter unique id\n");
  scanf("%u",&(mgmt_area->unique_id));
  printf("enter my_unique_id\n");
  scanf("%u",&(mgmt_area->my_unique_id));*/

  mgmt_area->no_blk = MAX_BLOCKS;
  mgmt_area->size = MGMT_BLKS_SIZE(mgmt_area->no_blk);
  mgmt_area->size += GENERAL_INFO_SIZE;
  mgmt_area->size += 2 * FREE_Q_SIZE(mgmt_area->no_blk);
  mgmt_area->offset = GENERAL_INFO_SIZE +
        2 * FREE_Q_SIZE(mgmt_area->no_blk);

  debug_print("no_blk:%d size:%08x offset:%08x size_buffer:%08x\n",
       mgmt_area->no_blk, mgmt_area->size, mgmt_area->offset, size_buffer);

  mgmt_area->int_cap = 0; /*by default nothing.( INT, POLL are possible)*/

  debug_print("initializing free/used queues\n");
  mgmt_area->free_Q = malloc(FREE_Q_SIZE(mgmt_area->no_blk));
  if (mgmt_area->free_Q == NULL) {
    err_print("FREE-Q-- malloc failed\n");
    return -1;
  }

  for (i = 0; i < (mgmt_area->no_blk); i++)
    (mgmt_area->free_Q)[i] = 0;

  mgmt_area->Used_Q = malloc(USED_Q_SIZE(mgmt_area->no_blk));
  if (mgmt_area->Used_Q == NULL) {
    err_print("Used-Q-- malloc failed\n");
    free(mgmt_area->free_Q);
    return -1;
  }

  for (i = 0; i < (mgmt_area->no_blk); i++)
    (mgmt_area->Used_Q)[i] = 0;

  debug_print("initializing management blocks\n");
  mgmt_area->mgmt_blk = malloc(MGMT_BLKS_SIZE(mgmt_area->no_blk));
  if (mgmt_area->mgmt_blk == NULL) {
    err_print("Mgmt-blk-- malloc failed\n");
    free(mgmt_area->Used_Q);
    free(mgmt_area->free_Q);
    return -1;
  }

  /*i=(start_addr.start_addr_virt + mgmt_area->size ) % PAGE_SIZE_EP;
  buffer_start_addr = start_addr.start_addr_virt + mgmt_area->size +
        PAGE_SIZE_EP - i;*/ /*to make 4 KB alignment*/

  for (i = 0; i < (mgmt_area->no_blk); i++) {
    ((mgmt_area->mgmt_blk)[i]).buf_ptr = 0;
    ((mgmt_area->mgmt_blk)[i]).rd_idx = 0;
    ((mgmt_area->mgmt_blk)[i]).wr_idx = 0;
    ((mgmt_area->mgmt_blk)[i]).status = 0;
    ((mgmt_area->mgmt_blk)[i]).size = size_buffer;
  }
  return 0;
}


/**
 * ti81xx_calculate_ptr()-- calculate pointers to different information in
 * management area and put them in struct ti81xx_ptrs
 * @mgmt_area: points to struct ti81xx_mgmt_area that have details of
 * management area.
 * @mapped_buffer: address returned by mmap (mmap from start of dedicated
 * memory) on dedicated memory
 * @ptr: points to struct ti81xx_ptrs that have to be filled.
 */

int ti81xx_calculate_ptr(struct ti81xx_mgmt_area *mgmt_area,
        char *mapped_buffer, struct ti81xx_ptrs *ptr)
{
  ptr->offset_free = (unsigned int *)(mapped_buffer +
          mgmt_area->offset -
          2 * FREE_Q_SIZE(mgmt_area->no_blk));
  ptr->offset_used = (unsigned int *)(mapped_buffer + mgmt_area->offset -
            FREE_Q_SIZE(mgmt_area->no_blk));
  ptr->offset_status = (unsigned int *)(mgmt_area->offset +
            3 * sizeof(unsigned int) +
              mapped_buffer);
  ptr->offset_wr_idx = (unsigned int *)(mgmt_area->offset +
            2 * sizeof(unsigned int) +
              mapped_buffer);
  ptr->offset_rd_idx = (unsigned int *)(mgmt_area->offset +
            sizeof(unsigned int) +
              mapped_buffer);
  ptr->offset_size = (unsigned int *)(mgmt_area->offset +
            4 * sizeof(unsigned int) +
              mapped_buffer);
  return 0;
}


/**
 * ti81xx_access_free_Q()-- for accessing free-Q list
 * @value: pointer to an integer used for SET/GET
 * @blk_no: block no of buffer, in free-Q list.
 * @mode: SET/GET
 * @ptr: points to struct ti81xx_ptrs that contain pointers to management area.
 *
 * this function can SET/GET status of a particular buffer in free-Q.
 *
 * return value-- 0 on success.
 */

int ti81xx_access_free_Q(unsigned int *value, unsigned int blk_no,
        unsigned int mode, struct ti81xx_ptrs *ptr)
{
  if (mode == SET)
    (ptr->offset_free)[blk_no] = *value;
  else if (mode == GET)
    *value = (ptr->offset_free)[blk_no];
  return 0;
}

/**
 * ti81xx_access_used_Q()-- for accessing used-Q list
 * @value: pointer to an integer used for SET/GET.
 * @blk_no: block no of buffer, in used-Q list.
 * @mode: SET/GET
 * @ptr: points to struct ti81xx_ptrs that contain pointers to management area.
 *
 * this function can SET/GET status of a particular buffer in used-Q.
 *
 * return value-- 0 on success.
 */

int ti81xx_access_used_Q(unsigned int *value, unsigned int blk_no,
        unsigned int mode, struct ti81xx_ptrs *ptr)
{
  if (mode == SET)
    (ptr->offset_used)[blk_no] = *value;
  else if (mode == GET)
    *value = (ptr->offset_used)[blk_no];
  return 0;
}


/**
 * ti81xx_access_status()-- for accessing status of a particular buffer
 * in management block.
 * @value: pointer to an integer used for SET/GET.
 * @blk_no: block no of buffer.
 * @mode: SET/GET
 * @ptr: points to struct ti81xx_ptrs that contain pointer to management area
 *
 * this function can SET/GET status of a particular buffer in management block.
 *
 * return value-- 0 on success.
 */

int ti81xx_access_status(unsigned int *value, unsigned int blk_no,
        unsigned int mode, struct ti81xx_ptrs *ptr)
{
  if (mode == SET)
    (ptr->offset_status)[blk_no * 5] = *value;
  else if (mode == GET)
    *value = (ptr->offset_status)[blk_no * 5];

  return 0;
}

/**
 * ti81xx_access_wr_idx()-- for accessing wr_idx of a particular buffer
 * in management block.
 * @value: pointer to an integer used for SET/GET.
 * @blk_no: block no of buffer.
 * @mode: SET/GET
 * @ptr: points to struct ti81xx_ptrs that contain pointers to management area.
 *
 * this function can SET/GET wr_idx  of a particular buffer in management block.
 *
 * return value-- 0 on success.
 */

int ti81xx_access_wr_idx(unsigned int *value, unsigned int blk_no,
        unsigned int mode, struct ti81xx_ptrs *ptr)
{
  if (mode == SET)
    (ptr->offset_wr_idx)[blk_no * 5] = *value;
  else if (mode == GET)
    *value = (ptr->offset_wr_idx)[blk_no * 5];

  return 0;
}


/**
 * ti81xx_access_rd_idx()-- for accessing rd_idx of a particular buffer
 * in management block.
 * @value: pointer to an integer used for SET/GET.
 * @blk_no: block no of buffer.
 * @mode: SET/GET
 * @ptr: points to struct ti81xx_ptrs that contain pointers to management area.
 *
 * this function can SET/GET rd_idx of a particular buffer in management block.
 *
 * return value-- 0 on success.
 */

int ti81xx_access_rd_idx(unsigned int *value, unsigned int blk_no,
        unsigned int mode, struct ti81xx_ptrs *ptr)
{

  if (mode == SET)
    (ptr->offset_rd_idx)[blk_no * 5] = *value;
  else if (mode == GET)
    *value = (ptr->offset_rd_idx)[blk_no * 5];
  return 0;
}

/**
 * dedicate_buffer()-- for dedicating a buffer out of local buffer for
 * read/write by remote peer.
 * @mgmt_area: pointer to local management area returned by mmap.
 * @muid: unique id of remote peer for which this buffer is being dedicated.
 * @no_blk: no of buffers on locla peer
 * @buf_no: buffer no which is being dedicated.
 * @choice: for read/write i.e. WR/RD
 */

int dedicate_buffer(unsigned int *mgmt_area, unsigned int muid,
      unsigned int no_blk, unsigned int buf_no,
            unsigned int choice)
{
  if (buf_no >= no_blk) {
    err_print("buffer %u not resides on management area\n", buf_no);
    return -1;
  }
  /* update status field with unique id of remote peer */
  mgmt_area[6 + 2 * no_blk + buf_no * 5 + 3] = muid;
  /* set used Q filed of particular buffer to be RD/WR */
  mgmt_area[6 + no_blk + buf_no] = choice;
  debug_print("buffer %u is dedicated  with %u[5-RD/6-WR] USED_Q status "
        "to rmt peer having unique id %u\n", buf_no,
          mgmt_area[6 + no_blk + buf_no], muid);
  return 0;
}


#ifdef INTEGRITY
/**
 * dump_data_in_file(): this function dump data in a file from a buffer
 * @buf: pointer to buffer
 * @buf_len: length of data in buffer
 * @fp: file pointer
 */

int dump_data_in_file(char *buf, unsigned int buf_len, FILE *fp)
{
  int ret = 0;
  ret = fwrite(buf, buf_len, 1, fp);
  if (ret == 0) {
    err_print("failed to dump data in file\n");
    return -1;
  }
  debug_print("%d bytes are dumped into file\n", buf_len);
  return 0;
}

/**
 * set_data_to_buffer(): read dat from a file and piy it in a buffer
 * @buf: pointer to buffer
 * @buf_len: lenght of data to be put in buffer
 * @fp: file pointer
 */

int set_data_to_buffer(char *buf, unsigned int buf_len, FILE *fp)
{
  int ret = 0;
  ret = fread(buf, buf_len, 1, fp);
  if (ret < 0) {
    debug_print("error in fread data not set in buffer\n");
    return -1;
  }
  debug_print("%d bytes are dumped into buffer from file\n", buf_len);
  return 0;
}

/**
 * create_random_pattern() : craete a file with a particular pattern star
 * from 'A' and repeat after repeat index.
 * @len: length of file
 * @repeat: repetion index
 * @fp: file pointer
 */

int create_random_pattern(unsigned int len, unsigned int repeat, FILE *fp)
{
  int i;
  int j = 1;
  char pattern = 65;
  for (i = 0; i < len; i++, j++) {
    fputc(pattern, fp);
    if (j < repeat)
      pattern++;
    else {
      pattern = 65;
      j = 0;
    }
  }
  fseek(fp, 0, SEEK_SET);
  return 0;
}

/**
 * check_integrity(): compare two files byte by byte and check if they are same
 * @fp1: file pointer to first fiel
 * @fp2: file pointer to second file
 */

int check_integrity(FILE *fp1, FILE *fp2)
{
  int file_length1, file_length2;
  char ch2, ch1;
  fseek(fp1, 0, SEEK_SET);
  fseek(fp2, 0, SEEK_SET);
  fseek(fp1, 0, SEEK_END);
  file_length1 = ftell(fp1);
  fseek(fp1, 0, SEEK_SET);
  fseek(fp2, 0, SEEK_END);
  file_length2 = ftell(fp2);
  fseek(fp2, 0, SEEK_SET);

  if (file_length1 != file_length2) {
    err_print("data corrupted ,file original %u bytes and file "
            "recevied  %u bytes\n",
            file_length1, file_length2);
    return -1;
  }
  debug_print("length of received file is %d\n", file_length1);
  while (--file_length1 >= 0) {
    ch2 = fgetc(fp2);
    ch1 = fgetc(fp1);
    if (ch1 == ch2)
      continue;
    else {
      err_print("data pattern corrupted in recived file at "
          "count %u is %c",
            (unsigned int)ftell(fp2), ch2);
      return -1;
    }
  }
  debug_print("file recevied successfully\n");
  return 0;
}

#endif

#ifdef THPT
/* similar to dump_data_in_file except only called when last chunk is
 * recevied */

int dump_data_in_file_n(char *buf, unsigned int buf_len, char *name)
{
  int ret = 0;
  FILE *fpsec;
  fpsec = fopen(name, "w+");
  if (fpsec == NULL) {
    err_print("failed to dump last chunk in file\n");
    return -1;
  }
  ret = fwrite(buf, buf_len, 1, fpsec);
  if (ret == 0) {
    err_print("fwrite error occur\n");
    return -1;
  }
  debug_print("last chunk of %d bytes are dumped into file\n", buf_len);
  return 0;
}
#endif



/**
 * ti81xx_poll_for_data()-- polling for data in buffers
 *
 * this function goes through rd_idx and wr_idx of each buffer
 * and display the data, if present in any of them.
 * after reading data it again puts back the buffer from Used-Q to Free-Q.
 * also recycle buffers in which data was there for remote buffer and they have
 * read it.
 *
 * @ptr: pointer to structure ti81xx_ptrs that hold different pointers to
 * management  area.
 * @mgmt_area: pointer to struct ti81xx_mgmt_area that have details of
 * management area.
 * @mapped_buffer: address returned by mmap(from start)  on dedicated
 * memory.
 *
 */

int ti81xx_poll_for_data(struct ti81xx_ptrs *ptr,
        struct ti81xx_mgmt_area *mgmt_area,
          char *mapped_buffer, FILE *fp,
            unsigned long long *byte_recv)
{

  unsigned int offset = mgmt_area->offset + sizeof(unsigned int);
  unsigned int *rd_idx;
  unsigned int *wr_idx;
  unsigned int value = 0;
  unsigned int *used_Q = (unsigned int *)mapped_buffer;
  unsigned int i = value;
  #ifdef DISPLAY
  unsigned int j;
  #endif
  unsigned int align = (mgmt_area->size) % PAGE_SIZE_EP;
  unsigned int offset_buffer = mgmt_area->size + PAGE_SIZE_EP - align;
  used_Q += ((mgmt_area->no_blk) + 6);

  vdebug_print("mgmt_area:%p mgmt_area->no_blk:%d\n", mgmt_area, mgmt_area->no_blk);

  /* overhead part above of it must be executed only once at first time
  * after that always start from similar place
  * */
  for (i = 0; i < mgmt_area->no_blk; i++) {
    rd_idx = (unsigned int *)(mapped_buffer + offset);
    wr_idx = rd_idx + 1;

    vdebug_print("mapped_buffer:%08x offset:%08x rd_idx:%p wr_idx:%p\n", mapped_buffer, offset, rd_idx, wr_idx);

    #ifndef THPT
    vdebug_print("wr_idx  %u  wr_idx_ptr 0x%x rd_IDX %u rd_idx_ptr "
        " 0x%x used_Q %u[5-RD/6-WR]\n",
            *wr_idx, (unsigned int)wr_idx,
            *rd_idx, (unsigned int)rd_idx,
                used_Q[i]);
    #endif
    if ((*wr_idx - *rd_idx) && (used_Q[i] == RD)) {

      #ifndef THPT
      vdebug_print("there is data in buffer %u rd_idx=%u  "
          "wr_idx=%u\n", i, *rd_idx, *wr_idx);
      #endif

      #ifdef DISPLAY

      debug_print("block:%d Data: \n", i);
      for (j = 0; j < (*wr_idx - *rd_idx); j++) {
        if ((j + 1) % 30 == 0)
          printf("\n");
        else
          printf("%c", *(char *)(mapped_buffer +
              offset_buffer + j));
      }
      printf("\n");

      #endif

      #if defined(INTEGRITY) || defined(THPT)
      *byte_recv += (*wr_idx - *rd_idx);
      #endif

      #ifdef INTEGRITY
      dump_data_in_file(mapped_buffer + offset_buffer,
            (*wr_idx - *rd_idx), fp);
      memset(mapped_buffer + offset_buffer, 0,
            (*wr_idx - *rd_idx));
      /*debug_print("bytes recvied are %u\n",
            (*wr_idx - *rd_idx));*/
      #endif

      #ifdef THPT
      if (*byte_recv == 0x140000000llu)
        dump_data_in_file_n(mapped_buffer +
              offset_buffer,
              (*wr_idx - *rd_idx),
                "sent.txt");
      #endif

      *wr_idx = 0; /* update wr_index to be zero again */

      #ifndef THPT
      vdebug_print("wr_index has been updated to zero\n");
      #endif

      #ifndef INTEGRITY
      #ifndef THPT
      /*remove from used -Q*/
      ti81xx_access_used_Q(&value, i, SET, ptr);
      vdebug_print("buffer remove from used Q\n");
      /*update in free-Q*/
      ti81xx_access_free_Q(&value, i, SET, ptr);
      vdebug_print("buffer update  in  free  Q\n");
      #endif
      #endif

      /*dedicated stuff always to be done*/
      /* update wr_idx*/
      /*ti81xx_access_wr_idx (&value, i, SET,ptr);*/
      /*update rd_idx*/
      /*ti81xx_access_rd_idx (&value, i, SET,ptr);*/
    }
    #ifndef THPT
    #ifndef INTEGRITY
    if ((*wr_idx == *rd_idx) && (used_Q[i] == WR)) {
      vdebug_print("data has been read by peer , "
            "recycle this %d buffer\n", i);
      /*remove from used -Q*/
      ti81xx_access_used_Q(&value, i, SET, ptr);
      /*update in free Q*/
      ti81xx_access_free_Q(&value, i, SET, ptr);
      /*update wr_idx*/
      ti81xx_access_wr_idx(&value, i, SET, ptr);
      /*update rd_idx*/
      ti81xx_access_rd_idx(&value, i, SET, ptr);
    }
    #endif
    #endif

    /*
    #ifdef THPT
    if ((*wr_idx != 0) && (used_Q[i] == WR))
      *wr_idx = 0;
    #endif
    */
    offset += sizeof(struct ti81xx_mgmt_blk);
    offset_buffer += *(wr_idx + 2);
  }
  return 0;
}
