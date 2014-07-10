/*
 * This file contain definition of data structure used for managing the
 * management area inside driver. There is no hard and fast rule to use
 * these data structures. driver is not aware of any of theses. for driver,
 * management area is just contiguous chunk of memory on which management
 * information is written in a particular format. Applications can have their
 * own custom data structures, but at the end, information must be written in
 * same format that is known to driver.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#ifndef __TI816x_PCIE_EP_MGMT__
#define __TI816x_PCIE_EP_MGMT__

#include <drivers/char/ti81xx_pcie_epdrv.h>
#include <drivers/char/ti81xx_edma.h>

#define SET		1
#define GET		0
#define USED		1
#define INPROCESS	1


#define RC_UNIQ_ID		1


#define INT_CAP				1
#define POLL_CAP			2
#define MAX_BLOCKS		6
#define GENERAL_INFO_SIZE	(sizeof(unsigned int) * 6)
#define FREE_Q_SIZE(no_blk)	(sizeof(unsigned int) * no_blk)
#define USED_Q_SIZE(no_blk)	(sizeof(unsigned int) * no_blk)
#define MGMT_BLKS_SIZE(no_blk)	(sizeof(struct ti81xx_mgmt_blk) * no_blk)
#define PAGE_SIZE_EP		4096
#define RD	5
#define WR	6
#include <stdio.h>

/**
 * ti81xx_mgmt_blk - Management information related to buffer on local
 * EP's physical memory
 * @buf_ptr: Pointer to crrespoding local buffer
 * @rd_idx: pointing the read index in buffer
 * @wr_idx: pointing the write index in buffer
 * @status: pointing the status of buffer free/used/invalid/
 * transaction_inprogress
 * @size: total size of the local buffer
 *
 * Description:
 * This structure describe the local buffer
 * There will be one such structure for each buffer.
 * Size of available data shall be calculated with rd_idx & wr_idx
 * When rd_idx == wr_idx, buffer is considered empty
 */

struct ti81xx_mgmt_blk {
	unsigned int buf_ptr;
	unsigned int rd_idx;
	unsigned int wr_idx;
	unsigned int status;
	unsigned int size;
};

/**
 * ti81xx_mgmt_area - Management information area, contains book keeping
 * info of physical memory
 * @unique_id: contains a uinque id of the Remote end point that wants to
 * access Management area
 * @my_unique_id: unique id of locla peer it self
 * @no_blk: no of management_blocks in management area, which also indicates
 * no. of local buffers
 * @offset: offset of first management block from unique id
 * @size: size of management area
 * @int_cap: this filed will be set by remote peer about it's interrupt
 * notifying capability. (1--int/2--poll default 0).
 * @free_Q: pointer to an array of free-Q indexs
 * @used_Q: pointer to an array of used_Q indexs
 * @mgmt_blk: pointer to a struct ti81xx_mgmt_blk
 * @my_unique_id: unique id of local EP on which this structure resides
 *
 * Description:
 * This structure describes the entire management info
 * There will be one such structure on each EP.
 */


struct ti81xx_mgmt_area {
	unsigned int unique_id;
	unsigned int my_unique_id;
	unsigned int no_blk;
	unsigned int offset;
	unsigned int size;
	unsigned int int_cap;
	unsigned int *free_Q;
	unsigned int *Used_Q;
	struct ti81xx_mgmt_blk *mgmt_blk;
};


/**
 * ti81xx_ptrs:  this structure contain poiinters to different mgmt
 * information in mgmt area.
 *
 * @offset_free: pointer to start of free Q
 * @offset_used: pointer to start of used Q
 * @offset_status: pointer to status of very first buffer
 * @offset_wr_idx: pointer to wr_idx of first buffer
 * @offset_rd_idx: pointer to rd_idx of first buffer
 * @offset_size: pointer to size of first buffer
 */

struct ti81xx_ptrs {
	unsigned int *offset_free;
	unsigned int *offset_used;
	unsigned int *offset_status;
	unsigned int *offset_wr_idx;
	unsigned int *offset_rd_idx;
	unsigned int *offset_size;
};

struct dedicated_buf {
	unsigned int *wr_ptr;
	unsigned int *rd_ptr;
	unsigned int off_st;
};


/*
 * function declaration
 */
int print_mgmt_area(char *func, int line, unsigned int *mgmt_area);

int ti81xx_set_mgmt_area(struct ti81xx_mgmt_area *mgmt_area,
					unsigned int *mapped_buffer);
int ti81xx_get_mgmt_area(struct ti81xx_mgmt_area *mgmt_area,
					unsigned int *mapped_buffer);
int ti81xx_prepare_mgmt_info(struct ti81xx_mgmt_area *mgmt_area, unsigned int
		size_buffer);
int ti81xx_calculate_ptr(struct ti81xx_mgmt_area *mgmt_area,
			char *mapped_buffer, struct ti81xx_ptrs *ptr);
int ti81xx_access_free_Q(unsigned int *value, unsigned int blk_no,
				unsigned int mode, struct ti81xx_ptrs *ptr);
int ti81xx_access_used_Q(unsigned int *value, unsigned int blk_no,
				unsigned int mode, struct ti81xx_ptrs *ptr);
int ti81xx_access_status(unsigned int *value, unsigned int blk_no,
				unsigned int mode, struct ti81xx_ptrs *ptr);
int ti81xx_access_wr_idx(unsigned int *value, unsigned int blk_no,
				unsigned int mode, struct ti81xx_ptrs *ptr);
int ti81xx_access_rd_idx(unsigned int *value, unsigned int blk_no,
				unsigned int mode, struct ti81xx_ptrs *ptr);
int ti81xx_poll_for_data(struct ti81xx_ptrs *ptr,
				struct ti81xx_mgmt_area *mgmt_area,
					char *mapped_buffer, FILE *fp,
						unsigned long long *byte_recv);
int dedicate_buffer(unsigned int *mgmt_area, unsigned int muid,
						unsigned int no_blk,
							unsigned int buf_no,
							unsigned int choice);
#ifdef INTEGRITY
int dump_data_in_file(char *buf, unsigned int buf_len, FILE *fp);
int set_data_to_buffer(char *buf, unsigned int buf_len, FILE *fp);
int create_random_pattern(unsigned int len, unsigned int repeat, FILE *fp);
int check_integrity(FILE *fp1, FILE *fp2);
#endif
#ifdef THPT
int dump_data_in_file_n(char *buf, unsigned int buf_len, char *name);
#endif
#endif
