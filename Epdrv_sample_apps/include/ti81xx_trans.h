/*
 * This file contain function declaration used for data transfers defined
 * in ti81xx_trans.c
 *
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

#ifndef __TRANS_FUNC__
#define __TRANS_FUNC__

/* direction of EDMA transfers */
#define EP_OUT			0
#define EP_IN			1

/* transfers mode */
#define EDMA			1
#define CPU			2

extern int access_mgmt_area(unsigned int *mgmt_area, unsigned int unique_id);
extern inline void release_mgmt_area(unsigned int *mgmt_area);
extern int get_free_buffer(unsigned int *mgmt_area);
extern unsigned int offset_to_buffer(unsigned int *mgmt_area, int i);
extern int send_to_remote_buf_by_cpu(unsigned int *mgmt_area,
					unsigned int off_st, int i);
extern int put_data_in_local_buffer(unsigned int *mgmt_area,
					unsigned int off_st, int i);
extern int read_from_remote_buf_by_cpu(char *mgmt_area, unsigned int off_st);
extern int  process_remote_buffer_for_data(unsigned int *mgmt_area,
					int choice, int fd_dma);
extern int read_from_remote_buf_by_dma(unsigned int outb_address,
					unsigned int off_st, int fd_dma);
extern int send_to_remote_buf_by_dma(unsigned int *mgmt_area,
						unsigned int outb_address,
						unsigned int off_st,
						int i, int fd_dma);
extern int find_dedicated_buffer(unsigned int *mgmt_area, unsigned int muid,
						struct dedicated_buf *dbuf,
							unsigned int choice);
#endif
