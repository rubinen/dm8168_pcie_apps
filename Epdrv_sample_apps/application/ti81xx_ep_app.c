/*
 * Application to Test EP driver and it's functionality along with EDMA.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <pthread.h>
#include <semaphore.h>
#include "ti81xx_ep_lib.h"
#include "ti81xx_mgmt_lib.h"
#include "ti81xx_trans.h"
#include "debug_msg.h"
#define HZ	100


/* for test case summary */
struct test_case {
	char test_case_id[15];
	char result[50];
};

struct test_case test_summary[20];




#if defined(INTEGRITY) || defined(THPT)

unsigned long long byte_recv; /* send by remote peer */
unsigned long long byte_rx; /* read from remote peer */

/* information for dedicated buffer */
struct fix_trans {
	struct dedicated_buf bufd;
	unsigned int out_phy;
	char *user_buf;
	unsigned int size_buf;
};

#endif

#ifdef INTEGRITY
/* file used for data integrity testing */
FILE                    *fp1, *fp2;
#endif


/*
 * global variable ( each bit of this 32 bit integer represents
 * status of corresponding outbound region.
 * if bit is set -  free
 * else -- in use
 */

unsigned int		status;

/* other globals*/
char			*mapped_buffer;
char			*mapped_pci;
char			*mapped_dma, *mapped_dma_recv;
unsigned int		*rm_info;
struct ti81xx_mgmt_area  mgmt_area;
struct pollfd		fds[1];
int			fd_dma;
int			fd;
struct ti81xx_ptrs	ptr;
pthread_mutex_t mutex_int = PTHREAD_MUTEX_INITIALIZER;
int			counter;
sem_t			mutex;
unsigned int		intr_cap; /* interrupt capability of this peer */
unsigned int fin_time1, cur_time1, fin_time2, cur_time2;
struct dma_cnt_conf	dma_cnt;
struct dma_buf_info	dma_b;
int integrity_test;

char pattern[200] = {"## Endpoint marker ## "};
int debug_test = 0;
int bar_chosen = 2;

void send_data_by_cpu()
{
	int ret;
	unsigned int offset;

ACCESS_MGMT:
	while ((ret = access_mgmt_area((u32 *)mapped_pci,
					mgmt_area.my_unique_id)) < 0) {
		debug_print("mgmt_area access not granted\n");
		sleep(2);
		continue;
	}

	while ((ret = get_free_buffer((u32 *)mapped_pci)) < 0) {
		debug_print("buffer not available\n");
		release_mgmt_area((u32 *)mapped_pci);
		sleep(2);
		goto ACCESS_MGMT;
	}

	offset = offset_to_buffer((u32 *)mapped_pci, ret);

	send_to_remote_buf_by_cpu((u32 *)mapped_pci, offset, ret);
	debug_print("send by cpu successful\n");
	release_mgmt_area((u32 *)mapped_pci);
}


void send_data_by_dma()
{
	int ret;
	unsigned int offset;

ACCESS_MGMT:
	while ((ret = access_mgmt_area((u32 *)mapped_pci,
					mgmt_area.my_unique_id)) < 0) {
		debug_print("mgmt area access not granted\n");
		sleep(3);
		continue;
	}

	while ((ret = get_free_buffer((u32 *)mapped_pci)) < 0) {
		debug_print("buffer not available\n");
		release_mgmt_area((u32 *)mapped_pci);
		sleep(3);
		goto ACCESS_MGMT;
	}

	offset = offset_to_buffer((u32 *)mapped_pci, ret);
	send_to_remote_buf_by_dma((u32 *)mapped_pci, 0x20000000,
							offset, ret, fd_dma);
	debug_print("send by dma successful\n");
	release_mgmt_area((u32 *)mapped_pci);
}

void process_local_rmt_bufs()
{
	int ret;
	while ((ret = access_mgmt_area((u32 *)mapped_pci,
					mgmt_area.my_unique_id)) < 0) {
		debug_print("mgmt area access not granted\n");
		sleep(4);
		continue;
	}

	process_remote_buffer_for_data((u32 *)mapped_pci, EDMA, fd_dma);
	release_mgmt_area((u32 *)mapped_pci);

	while ((ret = access_mgmt_area((u32 *)mapped_buffer,
					mgmt_area.my_unique_id)) < 0) {
		debug_print("mgmt area access not granted\n");
		sleep(4);
		continue;
	}

	ti81xx_poll_for_data(&ptr, &mgmt_area, mapped_buffer, NULL, NULL);
	release_mgmt_area((u32 *)mapped_buffer);

}


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


#if defined(INTEGRITY) || defined(THPT)
/* send data to a dedicated buffer on remote peer*/

void *send_to_dedicated_buf(void *arg)
{

	struct fix_trans *tx = (struct fix_trans *) arg;
	struct dma_info info;
	int count;

#ifdef INTEGRITY
	int choice = EDMA;
#endif
	unsigned int chunk_trans;
	int ret;
	info.size = tx->size_buf;
	info.user_buf = (unsigned char *)tx->user_buf;
	info.src = 0;
	info.dest = (unsigned int *)(tx->out_phy + (tx->bufd).off_st);
	/* take size_buf to be multiple of page size */
	chunk_trans = (1024 * 1024) / (tx->size_buf);

#ifdef THPT
	/* 5BG devided by buffer size */
	chunk_trans = 5368709120llu / tx->size_buf;
	debug_print("total iteration to send data will be %u\n", chunk_trans);
	memset(mapped_dma, 78, tx->size_buf);

#endif

	ioctl(fd, TI81XX_CUR_TIME, &cur_time1);

	for (count = 0; count < chunk_trans; count++) {

#ifdef INTEGRITY
		/* A 1 MB file of known pattern will be transmit to other end*/
		memset(tx->user_buf, 0, tx->size_buf);
		set_data_to_buffer(tx->user_buf, tx->size_buf, fp1);
		/*
		   if (count %2 == 0)
		   choice = CPU;
		   else
		   choice = EDMA;
		   */
		while (*((tx->bufd).wr_ptr) != 0) {
			/*polling for wr_idx to be zero again.*/
			debug_print("polling for write index--[%u] to be zero "
					"again\n", *((tx->bufd).wr_ptr));
			sleep(1);
		}

		if (choice == EDMA) {
			debug_print("sending data to buffer at  0x%x address\n",
						(unsigned int)info.dest);
			ret = ioctl(fd_dma, TI81XX_EDMA_WRITE, &info);
			if (ret < 0) {
				err_print("edma ioctl failed,  error in dma "
							"data transfer\n");
				pthread_exit(NULL);
			}

			*((tx->bufd).wr_ptr) = ret;

			ioctl(fd, TI81XX_SEND_MSI, 0);
			debug_print("data sent %d times, bytes sent in this "
						"chunk are %d\n", count+1, ret);
		}

		if (choice == CPU) {
			memcpy(mapped_pci + (tx->bufd).off_st,
						tx->user_buf, tx->size_buf);
			/*assuming buffers size on remote peer
			* is size_buf or more than that.
			*/
			*((tx->bufd).wr_ptr) = tx->size_buf;
			ioctl(fd, TI81XX_SEND_MSI, 0);
			debug_print("data sent %d times, bytes sent in this "
					"chunk are %d\n",
						count + 1, tx->size_buf);
		}

#endif


#ifdef THPT
		while (*((tx->bufd).wr_ptr) != 0)
			/*debug_print("waiting for wr index to be zero\n");*/
#if 0
		if (count == chunk_trans-1)
			memset(tx->user_buf, 78, tx->size_buf);

		ret = ioctl(fd_dma, TI81XX_EDMA_WRITE, &info);
		if (ret < 0) {
			err_print("edma ioctl failed, error in dma data "
								"transfer\n");
			pthread_exit(NULL);
		}
#endif

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
#endif
	}
	ioctl(fd, TI81XX_CUR_TIME, &fin_time1);
#ifdef THPT
	debug_print("tx test case executed with tx %llu bytes in  %u jiffies\n",
					5368709120llu, fin_time1 - cur_time1);
	printf("THPT calculated in TX direction is : %f MBPS\n",
			(float)((5.0 * 1024 * HZ) / (fin_time1 - cur_time1)));
#endif
	/*sprintf(test_summary[16].result,"tx thpt is %f MBPS",
	*(5.0*1024*HZ)/(fin_time1-cur_time1));
	*/
	/* HZ=100 */
	pthread_exit(NULL);
}

/* read data from a dedicated buffer on remote peer */
void *read_from_dedicated_buf(void *arg)
{
	struct fix_trans *rx = (struct fix_trans *)arg;
	struct dma_info info;
	int count;
#ifdef INTEGRITY
	int choice = CPU;
#endif
	unsigned int chunk_trans;
	int ret;
	info.size = rx->size_buf;
	info.user_buf = (unsigned char *)rx->user_buf;
	info.dest = 0;
	info.src = (unsigned int *)(rx->out_phy + (rx->bufd).off_st);
	/* take size_buf to be multiple of page size */
	chunk_trans = (1024 * 1024) / (rx->size_buf);

#ifdef THPT
	chunk_trans = 5368709120llu / rx->size_buf; /* 5GB devided by buf size*/
	debug_print("total iteration to read data will be %u\n", chunk_trans);
	byte_rx = 0;
#endif

	ioctl(fd, TI81XX_CUR_TIME, &cur_time2);
	for (count = 0; count < chunk_trans; count++) {

#ifdef INTEGRITY
		while (*((rx->bufd).wr_ptr) == 0) {
			/*polling for wr_idx to be non zero again.*/
			sleep(1);
			debug_print("polling for write index--[%u] to be non "
					"zero again\n", *((rx->bufd).wr_ptr));
		}

		if (choice == EDMA) {
			debug_print("reading data from buffer at 0x%x\n",
							(unsigned int)info.src);
			ret = ioctl(fd_dma, TI81XX_EDMA_READ, &info);
			if (ret < 0) {
				err_print("edma ioctl failed, error in dma "
							"data transfer\n");
				pthread_exit(NULL);
			}
			debug_print("data read %d times, bytes read in this "
						"chunk are %d\n", count+1, ret);
			*((rx->bufd).wr_ptr) = 0;
			ioctl(fd, TI81XX_SEND_MSI, 0);
		}

		if (choice == CPU) {
			memcpy(rx->user_buf, mapped_pci + (rx->bufd).off_st,
								rx->size_buf);
			*((rx->bufd).wr_ptr) = 0;
			ioctl(fd, TI81XX_SEND_MSI, 0);
			debug_print("data read %d times bytes read in this "
						"chunk are %d\n",
							count+1, rx->size_buf);
		}
#endif

#ifdef THPT

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

#endif
	}

	ioctl(fd, TI81XX_CUR_TIME, &fin_time2);
	debug_print("RX test case executed with receving %llu bytes in "
				"%u jiffies\n", byte_rx, fin_time2-cur_time2);
#ifdef THPT
#if 0
	dump_data_in_file_n(rx->user_buf, ret, "recv.txt");
#endif

	dump_data_in_file_n(mapped_dma_recv, ret, "recv.txt");
	printf("THPT calculated in RX direction is : %f MBPS\n",
			(float)((5.0 * 1024 * HZ) / (fin_time2 - cur_time2)));
	/*sprintf(test_summary[17].result,"rx thpt is %f MBPS",
	* (5.0 * 1024 * HZ) / (fin_time2 - cur_time2));
	*/
	/* HZ=100*/
#endif
	pthread_exit(NULL);
}

int read_from_dedicated_buf_func(void *arg)
{
	struct fix_trans *rx = (struct fix_trans *)arg;
	struct dma_info info;
	int ret;
	info.size = rx->size_buf;
	info.user_buf = (unsigned char *)rx->user_buf;
	info.dest = 0;
	info.src = (unsigned int *)(rx->out_phy + (rx->bufd).off_st);

	if (*((rx->bufd).wr_ptr) != 0) {
		ret = ioctl(fd_dma, TI81XX_EDMA_READ, &info);
		if (ret < 0) {
			err_print("edma ioctl failed, error in dma "
							"data transfer\n");
			return -1;
		}
		*((rx->bufd).wr_ptr) = 0;
		/*ioctl(fd, TI81XX_SEND_MSI, 0);*/
		byte_rx += ret;
	}
	return 0;
}



#endif


int outbound_regn_mgmt_test()
{
	struct ti81xx_outb_region ob, ob1;
	unsigned int ob_size = 0;
	int i;
	ob.ob_offset_hi = 0;
	ob.ob_offset_idx = 0;

	debug_print("executing outbound test for clear mapping and "
							"request success\n");

	fd = open("/dev/ti81xx_pcie_ep", O_RDWR);
	if (fd == -1) {
		err_print("EP device file open fail\n");
		return -1;
	}

	debug_print("setting outbound region size 1 MB\n");
	ioctl(fd, TI81XX_SET_OUTBOUND_SIZE, ob_size);
	status = 0;
	if (ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status) < 0) {
		err_print("ioctl to retrieve outbound region status failed\n");
		return -1;
	}

	debug_print("status of outbound regions before  request "
							"is OX%X\n", status);
	debug_print("request for a outbound mapping of 8 MB\n");

	ob.size = 0x800000; /* request  for outbound mapping of 8MB */

	if (ti81xx_set_outbound_region(&ob, fd) == 0) {
		debug_print("outbound mapping request successfull\n");
		ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status);
		debug_print("status of outbound regions after request "
							"is OX%X\n", status);
		if (status == 0x7fffff00)
			sprintf(test_summary[2].result, "%s", "Pass");
		debug_print("clearing mapping of outbound region\n");
		ioctl(fd, TI81XX_CLR_OUTBOUND_MAP, &ob);
		ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status);
		if (status == 0x7fffffff)
			sprintf(test_summary[11].result, "%s", "Pass");
		debug_print("status of outbound regions after "
				"clearing request is OX%X\n", status);

	}

	debug_print("executing  test case for EINOB (increase "
					"outbound region size)\n");

	ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status);
	debug_print("status of outbound regions before request "
						"is OX%X\n", status);
	debug_print("sending a request for 28 MB request\n");
	ob.ob_offset_hi = 1;
	ob.ob_offset_idx = 1;
	ob.size = 0x1C00000;/* request 28 MB */
	if (ti81xx_set_outbound_region(&ob, fd) == 0) {
		debug_print("outbound mapping request successfull\n");
		ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status);
		debug_print("status of outbound regions after 28 MB "
					"request is OX%X\n", status);
	}

	ob1.ob_offset_hi = 2;
	ob1.ob_offset_idx = 2;
	ob1.size = 0x600000;/* request 6 MB */
	debug_print("sending a request for 6 MB request\n");
	if (ti81xx_set_outbound_region(&ob1, fd) == EINOB) {
		debug_print("outbound mapping request return with EINOB "
							"error code\n");
		debug_print("outbound mapping test case of "
						"EINOB is passed\n");
		ioctl(fd, TI81XX_CLR_OUTBOUND_MAP, &ob);
		sprintf(test_summary[4].result, "%s", "Pass");
	}

	debug_print("executing test case of ENMEM\n");
	ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status);
	debug_print("status of outbound regions before is OX%X\n", status);
	debug_print("sending 28 request for 1 MB each\n");

	for (i = 0; i <= 27; i++) {
		ob.ob_offset_hi = i;
		ob.ob_offset_idx = i;
		ob.size = 0x100000;
		if (ti81xx_set_outbound_region(&ob, fd) != 0)
			debug_print("test case execution failed\n");
	}


	debug_print("sending a request of 32 MB\n");
	ob1.ob_offset_hi = 1024;
	ob1.ob_offset_idx = 1024;
	ob1.size = 0x2000000;
	if (ti81xx_set_outbound_region(&ob1, fd) == ENMEM) {
		debug_print("ENMEM passed\n");
		sprintf(test_summary[3].result, "%s", "Pass");
	}
	close(fd);
	return 0;
}

static int parse_opts(int argc, char **argv)
{
	int index;
	int c;

	opterr = 0;

	while ((c = getopt (argc, argv, "vb:s:")) != -1)
	{
		switch (c)
		{
			case 'v':
				debug_test = 1;
				break;
			case 'b':
				bar_chosen = atoi(optarg);
				break;
			case 's':
			{
				int free_size = sizeof(pattern) - strlen(pattern);
				strncpy (pattern + strlen(pattern), optarg, free_size);
				break;
			}
			case '?':
				if (optopt == 'c')
				{
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				}
				else if (isprint (optopt))
				{
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				}
				else
				{
					fprintf (stderr,
					"Unknown option character `\\x%x'.\n",
					optopt);
				}
				return 1;
			default:
				abort ();
		}
	}

	for (index = optind; index < argc; index++)
	{
		printf ("Non-option argument %s\n", argv[index]);
	}
	return 0;
}


int main(int argc, char **argv)
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
#ifdef THPT
	unsigned int size_buf = 1024 * 1024;
#else
	unsigned int size_buf = 4 * 1024;
#endif

#if defined(INTEGRITY) || defined(THPT)
	dma_cnt.acnt = 256;
	dma_cnt.bcnt = 4096;
	dma_cnt.ccnt = 1;
	dma_cnt.mode = 1;
#else
	dma_cnt.acnt = 510;
	dma_cnt.bcnt = 1;
	dma_cnt.ccnt = 1 ;
	dma_cnt.mode = 0;
#endif

	struct ti81xx_pciess_regs pcie_regs;

#if defined(INTEGRITY) || defined(THPT)

	struct fix_trans rd_buf, wr_buf;
	void *send_to_dedicated_buf(void *arg);
	void *read_frm_dedicate_buf(void *arg);
	void *cpu_utilize(void *arg);

#endif

	pthread_t t1, t2, t3, t4;

	void *wait_for_int(void *);
#ifdef INTEGRITY
	void *wait_for_int_test(void *);
	void *rmt_not_cap_of_int(void *arg);
#endif
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
	#if defined(INTEGRITY) || defined(THPT)
	byte_recv = 0;
	byte_rx = 0;
	#endif
	status = 0;
	counter = 0;

	if (sem_init(&mutex, 0, 1) < 0) {
		perror("semaphore initilization failed");
		exit(0);
	}

	parse_opts(argc, argv);

	printf("debugs: %d\n", debug_test);
	printf("BAR%d used\n", bar_chosen);
#if !defined(INTEGRITY) && !defined(THPT)
	printf("check for following pattern on RC side: \n '%s' \n", pattern);
#endif

#ifdef INTEGRITY
	fp1 = fopen("ep_tx.cap", "w+");
	if (fp1 == NULL) {
		err_print("file open to transfer data failed\n");
		return -1;
	}

	fp2 = fopen("ep_rx.cap", "w+");

	if (fp2 == NULL) {
		fclose(fp1);
		err_print("file open to receive data failed\n");
		return -1;
	}

	for (ret = 0; ret <= 18; ret++) {
		sprintf(test_summary[ret].test_case_id, "%s%03d",
							"DM81XX", ret+1);
		sprintf(test_summary[ret].result, "%s", "NE/C");
	}
	sprintf(test_summary[19].test_case_id, "%s", "DM81XX16EX");
	sprintf(test_summary[19].result, "%s", "NE/C");

#endif

	fd = open("/dev/ti81xx_pcie_ep", O_RDWR);
	if (fd == -1) {
		err_print("EP device file open fail\n");
#ifdef INTEGRITY
		fclose(fp1);
		fclose(fp2);
#endif
		return -1;
	}

	/*local management area lock has been hold by EP itself.*/


	ret = ioctl(fd, TI81XX_GET_PCIE_MEM_INFO, &start_addr);

	if (ret < 0) {
		err_print("START_MGMT_AREA ioctl failed\n");
#ifdef INTEGRITY
		fclose(fp1);
		fclose(fp2);
#endif
		close(fd);
		return -1;
	}

	if (!start_addr.size || (start_addr.size < SIZE_AREA)) {
		if (!start_addr.size)
			err_print("No reserved memory available for PCIe "
					"transfers, quitting...\n");
		else
			err_print("Minimum %#x bytes required as reserved "
					"memory, quitting...\n", SIZE_AREA);

#ifdef INTEGRITY
		fclose(fp1);
		fclose(fp2);
#endif
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

	if ((void *)-1 == (void *) mapped_buffer) {
		err_print("mapping dedicated memory fail\n");
#ifdef INTEGRITY
		fclose(fp1);
		fclose(fp2);
#endif
		close(fd);
		return -1;

	}

	test = (unsigned int *)mapped_buffer;

	pcie_regs.offset = GPR0;
	pcie_regs.mode = GET_REGS;
	if (ioctl(fd, TI81XX_ACCESS_REGS, &pcie_regs) < 0) {
		err_print("GET_REGS mode ioctl failed\n unable to fetch "
						"unique id from GPR0\n");
#ifdef INTEGRITY
		fclose(fp1);
		fclose(fp2);
#endif
		close(fd);
		return -1;
	}

	if (pcie_regs.value < 2) {
		err_print("Not a valid id assigned to it\n still continue by "
					"manually assigning valid id -- 2\n");
	}
	/******* by default uid 2 is assigned to This EP ****/

	test[0] = bar_chosen;
	test[1] = bar_chosen;
	fd_dma = open("/dev/ti81xx_edma_ep", O_RDWR);
	if (fd_dma == -1) {
		err_print("EP DMA device file open fail\n");
		close(fd);
#ifdef INTEGRITY
		fclose(fp1);
		fclose(fp2);
#endif
		close(fd);
		return -1;
	}

	ret = ioctl(fd_dma, TI81XX_EDMA_SET_CNT, &dma_cnt);
	if (ret < 0) {
		debug_print("dma count setting ioctl failed\n");
		goto ERROR;
	}
	ret = ioctl(fd_dma, TI81XX_EDMA_SET_BUF_INFO, &dma_b);
	if (ret < 0) {
		debug_print("dma buffer setting ioctl failed\n");
		goto ERROR;
	}
#ifdef INTEGRITY
	create_random_pattern((1024 * 1024), 4, fp1);
#endif

	fds[0].fd = fd;

	debug_print("INFO: start address of mgmt_area is virt--%x  phy--%x\n",
			start_addr.base, start_addr.base);


	/*inbound setup to be done for inbound to be enabled. by default BAR 2*/

	pcie_regs.offset = LOCAL_CONFIG_OFFSET + (BAR0 + 0x4 * bar_chosen);
	printf("PCIE_Regs  offset: %08x (BAR%d offs: %08x) \n", pcie_regs.offset, bar_chosen, (BAR0 + 0x4 * bar_chosen));
	pcie_regs.mode = GET_REGS;
	if (ioctl(fd, TI81XX_ACCESS_REGS, &pcie_regs) < 0) {
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

	if (ti81xx_set_inbound(&in, fd) < 0) {
		err_print("setting in bound config failed\n");
		goto ERROR;
	}

	if (ti81xx_enable_in_translation(fd) < 0) {
		err_print("enable in bound failed\n");
		goto ERROR;
	}

	/*inbound setup complete.*/

	mapped_pci = (char *)mmap(0, PCIE_NON_PREFETCH_SIZE,
				PROT_READ|PROT_WRITE, MAP_SHARED,
						fd, (off_t)0x20000000);
	if ((void *)-1 == (void *) mapped_pci) {
		err_print("mapping PCIE_NON_PREFETCH memory fail\n");
		goto ERROR;
	}


	mapped_dma = (char *)mmap(0, dma_b.size,
				PROT_READ|PROT_WRITE, MAP_SHARED,
					fd, (off_t)dma_b.send_buf);
	if ((void *)-1 == (void *) mapped_dma) {
		err_print("mapping DMA tx memory fail\n");
		goto ERROR;
	}

	mapped_dma_recv = (char *)mmap(0, dma_b.size, PROT_READ|PROT_WRITE,
					MAP_SHARED, fd, (off_t)dma_b.recv_buf);
	if ((void *)-1 == (void *) mapped_dma) {
		err_print("mapping DMA rx memory fail\n");
		goto ERROR;
	}

	int_cap = (unsigned int *)mapped_pci;

	if (ti81xx_prepare_mgmt_info(&mgmt_area, size_buf) < 0) {
		err_print("prepare_mgmt_info failed\n");
		goto ERROR;
	}

	debug_print("mgmt_area.size:%08x\n", mgmt_area.size);

	rm_info = (u32 *)(mapped_buffer + mgmt_area.size);

	ti81xx_set_mgmt_area(&mgmt_area, (unsigned int *)mapped_buffer);
	debug_print("initialization of management mgmt_area complete\n");
#ifdef INTEGRITY
	dedicate_buffer((unsigned int *)mapped_buffer, 1,
						mgmt_area.no_blk, 0, RD);
#endif


	test[0] = 1; /*set lock to be accessed by RC*/
	mgmt_area.my_unique_id = test[1];

	while ((ret = access_mgmt_area((u32 *)mapped_buffer,
						test[1])) < 0) {
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

	if ((scan_mgmt_area(rm_info, muid, &ob) == -1)) {
		/*this function will fillup outbound structure accordingly.*/
		debug_print("no remote peer of this muid exist\n");
		/*handle error free resource or continue without outbound.*/
	}

	debug_print("outbound config is %x %x\n",
					ob.ob_offset_hi, ob.ob_offset_idx);
	ob.size = 4194304;/* 4 MB*/




	ti81xx_set_outbound_region(&ob, fd); /*outbound will be setup here.*/


	if (ti81xx_enable_out_translation(fd) < 0) {
		err_print("enable outbound failed\n");
		goto ERROR;
	}


	if (ti81xx_enable_bus_master(fd) < 0) {
		err_print("enable bus master fail\n");
		goto ERROR;
	}

	mgmt_area.size = test[4]; /* updated by rc application */

	release_mgmt_area((u32 *)mapped_buffer); /*release management area */


	while (test[5] == 0) {
		debug_print("INFO: waiting for interrupt capability "
					"info from remote peer\n");
		sleep(1);
		continue;
	}

	printf("***Advertised interrupt capability, EP will generate MSI.\n");
	intr_cap = 1;


	int_cap[5] = intr_cap;


#if defined(INTEGRITY) || defined(THPT)
	/*  ////////
	    if (test[5] != 1) {
		debug_print("Exiting from application\n");
		goto ERROR;
	    }*/
	/* user buffer allocation */
	rd_buf.size_buf = size_buf;
	rd_buf.out_phy = 0x20000000;
	wr_buf.size_buf = size_buf;
	wr_buf.out_phy = 0x20000000;

	rd_buf.user_buf = malloc(size_buf);
	if (rd_buf.user_buf == NULL) {
		printf("user buffer allocation failed\n");
		goto ERROR;
	}

	wr_buf.user_buf = malloc(size_buf);
	if (wr_buf.user_buf == NULL) {
		printf("user buffer allocation failed\n");
		free(rd_buf.user_buf);
		goto ERROR;
	}

	memset(rd_buf.user_buf, 66, size_buf);
	memset(wr_buf.user_buf, 66, size_buf);

#ifdef THPT
	/* find dedicated buffer for reading */
	if (find_dedicated_buffer((unsigned int *)mapped_pci,
				mgmt_area.my_unique_id, &rd_buf.bufd, WR) < 0) {
		err_print("no dedicated buffer for RX on remote peer\n");
		goto FREE_BUFFER;
	}
	debug_print("offset of buffer dedicated for RX on "
				"remote peer is %X\n", rd_buf.bufd.off_st);

#endif
	/* find dedicated buffer for transmitting data */
	if (find_dedicated_buffer((unsigned int *)mapped_pci,
				mgmt_area.my_unique_id, &wr_buf.bufd, RD) < 0) {
		err_print("no dedicated buffer for TX on remote peer\n");
		goto FREE_BUFFER;
	}

	debug_print("offset of buffer dedicated for TX on remote "
					"peer is  %X\n", wr_buf.bufd.off_st);


#ifdef INTEGRITY
	if (test[5] == 1) /* remote peer can send interrupt */
		pthread_create(&t2, &attr_t2, wait_for_int, NULL);
	if (test[5] == 2) { /*remote peer can't send interrupt i.e. system-PC*/
		unsigned int *wr_pointer =
				(mapped_buffer + 6 + 2 * mgmt_area.no_blk + 2);
		pthread_create(&t2, &attr_t2, rmt_not_cap_of_int, wr_pointer);
	}
	pthread_create(&t3, &attr_t3, process_local_buffer, NULL);
	pthread_create(&t1, &attr_t1, send_to_dedicated_buf, &wr_buf);
#endif

#ifdef THPT

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

#endif

	if (test[5] == INT) {
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
#if defined(INTEGRITY) || defined(THPT)
CLEAR_MAP:
#endif

#ifdef INTEGRITY
	pthread_join(t1, NULL);
	pthread_join(t2, NULL);
	pthread_join(t3, NULL);
#endif
#ifdef THPT
	pthread_join(t1, NULL);
	pthread_join(t4, NULL);
	debug_print("bytes recv from  dedicated buffer are %llu\n", byte_rx);
#endif
#if defined(THPT) || defined(INTEGRITY)
FREE_BUFFER:
	free(wr_buf.user_buf);
	free(rd_buf.user_buf);
#endif
#ifdef THPT

	printf("Total thpt calculated tx+rx is:  %f MBPS\n",
			(float)((5.0 * 1024 * HZ) / (fin_time2 - cur_time2))
			+ (float)((5.0 * 1024 * HZ) / (fin_time1 - cur_time1)));
#endif

ERROR:
#ifdef INTEGRITY
	fclose(fp1);
	fclose(fp2);
#endif
	close(fd);
	close(fd_dma);

	/* additional tests */
#ifdef INTEGRITY
	outbound_regn_mgmt_test();
	if (integrity_test == 0) {
		sprintf(test_summary[0].result, "%s", "Pass");
		sprintf(test_summary[5].result, "%s", "Pass");
		sprintf(test_summary[7].result, "%s", "Pass");
		sprintf(test_summary[9].result, "%s", "Pass");
		sprintf(test_summary[12].result, "%s", "Pass");
		sprintf(test_summary[13].result, "%s", "Pass");
		sprintf(test_summary[15].result, "%s", "Pass");
	}
	fd = open("/dev/ti81xx_pcie_ep", O_RDWR);
	if (fd == -1) {
		err_print("EP device file open fail unable to "
					"execute stress interrupt\n");
		return -1;
	}
	fds[0].events = POLLIN;
	fds[0].fd = fd;

	if ((scan_mgmt_area(rm_info, muid, &ob) == -1)) {
		/*this function will fillup outbound structure accordingly.*/
		debug_print("no remote peer of this muid exist\n");
	}

	debug_print("outbound config is %x %x\n",
				ob.ob_offset_hi, ob.ob_offset_idx);
	ob.size = 4194304;/* 4 MB*/

	ti81xx_set_outbound_region(&ob, fd); /*outbound will be setup here.*/
	/* to notify RC that he can send interrupt now for stress test */
	int_cap[0] = mgmt_area.my_unique_id;
	pthread_create(&t1, &attr_t1, wait_for_int_test, NULL);
	debug_print("thread for stress interrupt test is created\n");
	pthread_join(t1, NULL);

	printf("test summary on EP is :\n NE/C : means not executed or "
				"confirmed at this end:\n other test cases "
				"will be covered by demo mode\n");
	for (ret = 0; ret <= 19; ret++) {
		printf("%s\t %s\n", test_summary[ret].test_case_id,
						test_summary[ret].result);
	}
#endif
	return 0;
}





void *wait_for_int(void *arg)
{
	int ret;
	while (1) {
		ret = poll(fds, 1, 3000); /*3 sec wait time out*/

		if (ret == POLLIN) {
			sem_wait(&mutex);
			counter = 2;
			sem_post(&mutex);


		} else
			debug_print("no data in buffers -- timed out\n");
#ifdef INTEGRITY
		if (byte_recv == (1024 * 1024))
			break;
#endif

#ifdef THPT
		if (byte_rx == 0x140000000llu)
			break;
#endif

	}

	pthread_exit(NULL);
}
#ifdef INTEGRITY
void *rmt_not_cap_of_int(void *arg)
{
	unsigned int *index = arg;
	while (1) {
		while (*index == 0)
			debug_print("no data in buffer, poll continue\n");
		sem_wait(&mutex);
		counter = 2;
		sem_post(&mutex);

		if (byte_recv == (1024 * 1024))
			break;
	}
	pthread_exit(NULL);
}

#endif



void *wait_for_int_test(void *arg)
{
	int ret;
	unsigned int wake_up = 0;
	unsigned int intr_cntr = 0;
	unsigned int i = 0;
	sleep(5);
	while (1) {
		ret = poll(fds, 1, 1); /*1 mili sec wait time out*/
		if (ret == POLLIN) {
			wake_up++;
		} else {
			ioctl(fd, TI81XX_GET_INTR_CNTR, &intr_cntr);
			if (++i >= 10) { /* 60 % of sent interrupt */
				debug_print("interrupt occur %u times "
						"application wakeup %u times\n",
							intr_cntr, wake_up);
				sprintf(test_summary[19].result,
						"stress_int %u wakeup %u",
							intr_cntr, wake_up);
				pthread_exit(NULL);
			}
		}
	}
}



void *process_local_buffer(void *arg)
{
	int ret = 0;
	while (1) {
		sem_wait(&mutex);
		if (counter > 0) {

			while ((ret = access_mgmt_area((u32 *)mapped_buffer,
						mgmt_area.my_unique_id)) < 0) {
				debug_print("management area access "
							"not granted\n");
				continue;
			}
#ifdef INTEGRITY
			ti81xx_poll_for_data(&ptr, &mgmt_area,
						mapped_buffer, fp2, &byte_recv);
#elif defined(THPT)
			ti81xx_poll_for_data(&ptr, &mgmt_area,
						mapped_buffer,
							NULL, &byte_recv);
#elif defined(DISPLAY)
			ti81xx_poll_for_data(&ptr, &mgmt_area,
						mapped_buffer,
							NULL, NULL);
#endif
			release_mgmt_area((u32 *)mapped_buffer);
			counter--;
		}
		sem_post(&mutex);
#ifdef INTEGRITY
		if (byte_recv == (1024 * 1024)) {
			integrity_test = check_integrity(fp1, fp2);

			break;
		}
#endif
	}
	pthread_exit(NULL);
}


void *process_rmt_buffer(void *arg)
{
	int ret = 0;
	/*
#ifdef THPT
ioctl(fd, TI81XX_CUR_TIME, &cur_time2);
#endif*/
	while (1) {
		sem_wait(&mutex);
		if (counter > 0) {
			/*
#ifdef THPT
read_from_dedicated_buf_func(arg);
#endif*/

			while ((ret = access_mgmt_area((u32 *)mapped_pci,
						mgmt_area.my_unique_id)) < 0) {
				debug_print("access not granted\n");
				continue;
			}

			process_remote_buffer_for_data((u32 *)mapped_pci,
								EDMA, fd_dma);
			release_mgmt_area((u32 *)mapped_pci);
			if (intr_cap == 1)
				ioctl(*(unsigned int *)arg,
							TI81XX_SEND_MSI, 0);

			counter--;
		}
		sem_post(&mutex);
		/*
		   if (byte_rx == 0x140000000llu)
		   break;*/
	}
	/*
#ifdef THPT
ioctl(fd, TI81XX_CUR_TIME, &fin_time2);
debug_print("test case RX completed with reading byte_rx=%llu "
			"in %u jiffies\n",byte_rx, fin_time2-cur_time2);
dump_data_in_file_n(((struct fix_trans *)arg)->user_buf ,
		((struct fix_trans *)arg)->size_buf , "recv.txt");
#endif*/
	pthread_exit(NULL);
}


void *wait_for_data(void *arg)
{	int try = 0;
	while (1) {
		sleep(3);
		process_local_rmt_bufs();
		try++;
		debug_print("data for reading is available "
					"called %d times\n\n", try);
	}
	pthread_exit(NULL);
}




void *send_data(void *arg)
{
	int try = 0;

	while (1) {

		sleep(10);
		send_data_by_cpu();
		if (intr_cap == 1)
			ioctl(*(unsigned int *)arg, TI81XX_SEND_MSI, 0);
		send_data_by_dma();
		if (intr_cap == 1)
			ioctl(*(unsigned int *)arg, TI81XX_SEND_MSI, 0);
		debug_print("send data %d times\n", try);
		try++;
	}
	debug_print("sending data exits");
	pthread_exit(NULL);

}

void *cpu_utilize(void *arg)
{
	system("top >> cpuutilize");
	pthread_exit(NULL);
}
