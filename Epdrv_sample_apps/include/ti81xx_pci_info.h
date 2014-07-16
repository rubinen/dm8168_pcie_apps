/*
 * This file contain data structure and function declarations for RC application
 * to be used to propagate Resource info to every other EP.
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


#ifndef __RC_HLPR__
#define __RC_HLPR__


#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "debug_msg.h"

#define VENDOR "/vendor"
#define DEVICE "/device"
#define RESOURCE "/resource"
#define CLASS "/class"

#define VENDOR_ID 0x104c
#define DEVICE_ID 0xb800

struct pci_sys_info {
  unsigned int res_value[7][2];
  struct pci_sys_info *next;
};

/*
 * function declaration
 */

int add_node_in_list(struct pci_sys_info *pci_info,
          struct pci_sys_info **start);
int free_list(struct pci_sys_info *start);
int add_resource_in_list(FILE *fr, struct pci_sys_info **start);
int fetch_my_unique_id(unsigned int *mgmt_area, struct pci_sys_info *node);
int print_list(struct pci_sys_info *start);
int dump_info_on_ep(struct pci_sys_info *start, unsigned int *mgmt_area,
        unsigned int eps, unsigned int startaddr);
int get_devices(struct pci_sys_info **start);
int propagate_system_info(struct pci_sys_info *start,
        int fd, int eps, unsigned int startaddr);

#endif
