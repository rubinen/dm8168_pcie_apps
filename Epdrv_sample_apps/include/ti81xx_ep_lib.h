/*
 * This file contains definition of data structures, Registers offset and
 * Ioctls used by application to access PCIESS.
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

#ifndef __TI816x_PCIE_EP_APPL__
#define __TI816x_PCIE_EP_APPL__

#include <drivers/char/ti81xx_pcie_epdrv.h>
#include <drivers/char/ti81xx_edma.h>

/* offsets of application registers  starting from 0x51000000 */
#define CMD_STATUS      0x004
#define CFG_SETUP     0x008
#define IB_BAR(x)     (0x300 + (0x10 * x))
#define IB_START_LO(x)      (0x304 + (0x10 * x))
#define IB_START_HI(x)      (0x308 + (0x10 * x))
#define IB_OFFSET(x)      (0x30c + (0x10 * x))
#define OB_SIZE       0x30
#define CFG_PCIM_WIN_CNT    32
#define CFG_PCIM_WIN_SZ_IDX   3
#define OB_OFFSET_INDEX(n)    (0x200 + (8 * n))
#define OB_OFFSET_HI(n)     (0x204 + (8 * n))
#define MSI_IRQ       0x54
#define MSI0_IRQ_STATUS     0x104
#define MSI0_IRQ_ENABLE_SET   0x108
#define MSI0_IRQ_ENABLE_CLR   0x10c
#define IRQ_EOI       0x50
#define GPR0        0x70

/* these three application register are for alternate mechanism to
 * send interrupt from ep to rc
 */

#define EP_IRQ_SET      0x64
#define EP_IRQ_CLR      0x68
#define EP_IRQ_STATUS     0x6c

/* these are for monitoring error status */
#define ERR_IRQ_STATUS_RAW    0x1c0
#define ERR_IRQ_STATUS      0x1C4
#define ERR_IRQ_ENABLE_SET    0x1C8
#define ERR_IRQ_ENABLE_CLR    0x1CC

/*configuration register these are at offset 0x1000 from 0x51000000*/
#define STATUS_COMMAND      0x4
#define VENDOR_DEVICE_ID    0x0
#define  BAR0       0x10
#define  BAR1       0x14
#define  BAR2       0x18
#define  BAR3       0x1C

/* msi capability registers-- these register are at 0x1000+0x50
 * from 0x51000000
 */

#define MSI_CAP       0x0
#define MSI_LOW32     0x4
#define MSI_UP32      0x8
#define MSI_DATA      0xC

/* power management capability register-- these are at offset
 * 0x1000+0x40 from 0x51000000
 */

#define  PMCAP        0x0
#define  PM_CTL_STAT      0x4

#define SET_REGS  1
#define GET_REGS  2
#define PAGE_SIZE_EP  4096
#define RAM_START_RC  0x80000000

#define BAR2_START  0x20800000
#define ENABLE_IN 0x4
#define ENABLE_OUT  0x2
#define LOCAL_CONFIG_OFFSET 0x1000
#define ENABLE_MASTER 0x4

#define ENMEM   1
#define EINOB   2
#define EFAIL   3

#define MB    (1024 * 1024)
#define PCIE_NON_PREFETCH_START 0x20000000
#define PCIE_NON_PREFETCH_SIZE  (256 * MB)
#define SIZE_AREA   (8 * MB)

/*
 * function decalration
 */

int ti81xx_enable_in_translation(int fd);
int ti81xx_enable_out_translation(int fd);
int ti81xx_enable_bus_master(int fd);
int ti81xx_set_inbound(struct ti81xx_inb_window *in, int fd);
int ti81xx_set_outbound_region(struct ti81xx_outb_region *ob , int fd);
int ti81xx_clear_outbound_mapping(struct ti81xx_outb_region *ob, int fd);

#endif
