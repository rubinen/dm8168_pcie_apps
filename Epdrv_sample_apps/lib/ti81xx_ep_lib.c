/*
 * This File contain functions definition related to application running
 * on EP. functions defined in this file are mainly for PCIESS access and
 * controll.
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


#include "ti81xx_ep_lib.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include "debug_msg.h"

/*
 * status of outbound regions is represented by
 * corresponding bit of this global variable
 */

extern unsigned int status;

/**
 * ti81xx_enable_in_translation()-- enable in bound translation mechanism
 * @fd: file descriptor
 *
 * return value-- 0 on success, -1 for failure.
 */

int ti81xx_enable_in_translation(int fd)
{
  struct ti81xx_pciess_regs regs;
  regs.offset = CMD_STATUS;
  regs.mode = GET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &regs) < 0) {
    err_print("GET_REGS mode ioctl failed\n");
    return -1;
  }
  regs.value = regs.value | ENABLE_IN;
  regs.mode = SET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &regs) < 0) {
    err_print("SET_REGS mode ioctl failed\n");
    return -1;
  }
  debug_print("inbound translation enabled\n");
  return 0;
}

/**
 * ti81xx_enable_out_translation()-- enable out bound translation mechanism
 * @fd: file descriptor
 *
 * return value-- 0 on success, -1 for failure.
 */

int ti81xx_enable_out_translation(int fd)
{
  struct ti81xx_pciess_regs regs;
  regs.offset = CMD_STATUS;
  regs.mode = GET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &regs) < 0) {
    err_print("GET_REGS mode ioctl failed\n");
    return -1;
  }
  regs.value = regs.value | ENABLE_OUT;
  regs.mode = SET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &regs) < 0) {
    err_print("SET_REGS mode ioctl failed\n");
    return -1;
  }
  debug_print("outbound translation enabled\n");
  return 0;
}


/**
 * ti81xx_enable_bus_master()-- enable bus mastering capability
 * @fd: file descriptor
 *
 * return value-- 0 on success, -1 for failure.
 */

int ti81xx_enable_bus_master(int fd)
{
  struct ti81xx_pciess_regs regs;
  regs.offset = LOCAL_CONFIG_OFFSET + STATUS_COMMAND;
  regs.mode = GET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &regs) < 0) {
    err_print("GET_REGS mode ioctl failed\n");
    return -1;
  }
  regs.value = regs.value | ENABLE_MASTER;
  regs.mode = SET_REGS;
  if (ioctl(fd, TI81XX_ACCESS_REGS, &regs) < 0) {
    err_print("SET_REGS mode ioctl failed\n");
    return -1;
  }
  debug_print("bus master enable\n");
  return 0;
}


/**
 * ti81xx_set_inbound()-- setup inbound translation window
 * @in: pointer to struct ti81xx_inb_window, that contain info related to
 * inbound configuration
 * @fd: file descriptor
 *
 * return value-- 0 on success, -1 for failure.
 */

int ti81xx_set_inbound(struct ti81xx_inb_window *in , int fd)
{
  debug_print("setting up inbound window\n");
  if (ioctl(fd, TI81XX_SET_INBOUND, in) < 0) {
    err_print("setting in-bound window failed\n");
    return -1;
  }
  return 0;
}

/**
 * ti81xx_set_outbound_region()-- setup outbound translation region
 * @ob: pointer to struct ti81xx_outb_region, that contain info related
 *  to outbound configuration
 * @fd: file descriptor
 *
 */

int ti81xx_set_outbound_region(struct ti81xx_outb_region *ob , int fd)
{
  int ret;
  ret = ioctl(fd, TI81XX_SET_OUTBOUND, ob);
  if (ret == ENMEM)
    debug_print("ENMEM -- no memory available\n");
  else if (ret == EFAIL)
    debug_print("EFAIL-- some error occurs\n");
  else if (ret == EINOB)
    debug_print("EINOB -- increase size to get this"
            " request completed\n");
  else if (ret == 0) {
    debug_print("outbound mapping request accepted\n");
    ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status);/*just for debug purpose*/
    debug_print("status of outbound regions is 0X%x\n", status);
  }
  return ret;
}

/**
 * ti81xx_clear_outbound_mapping() -- clear outbound mapping
 * @ob: pointer to struct ti81xx_outb_region, that contain info related to
 *  outbound configuration.
 * that have to be released.
 * @fd: file descriptor
 *
 */


int ti81xx_clear_outbound_mapping(struct ti81xx_outb_region *ob, int fd)
{
  int ret = 0;
  printf("enter details  for clear mapping\n1> hi 32-bit address:"
            "\n2> low 32-bit addrees:\n");
  scanf("%u %u", &ob->ob_offset_hi, &ob->ob_offset_idx);
  ret = ioctl(fd, TI81XX_CLR_OUTBOUND_MAP, ob);
  /*put retuirn value check on ioctls*/
  ret = ioctl(fd, TI81XX_GET_OUTBOUND_STATUS, &status); /*just for debug purpose*/
  debug_print("status of outbound regions is:  0x%x\n", status);
  return ret;
}
