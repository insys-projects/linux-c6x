/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MACH_C6X_PA_H
#define __MACH_C6X_PA_H

#define DEVICE_PA_BASE				0x02000000
#define DEVICE_PA_RUN_CHECK_COUNT		100
#define DEVICE_PA_NUM_PDSPS			6
#define PA_MAGIC_ID				0x0CEC11E0
#define PA_REG_MAILBOX_SLOT(pdsp, slot)		(0x00 + ((pdsp) * 0x10) + ((slot) * 0x04))
#define PA_REG_PDSP_CTL(pdsp)			(0x1000 + ((pdsp) * 0x100))
#define PA_MEM_PDSP_IRAM(pdsp)			(0x10000 + ((pdsp) * 0x8000))

/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16) | 0x3)
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP	0

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS			4

struct pa_config {
	/* 32 most significant bits of the mac address */
	u32  mac0_ms;
	/*
	 * 32 least significant bits of the mac address,
	 * in the 16msbs of this word
	 */
	u32  mac0_ls;
	/* 32 most significant bits of the mac address */
	u32  mac1_ms;
	/*
	 * 32 least significant bits of the mac address,
	 * in the 16 msbs of this word
	 */
	u32  mac1_ls;
	/* Receive packet queue number */
	u32  rx_qnum;
	/* Buffer used to create PA command */
	u8   *cmd_buf;
};

#endif /* __MACH_C6X_PA_H */

