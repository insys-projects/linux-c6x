/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
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
#ifndef __MACH_C6X_KEYSTONE_PA_H
#define __MACH_C6X_KEYSTONE_PA_H

#define DEVICE_PA_BASE				0x02000000
#define DEVICE_PA_RUN_CHECK_COUNT		100
#define DEVICE_PA_NUM_PDSPS			6

#define PA_CMD_SIZE 16
#define PA_MAGIC_ID				0x0CEC11E0
#define PA_REG_MAILBOX_SLOT(pdsp, slot)		(0x00 + ((pdsp) * 0x10) + ((slot) * 0x04))
#define PA_REG_PDSP_CTL(pdsp)			(0x1000 + ((pdsp) * 0x100))
#define PA_REG_TIMER_CTL(pdsp)			(0x3000 + ((pdsp) * 0x100))
#define PA_REG_TIMER_LOAD(pdsp)			(0x3000 + ((pdsp) * 0x100) + 0x4)
#define PA_REG_TIMER_VAL(pdsp)			(0x3000 + ((pdsp) * 0x100) + 0x8)
#define PA_REG_TIMER_TIMER_IRQ(pdsp)	        (0x3000 + ((pdsp) * 0x100) + 0xC)
#define PA_MEM_PDSP_IRAM(pdsp)			(0x10000 + ((pdsp) * 0x8000))
#define PA_REG_PKTID_REV	                0x00400
#define PA_REG_PKTID_SOFT_RESET	                0x00404
#define PA_REG_PKTID_RANGE_LIM	                0x00408
#define PA_REG_PKTID_IDVAL	                0x0040C
#define PA_REG_LUT2_REV	                        0x00500
#define PA_REG_LUT2_SOFT_RESET	                0x00504
#define PA_REG_LUT2_DATA0	                0x00520
#define PA_REG_LUT2_ADD_DEL_KEY	                0x00530
#define PA_REG_LUT2_ADD_CTL	                0x00534
#define PA_REG_STATS_REV	                0x06000
#define PA_REG_STATS_SOFT_RESET	                0x06004
#define PA_REG_STATS_INCREM	                0x06008
#define PA_REG_STATS_CAPT	                0x0600C
#define PA_REG_STATS0 	                        0x06020

/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP	1
#define PA_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA_REG_VAL_PDSP_CTL_STATE               (1 << 15)
#define PA_REG_VAL_PDSP_CTL_ENABLE              (1 << 1)
#define PA_REG_VAL_PDSP_CTL_SOFT_RESET          (1 << 0)
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16)	\
						 | PA_REG_VAL_PDSP_CTL_ENABLE \
						 | PA_REG_VAL_PDSP_CTL_SOFT_RESET)

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS			4

#endif /* __MACH_C6X_KEYSTONE_PA_H */

