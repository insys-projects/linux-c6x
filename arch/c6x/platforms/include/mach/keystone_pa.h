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
#define DEVICE_PA_REGION_SIZE                   0x48000
#define DEVICE_PA_RUN_CHECK_COUNT		100
#define DEVICE_PA_NUM_PDSPS			6

#define PA_CMD_SIZE                             16
#define PA_MAGIC_ID				0x0CEC11E0
#define PA_REG_MAILBOX_SLOT(pdsp, slot)		(0x00 + ((pdsp) * 0x10) + ((slot) * 0x04))
#define PA_REG_PDSP_CTL(pdsp)			(0x1000 + ((pdsp) * 0x100))
#define PA_REG_PDSP_CONSTANT_TBL_BLOCK_INDEX0(pdsp) (0x1000 + 0x20 + ((pdsp) * 0x100))
#define PA_REG_PDSP_CONSTANT_TBL_BLOCK_INDEX1(pdsp) (0x1000 + 0x24 + ((pdsp) * 0x100))
#define PA_REG_PDSP_CONSTANT_TABLE_PROG_PTR_0(pdsp) (0x1000 + 0x28 + ((pdsp) * 0x100))
#define PA_REG_PDSP_CONSTANT_TABLE_PROG_PTR_1(pdsp) (0x1000 + 0x2C + ((pdsp) * 0x100))
#define PA_REG_TIMER_CTL(pdsp)			(0x3000 + ((pdsp) * 0x100))
#define PA_REG_TIMER_LOAD(pdsp)			(0x3000 + ((pdsp) * 0x100) + 0x4)
#define PA_REG_TIMER_VAL(pdsp)			(0x3000 + ((pdsp) * 0x100) + 0x8)
#define PA_REG_TIMER_TIMER_IRQ(pdsp)	        (0x3000 + ((pdsp) * 0x100) + 0xC)
#define PA_MEM_PDSP_IRAM(pdsp)			(0x10000 + ((pdsp) * 0x8000))
#define PA_MEM_PDSP_SRAM(num)			(0x40000 + ((num) * 0x2000))
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
/* PA PktDMA defines */
#define DEVICE_PA_CDMA_BASE			0x02004000
#define DEVICE_PA_CDMA_SIZE			0x1400
#define DEVICE_PA_CDMA_GLOBAL_CFG_OFFSET	0x0000
#define DEVICE_PA_CDMA_TX_CHAN_CFG_OFFSET	0x0400
#define DEVICE_PA_CDMA_RX_CHAN_CFG_OFFSET	0x0800
#define DEVICE_PA_CDMA_TX_SCH_CFG_OFFSET	0x0C00
#define DEVICE_PA_CDMA_RX_FLOW_CFG_OFFSET	0x1000

#define DEVICE_PA_CDMA_TX_PDSP0_CHANNEL         0
#define DEVICE_PA_CDMA_TX_PDSP1_CHANNEL         1
#define DEVICE_PA_CDMA_TX_PDSP2_CHANNEL         2
#define DEVICE_PA_CDMA_TX_PDSP3_CHANNEL         3
#define DEVICE_PA_CDMA_TX_PDSP4_CHANNEL         4
#define DEVICE_PA_CDMA_TX_PDSP5_CHANNEL         5
#define DEVICE_PA_CDMA_TX_ETH_CHANNEL           8
#define DEVICE_PA_CDMA_TX_NUM_CHANNELS		9

#define DEVICE_PA_CDMA_RX_FIRST_CHANNEL 	0
#define DEVICE_PA_CDMA_RX_NUM_CHANNELS		24
#define DEVICE_PA_CDMA_RX_NUM_ETH_CHANNELS      (DEVICE_NETCP_NUM_INSTANCES + 1)  /* Number of channels used:
										     one per core for Ethernet
										     + one for PA commands */
#define DEVICE_PA_CDMA_RX_FIRST_FLOW 	        0
#define DEVICE_PA_CDMA_RX_NUM_FLOWS		32
#define DEVICE_PA_CDMA_RX_ETH_FLOW              0                                 /* Rx flow used for Ethernet */
#define DEVICE_PA_CDMA_RX_NUM_ETH_FLOWS         (DEVICE_NETCP_NUM_INSTANCES + 1)  /* Number of flow used:
										     one per core for Ethernet
										     + one for PA commands */
#define DEVICE_PA_CDMA_RX_FIRMWARE_FLOW         DEVICE_NETCP_NUM_INSTANCES        /* Rx flow used for PA commands */

/* Firmware */
#define DEVICE_PA_PDSP_FIRMWARE_012             "keystone/pa_pdsp02_1_0_2_1.fw"
#define DEVICE_PA_PDSP_FIRMWARE_3               "keystone/pa_pdsp3_1_0_2_1.fw"
#define DEVICE_PA_PDSP_FIRMWARE_45              "keystone/pa_pdsp45_1_0_2_1.fw"

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS			4

/* System Timestamp */
#define PAFRM_SRAM_SIZE			        0x2000		
#define PAFRM_SYS_TIMESTAMP_ADDR	        0x6460
#define PAFRM_SYS_TIMESTAMP_SRAM_INDEX	        (PAFRM_SYS_TIMESTAMP_ADDR / PAFRM_SRAM_SIZE)
#define PAFRM_SYS_TIMESTAMP_OFFSET	        ((PAFRM_SYS_TIMESTAMP_ADDR % \
						  PAFRM_SRAM_SIZE)/sizeof(u32))
#endif /* __MACH_C6X_KEYSTONE_PA_H */

