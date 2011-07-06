/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
 *         Aurelien Jacquiot <a-jacquiot@ti.com>
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

#ifndef __MACH_C6X_KEYSTONE_PKTDMA_H
#define __MACH_C6X_KEYSTONE_PKTDMA_H

/* Emulation control register */
#define PKTDMA_REG_EMU_CTL		        0x08

/* CPPI Tx DMA channel control registers */
#define PKTDMA_REG_TCHAN_CFG_REG_A(x)	        (0x00 + (x) * 0x20)
#define PKTDMA_REG_TCHAN_CFG_REG_B(x)	        (0x04 + (x) * 0x20)

/* CPPI Rx DMA channel control register */
#define PKTDMA_REG_RCHAN_CFG_REG_A(x)           (0x00 + (x) * 0x20)

/* CPPI Tx DMA Scheduler Configuration register */
#define PKTDMA_REG_TCHAN_SCHED_CFG(x)	        ((x) * 0x04)

/* CPPI Rx DMA flow configuration registers */
#define PKTDMA_RX_FLOW_CFG(reg, idx)	        (((reg) * 4) + ((idx) * 0x20))
#define PKTDMA_RX_FLOW_REG_A		        0
#define PKTDMA_RX_FLOW_REG_B		        1
#define PKTDMA_RX_FLOW_REG_C		        2
#define PKTDMA_RX_FLOW_REG_D		        3
#define PKTDMA_RX_FLOW_REG_E		        4
#define PKTDMA_RX_FLOW_REG_F		        5
#define PKTDMA_RX_FLOW_REG_G		        6
#define PKTDMA_RX_FLOW_REG_H		        7

/* Descriptor type created by flows */
#define PKTDMA_DESC_TYPE_HOST		        1

/* CPPI Tx DMA channel control register A definitions */
#define PKTDMA_REG_VAL_TCHAN_A_TX_ENABLE        ((u32)1 << 31)
#define PKTDMA_REG_VAL_TCHAN_A_TX_TDOWN         (1 << 30)

/* CPPI Tx DMA channel control register B definitions */
#define PKTDMA_REG_VAL_TCHAN_B_TX_FILT_EINFO	(1 << 30)
#define PKTDMA_REG_VAL_TCHAN_B_TX_FILT_PSWORDS	(1 << 29)
#define PKTDMA_REG_TCHAN_B_SET_DEFAULT_TDOWN_QMGR(x,v)  (x) = (SET_BITFIELD((x), (v), 13, 12)
#define PKTDMA_REG_TCHAN_B_SET_DEFAULT_TDOWN_QNUM(x,v)  (x) = (SET_BITFIELD((x), (v), 11,  0)

/* CPPI Rx DMA channel control register A definitions */
#define PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE	((u32)1 << 31)
#define PKTDMA_REG_VAL_RCHAN_A_RX_TDOWN		(1 << 30)

/*
 * CPPI Tx DMA Scheduler Confuration value. This sets the priorities of
 * the channels. If set to all equal, the actual value doesn't matter
 */
#define PKTDMA_REG_VAL_TCHAN_SCHED_HIGH_PRIORITY	0 
#define PKTDMA_REG_VAL_TCHAN_SCHED_MED_HIGH_PRIORITY	1 
#define PKTDMA_REG_VAL_TCHAN_SCHED_MED_LOW_PRIORITY	2 
#define PKTDMA_REG_VAL_TCHAN_SCHED_LOW_PRIORITY		3 

/* CPPI Rx flow configuration register A */
#define PKTDMA_REG_VAL_MAKE_RX_FLOW_A(einfo,psinfo,rxerr,desc,psloc,sopOff,qmgr,qnum)  \
	(   ((einfo & 1) << 30)       |   \
            ((psinfo & 1) << 29)      |   \
            ((rxerr & 1) << 28)       |   \
            ((desc & 3) << 26)        |   \
            ((psloc & 1) << 25)       |   \
            ((sopOff & 0x1ff) << 16)  |   \
            ((qmgr & 3) << 12)        |   \
            ((qnum & 0xfff) << 0))
            
/* CPPI Rx flow configuration register B. No tags are used */
#define PKTDMA_REG_VAL_RX_FLOW_B_DEFAULT		0

/* CPPI Rx flow configuration register C. No tag replacement and no size thresholds */
#define PKTDMA_REG_VAL_RX_FLOW_C_DEFAULT		0

/* CPPI Rx flow configuration register D */
#define PKTDMA_REG_VAL_MAKE_RX_FLOW_D(fd0Qm, fd0Qnum, fd1Qm, fd1Qnum)   \
        (   ((fd0Qm & 3) << 28)         |   \
            ((fd0Qnum & 0xfff) << 16)   |   \
            ((fd1Qm & 3) << 12)         |   \
            ((fd1Qnum & 0xfff) <<  0))

/* CPPI Rx flow configuration register E */
#define PKTDMA_REG_VAL_RX_FLOW_E_DEFAULT		0

/* CPPI Rx flow configuration register F */
#define PKTDMA_REG_VAL_RX_FLOW_F_DEFAULT		0

/* CPPI Rx flow configuration register G */
#define PKTDMA_REG_VAL_RX_FLOW_G_DEFAULT		0

/* CPPI Rx flow configuration register H */
#define PKTDMA_REG_VAL_RX_FLOW_H_DEFAULT		0
            
/* Default Emulation control register value disables loopback */            
#define PKTDMA_REG_VAL_EMU_CTL_NO_LOOPBACK	        0

#endif /* __MACH_C6X_KEYSTONE_PKTDMA_H */
