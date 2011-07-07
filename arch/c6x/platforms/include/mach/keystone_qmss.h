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
#ifndef __MACH_C6X_KEYSTONE_QMSS_H
#define __MACH_C6X_KEYSTONE_QMSS_H

#define QMSS_LOW_PRIORITY_QUEUE_BASE       	0
#define QMSS_MAX_LOW_PRIORITY_QUEUE         	512
#define QMSS_AIF_QUEUE_BASE                     512         
#define QMSS_MAX_AIF_QUEUE                  	128    
#define QMSS_PASS_QUEUE_BASE                    640
#define QMSS_MAX_PASS_QUEUE                 	12
#define QMSS_INTC_QUEUE_BASE                    652
#define QMSS_MAX_INTC_QUEUE                 	20
#define QMSS_SRIO_QUEUE_BASE                    672
#define QMSS_MAX_SRIO_QUEUE                 	16
#define QMSS_FFTC_A_QUEUE_BASE                  688
#define QMSS_MAX_FFTC_A_QUEUE               	4
#define QMSS_FFTC_B_QUEUE_BASE                  692
#define QMSS_MAX_FFTC_B_QUEUE               	4
#define QMSS_HIGH_PRIORITY_QUEUE_BASE           704
#define QMSS_MAX_HIGH_PRIORITY_QUEUE        	32
#define QMSS_STARVATION_COUNTER_QUEUE_BASE  	736
#define QMSS_MAX_STARVATION_COUNTER_QUEUE   	64
#define QMSS_INFRASTRUCTURE_QUEUE_BASE      	800
#define QMSS_MAX_INFRASTRUCTURE_QUEUE       	32
#define QMSS_TRAFFIC_SHAPING_QUEUE_BASE     	832
#define QMSS_MAX_TRAFFIC_SHAPING_QUEUE      	32
#define QMSS_FFTC_C_QUEUE_BASE			864
#define QMSS_MAX_FFTC_C_QUEUE               	4
#define QMSS_BCP_QUEUE_BASE			868
#define QMSS_MAX_BCP_QUEUE               	8
#define QMSS_VUSR_QUEUE_BASE			864
#define QMSS_MAX_VUSR_QUEUE			32
#define QMSS_MAX_GENERAL_PURPOSE_QUEUE      	7296
#define QMSS_MAX_PDSP                           2

enum qmss_queuetype
{
	/* Low priority queue */
	QMSS_QUEUETYPE_LOW_PRIORITY_QUEUE = 0,
	/* AIF queue */
	QMSS_QUEUETYPE_AIF_QUEUE,
	/* PASS queue */
	QMSS_QUEUETYPE_PASS_QUEUE,
	/* INTC pending queue */
	QMSS_QUEUETYPE_INTC_QUEUE,
	/* SRIO queue */
	QMSS_QUEUETYPE_SRIO_QUEUE,
	/* FFTC queue A */
	QMSS_QUEUETYPE_FFTC_A_QUEUE,
	/* FFTC queue B */
	QMSS_QUEUETYPE_FFTC_B_QUEUE,
	/* High priority queue */
	QMSS_QUEUETYPE_HIGH_PRIORITY_QUEUE,
	/* starvation counter queue */
	QMSS_QUEUETYPE_STARVATION_COUNTER_QUEUE,
	/* Infrastructure queue */
	QMSS_QUEUETYPE_INFRASTRUCTURE_QUEUE,
	/* Traffic shaping queue */
	QMSS_QUEUETYPE_TRAFFIC_SHAPING_QUEUE,
	/* General purpose queue */
	QMSS_QUEUETYPE_GENERAL_PURPOSE_QUEUE
};

/* Memory alignment requirements (bytes) */
#define QM_LINKRAM_ALIGN	4 
#define QM_MEMR_ALIGN		16      /* Not specified in the doc */

/* The driver supports only a single descriptor size */
#define QM_DESC_SIZE_BYTES	64

#define QM_ACC_CMD_ENABLE       0x81
#define QM_ACC_CMD_DISABLE      0x80

#define QM_ACC_CMD_SIZE         (5 * 4)

#define DEVICE_QM
#define DEVICE_QM_QUEUE_STATUS_BASE	0x02a00000
#define DEVICE_QM_MANAGER_QUEUES_BASE	0x02a20000
#define DEVICE_QM_MANAGER_Q_PROXY_BASE	0x02a40000
#define DEVICE_QM_PDSP1_IRAM_BASE       0x02a60000
#define DEVICE_QM_PDSP2_IRAM_BASE       0x02a61000
#define DEVICE_QM_MANAGER_BASE		0x02a68000
#define DEVICE_QM_DESC_SETUP_BASE	0x02a6a000
#define DEVICE_QM_PDSP1_CTRL_BASE       0x02a6e000
#define DEVICE_QM_PDSP2_CTRL_BASE       0x02a6f000
#define DEVICE_QM_INTD_BASE             0x02aa0000
#define DEVICE_QM_PDSP1_CMD_BASE        0x02ab8000
#define DEVICE_QM_PDSP2_CMD_BASE        0x02abc000

#define DEVICE_QM_PDSP_CTRL_BASE(x)     (DEVICE_QM_PDSP1_CTRL_BASE \
					 + ((DEVICE_QM_PDSP2_CTRL_BASE - DEVICE_QM_PDSP1_CTRL_BASE) \
					    * (x)))
#define DEVICE_QM_PDSP_CMD_BASE(x)      (DEVICE_QM_PDSP1_CMD_BASE \
					 + ((DEVICE_QM_PDSP2_CMD_BASE - DEVICE_QM_PDSP1_CMD_BASE) \
					    * (x)))
#define DEVICE_QM_PDSP_IRAM_BASE(x)     (DEVICE_QM_PDSP1_IRAM_BASE \
					 + ((DEVICE_QM_PDSP2_IRAM_BASE - DEVICE_QM_PDSP1_IRAM_BASE) \
					    * (x)))

#define DEVICE_QM_NUM_LINKRAMS		2
#define DEVICE_QM_NUM_MEMREGIONS	20

/*
 * Descriptor Info: Descriptor type is host
 * with any protocol specific info in the descriptor
 */
#define QM_DESC_TYPE_HOST		0
#define QM_DESC_PSINFO_IN_DESCR		0
#define QM_DESC_DEFAULT_DESCINFO	(QM_DESC_TYPE_HOST << 30)    |  \
					(QM_DESC_PSINFO_IN_DESCR << 22)
#define QM_DESC_INFO_GET_PSINFO_LOC(x)	READ_BITFIELD((x), 22, 22)
#define QM_DESC_DESCINFO_SET_PKT_LEN(x,v)	(x) = SET_BITFIELD((x), (v), 21, 0)
#define QM_DESC_DESCINFO_GET_PKT_LEN(x)	READ_BITFIELD((x), 21, 0)

/* Packet Info */
#define QM_DESC_PINFO_EPIB		1
#define QM_DESC_PINFO_RETURN_OWN	1
#define QM_DESC_DEFAULT_PINFO		(QM_DESC_PINFO_EPIB << 31) | \
					(QM_DESC_PINFO_RETURN_OWN << 15)
#define QM_PKT_INFO_GET_EPIB(x)		READ_BITFIELD((x), 31, 31)
#define QM_PKT_INFO_SET_PSINFO_SIZE(x,v)    (x) = SET_BITFIELD((x), (v), 29, 24)
#define QM_DESC_PINFO_SET_QM(x,v)	(x) = SET_BITFIELD((x), (v), 13, 12)
#define QM_DESC_PINFO_SET_QUEUE(x,v)    (x) = SET_BITFIELD((x), (v), 11,  0)
#define QM_REG_REVISION			0x00
#define QM_REG_DIVERSION		0x08
#define QM_REG_LINKRAM_BASE(x)		(0x0c + 8*(x))
#define QM_REG_LINKRAM_SIZE(x)		(0x10 + 8*(x))

/* The queue peek registers (includes thresholds) */
#define QM_REG_STAT_CFG_REGD(x)		(0xc + 16*(x))

/* Relative to the descriptor setup region */
#define QM_REG_MEMR_BASE_ADDR(x)	(0x00 + 16*(x))
#define QM_REG_MEMR_START_IDX(x)	(0x04 + 16*(x))
#define QM_REG_MEMR_DESC_SETUP(x)	(0x08 + 16*(x))

/* Queues, register A */
#define QM_REG_QUEUE_REGA(x)		(0x00 + 16*(x))
#define QM_QA_ENTRY_COUNT_MSB		18
#define QM_QA_ENTRY_COUNT_LSB		0

/* Queues, register D */
#define QM_REG_QUEUE_REGD(x)		(0x0c + 16*(x))

/* Description region setup */
#define QM_REG_VAL_DESC_SETUP_SET_DESC_SIZE(x,v)  (x) = SET_BITFIELD((x),((v) >> 4)-1, 28, 16)

/* Maximum linking RAM size mask */
#define QM_REG_LINKRAM_SIZE_MAX_MASK	0x7ffff

/* QM PDSP control registers */
#define QM_REG_PDSP_CONTROL_REG         0x00
#define QM_REG_PDSP_STATUS_REG          0x04
#define QM_REG_PDSP_WAKEUP_ENABLE_REG   0x08
#define QM_REG_PDSP_CYCLE_COUNT_REG     0x0c
#define QM_REG_PDSP_STALL_COUNT_REG     0x10

#define QM_REG_VAL_PDSP_CTL_DISABLE     0x0000
#define QM_REG_VAL_PDSP_CTL_ENABLE(pc)  (((pc) << 16) | 0x3)
#define QM_REG_VAL_PDSP_CTL_STATE       0x8000

/* The QM INTD registers */
#define QM_REG_INTD_EOI		        0x010
#define QM_REG_INTD_STATUS0	        0x200
#define QM_REG_INTD_STATUS1	        0x204
#define QM_REG_INTD_STATUS4	        0x210
#define QM_REG_INTD_CLEAR0	        0x280
#define QM_REG_INTD_CLEAR1	        0x284
#define QM_REG_INTD_CLEAR4	        0x290
#define QM_REG_INTD_COUNT               0x300

#define QM_REG_INTD_COUNT_IRQ(i)        (DEVICE_QM_INTD_BASE + QM_REG_INTD_COUNT + (i << 2))

#define QM_REG_INTD_EOI_STARV_INDEX     0
#define QM_REG_INTD_EOI_HIGH_PRIO_INDEX 2
#define QM_REG_INTD_EOI_LOW_PRIO_INDEX  34

/* Queue definitions */
#define QM_LOW_PRIO_QUEUE               0
#define QM_HIGH_PRIO_QUEUE              704
#define QM_STARV_QUEUE                  736

/* Return queue for a given accumulation channel  */
#define QM_HIGH_PRIO_CHAN_MAP(c)        (QM_HIGH_PRIO_QUEUE + (c))

/* Return the channel for a given channel idx on the current core */
#define QM_HIGH_PRIO_IDX_MAP(i)         (((i) * CORE_NUM) + get_coreid())

/* Helper functions */
static inline int address_is_local(u32 addr)
{
	/* L2 */
	if ((addr >= RAM_SRAM) && (addr < (RAM_SRAM + RAM_SRAM_SIZE)))
		return 1;
    
	/* L1P */
	if ((addr >= 0x00e00000) && (addr < 0x00e08000))
		return 1;
	
	/* L1D */
	if ((addr >= 0x00f00000) && (addr < 0x00f08000))
		return 1;

	return 0;
}

static inline u32 device_local_addr_to_global(u32 addr)
{
	if (address_is_local (addr))
		addr = (1 << 28) | (get_coreid() << 24) | addr;
	
	return addr;
}

#define BITMASK(x,y)	                (((((u32)1 << (((u32)x)-((u32)y)+(u32)1)) \
					   - (u32)1 ))   <<  ((u32)y))
#define READ_BITFIELD(z,x,y)	        ((((u32)z) & BITMASK(x,y)) >> (y))
#define SET_BITFIELD(z,f,x,y)	        ((((u32)z) & ~BITMASK(x,y)) |	\
					 ((((u32)f) << (y)) & BITMASK(x,y)))

/*
 * Use MSM for descriptor/accumulator memory
 */
#include <asm/msmc.h>

static inline u32 qm_desc_ptov(u32 p) {
	return (p == 0) ?
		0 : msm_phys_to_virt(p);
}

static inline u32 qm_desc_vtop(u32 v) {
	return (v == 0) ?
		0 : msm_virt_to_phys(v);
}

static inline u32 qm_mem_alloc(size_t size, u32 *paddr) {
	return (u32) msm_alloc_coherent(size, (dma_addr_t *) paddr);
}

#endif /* __MACH_C6X_KEYSTONE_QMSS_H */

