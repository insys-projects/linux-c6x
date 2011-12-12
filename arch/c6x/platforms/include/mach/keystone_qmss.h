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

/* Memory alignment requirements (bytes) */
#define QM_LINKRAM_ALIGN	        4 
#define QM_MEMR_ALIGN		        16      /* Not specified in the doc */

/* Accumulator commands */
#define QM_ACC_CMD_ENABLE               0x81
#define QM_ACC_CMD_DISABLE              0x80

#define QM_ACC_CMD_SIZE                 (5 * 4)

/* pDSP number */
#define QM_MAX_PDSP                     2

/* QMSS register memory map */
#define DEVICE_QM
#define DEVICE_QM_BASE			0x02a00000
#define DEVICE_QM_SIZE			0xc0000
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
/* QMSS PKTDMA register memory map */
#define DEVICE_QM_CDMA_GLOBAL_CFG_BASE	 0x02a6c000
#define DEVICE_QM_CDMA_TX_CHAN_CFG_BASE	 0x02a6c400
#define DEVICE_QM_CDMA_RX_CHAN_CFG_BASE	 0x02a6c800
#define DEVICE_QM_CDMA_TX_SCHED_CFG_BASE 0x02a6cc00
#define DEVICE_QM_CDMA_RX_FLOW_CFG_BASE	 0x02a6d000

#define DEVICE_QM_NUM_LINKRAMS		2
#define DEVICE_QM_NUM_MEMREGIONS	20

/* The driver supports only a single descriptor size */
#define DEVICE_QM_DESC_SIZE_BYTES       64 
#define DEVICE_QM_NUM_DESCS             512 /* total number of queue descriptors */
#define DEVICE_QM_DESC_RAM_SIZE         (DEVICE_QM_DESC_SIZE_BYTES * DEVICE_QM_NUM_DESCS)

/*
 * Descriptor Info: Descriptor type is host
 * with any protocol specific info in the descriptor
 */

/* Desc info */
#define QM_DESC_DINFO_TYPE_HOST		0
#define QM_DESC_DINFO_PSINFO_IN_DESCR   0
#define QM_DESC_DINFO_DEFAULT	        (QM_DESC_DINFO_TYPE_HOST << 30)    | \
					(QM_DESC_DINFO_PSINFO_IN_DESCR << 22)
#define QM_DESC_DINFO_GET_PSINFO_LOC(x)	READ_BITFIELD((x), 22, 22)
#define QM_DESC_DINFO_SET_PKT_LEN(x,v)	(x) = SET_BITFIELD((x), (v), 21, 0)
#define QM_DESC_DINFO_GET_PKT_LEN(x)	READ_BITFIELD((x), 21, 0)

/* Tag info */
#define QM_DESC_TINFO_SET_S_TAG_HI(x,v) (x) = SET_BITFIELD((x), (v), 31, 24)
#define QM_DESC_TINFO_SET_S_TAG_LO(x,v) (x) = SET_BITFIELD((x), (v), 23, 16)
#define QM_DESC_TINFO_SET_D_TAG_HI(x,v) (x) = SET_BITFIELD((x), (v), 15, 8)
#define QM_DESC_TINFO_SET_D_TAG_LO(x,v) (x) = SET_BITFIELD((x), (v), 15, 0)

/* Packet info */
#define QM_DESC_PINFO_EPIB		1
#define QM_DESC_PINFO_RETURN_OWN	1
#define QM_DESC_PINFO_DEFAULT		(QM_DESC_PINFO_EPIB << 31) | \
					(QM_DESC_PINFO_RETURN_OWN << 15)
#define QM_DESC_PINFO_SET_QM(x,v)	(x) = SET_BITFIELD((x), (v), 13, 12)
#define QM_DESC_PINFO_SET_QUEUE(x,v)    (x) = SET_BITFIELD((x), (v), 11,  0)
#define QM_DESC_PINFO_GET_EPIB(x)	READ_BITFIELD((x), 31, 31)
#define QM_DESC_PINFO_SET_SIZE(x,v)     (x) = SET_BITFIELD((x), (v), 29, 24)

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

/* Return queue for a given accumulation channel  */
#define QM_HIGH_PRIO_CHAN_MAP(c)        (DEVICE_QM_HIGH_PRIO_Q + (c))

/* Return the channel for a given channel idx on the current core */
#define QM_HIGH_PRIO_IDX_MAP(i)         (((i) * CORE_NUM) + get_coreid())

/* Queue definitions for the device */
#define DEVICE_QM_LOW_PRIO_Q            0
#define DEVICE_QM_AIF_Q                 512
#define DEVICE_QM_PA_Q		        640
#define DEVICE_QM_PA_CFG_Q		640 /* PA configuration queue */
#define DEVICE_QM_PA_TX_Q               648 /* PA transmit queue */
#define DEVICE_QM_INTC_Q                652
#define DEVICE_QM_SRIO_Q                672
#define DEVICE_QM_FFTC_A_Q              688
#define DEVICE_QM_FFTC_B_Q              692
#define DEVICE_QM_HIGH_PRIO_Q           704
#define DEVICE_QM_STARV_Q               736
#define DEVICE_QM_INFRASTRUCTURE_Q 	800
#define DEVICE_QM_TRAFFIC_SHAPING_Q	832
#define DEVICE_QM_FFTC_C_Q		864
#define DEVICE_QM_BCP_Q		        868
#define DEVICE_QM_VUSR_Q		864
#define DEVICE_QM_VUSR_ALLOC_Q          4096 /* First allocable queue */
#define DEVICE_QM_VUSR_ALLOC_END_Q      8191 /* Last allocable queue */

/* Generic free queue */
#define DEVICE_QM_FREE_Q		910 /* Free buffer desc queue */

/* Ethernet (NetCP) queues */
#define DEVICE_QM_ETH_FREE_Q            DEVICE_QM_FREE_Q
#define DEVICE_QM_ETH_RX_FREE_Q         911 /* Ethernet Rx free desc queue */
#define DEVICE_QM_ETH_RX_Q		QM_HIGH_PRIO_CHAN_MAP(DEVICE_QM_ETH_ACC_RX_CHANNEL) /* Ethernet Rx queue (filled by PA) */
#define DEVICE_QM_ETH_TX_Q		DEVICE_QM_PA_TX_Q                                   /* Ethernet Tx queue (for PA) */
#define DEVICE_QM_ETH_TX_CP_Q		QM_HIGH_PRIO_CHAN_MAP(DEVICE_QM_ETH_ACC_TX_CHANNEL) /* Ethernet Tx completion queue (filled by PA) */

/* Accumulator channel definitions */
#define DEVICE_QM_ETH_ACC_RX_IDX        0   /* Rx Ethernet accumulator channel index */
#define DEVICE_QM_ETH_ACC_TX_IDX        1   /* Tx Ethernet accumulator channel index */

/* Accumulator channels */
#define DEVICE_QM_ETH_ACC_RX_CHANNEL    QM_HIGH_PRIO_IDX_MAP(DEVICE_QM_ETH_ACC_RX_IDX)
#define DEVICE_QM_ETH_ACC_TX_CHANNEL    QM_HIGH_PRIO_IDX_MAP(DEVICE_QM_ETH_ACC_TX_IDX)

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

/*
 * Platform data structures
 */
struct pdsp_platform_data {
	unsigned int pdsp;
	char        *firmware;
	int          firmware_version;
};

struct qmss_platform_data {
	u32 link_ram_base;
	u32 link_ram_size;
	u32 desc_ram_size;
	u32 desc_num;
	u32 free_queue;
	u32 slave;
	struct pdsp_platform_data qm_pdsp;
};

/*
 * Firmware 
 */
#include <asm/byteorder.h>
#ifdef CONFIG_CPU_BIG_ENDIAN
#define DEVICE_QM_PDSP_FIRMWARE "keystone-pdsp/qmss_pdsp_acc48_be.fw"
#else
#define DEVICE_QM_PDSP_FIRMWARE "keystone-pdsp/qmss_pdsp_acc48_le.fw"
#endif

/*
 * Copy a PDSP firmware image from/to PDSP memory. 
 * Firmware images are always little-endian.
 */
static inline void pdsp_fw_put(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = le32_to_cpu(*src++);
}

static inline void pdsp_fw_get(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = cpu_to_le32(*src++);
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

