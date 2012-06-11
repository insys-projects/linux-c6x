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
#define DEVICE_QM_BASE				0x02a00000
#define DEVICE_QM_SIZE				0xc0000
#define DEVICE_QM_QUEUE_STATUS_OFFSET		0x00000
#define DEVICE_QM_MANAGER_QUEUES_OFFSET		0x20000
#define DEVICE_QM_MANAGER_Q_PROXY_OFFSET        0x40000
#define DEVICE_QM_PDSP1_IRAM_OFFSET		0x60000
#define DEVICE_QM_PDSP2_IRAM_OFFSET		0x61000
#define DEVICE_QM_MANAGER_OFFSET		0x68000
#define DEVICE_QM_DESC_SETUP_OFFSET		0x6a000
#define DEVICE_QM_CDMA_GLOBAL_CFG_OFFSET	0x6c000
#define DEVICE_QM_CDMA_TX_CHAN_CFG_OFFSET	0x6c400
#define DEVICE_QM_CDMA_RX_CHAN_CFG_OFFSET	0x6c800
#define DEVICE_QM_CDMA_TX_SCHED_CFG_OFFSET      0x6cc00
#define DEVICE_QM_CDMA_RX_FLOW_CFG_OFFSET	0x6d000
#define DEVICE_QM_PDSP1_CTRL_OFFSET		0x6e000
#define DEVICE_QM_PDSP2_CTRL_OFFSET		0x6f000
#define DEVICE_QM_INTD_OFFSET			0xa0000
#define DEVICE_QM_PDSP1_CMD_OFFSET		0xb8000
#define DEVICE_QM_PDSP2_CMD_OFFSET		0xbc000


#define DEVICE_QM_PDSP_CTRL_OFFSET(x)	        (DEVICE_QM_PDSP1_CTRL_OFFSET + \
						((DEVICE_QM_PDSP2_CTRL_OFFSET - \
						DEVICE_QM_PDSP1_CTRL_OFFSET) * (x)))
#define DEVICE_QM_PDSP_CMD_OFFSET(x)            (DEVICE_QM_PDSP1_CMD_OFFSET  + \
						((DEVICE_QM_PDSP2_CMD_OFFSET - \
						DEVICE_QM_PDSP1_CMD_OFFSET) * (x)))
#define DEVICE_QM_PDSP_IRAM_OFFSET(x)           (DEVICE_QM_PDSP1_IRAM_OFFSET + \
						((DEVICE_QM_PDSP2_IRAM_OFFSET - \
						DEVICE_QM_PDSP1_IRAM_OFFSET) * (x)))

#define DEVICE_QM_NUM_LINKRAMS		2
#define DEVICE_QM_NUM_MEMREGIONS	20

/* The driver supports only a single descriptor size */
#define DEVICE_QM_DESC_SIZE_BYTES       128
#define DEVICE_QM_NUM_DESCS             4096 /* total number of queue descriptors */
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
#define QM_DESC_DINFO_SET_PSINFO_LOC(x,v) (x) = SET_BITFIELD((x), (v), 22, 22)
#define QM_DESC_DINFO_GET_PSINFO_LOC(x)	READ_BITFIELD((x), 22, 22)
#define QM_DESC_DINFO_SET_PKT_LEN(x,v)	(x) = SET_BITFIELD((x), (v), 21, 0)
#define QM_DESC_DINFO_GET_PKT_LEN(x)	READ_BITFIELD((x), 21, 0)
#define QM_DESC_DINFO_SET_PKT_TYPE(x,v) (x) = SET_BITFIELD((x), (v), 29, 25)
#define QM_DESC_DINFO_GET_PKT_TYPE(x)   READ_BITFIELD((x), 29, 25)

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

#define QM_REG_INTD_COUNT_IRQ(i)        (DEVICE_QM_INTD_OFFSET + QM_REG_INTD_COUNT + ((i) << 2))

#define QM_REG_INTD_EOI_STARV_INDEX     0
#define QM_REG_INTD_EOI_HIGH_PRIO_INDEX 2
#define QM_REG_INTD_EOI_LOW_PRIO_INDEX  34

/* Return queue for a given accumulation channel  */
#define QM_HIGH_PRIO_CHAN_MAP(c)        (DEVICE_QM_HIGH_PRIO_Q + (c))
#define QM_LOW_PRIO_CHAN_MAP(c)         (DEVICE_QM_LOW_PRIO_Q + ((c) << 5))

/* Return the channel for a given channel idx on a givent core */
#define QM_HIGH_PRIO_IDX_MAP(i, c)      (((i) * CORE_NUM) + (c))
#define QM_LOW_PRIO_IDX_MAP(i, c)       (i)

/* Offset of the low priority channels in combined channel firmware */
#define QM_LOW_PRIO_CHANNEL_OFFSET      32
#define QM_HIGH_PRIO_CHANNEL_OFFSET     0

/* Queue definitions for the device */
#define DEVICE_QM_LOW_PRIO_Q            0
#define DEVICE_QM_AIF_Q                 512
#define DEVICE_QM_PA_Q		        640
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
#define DEVICE_QM_VUSR_ALLOC_Q          4096 /* First allocable queue, note that 
						these queues cannot be used for PKTDMA */
#define DEVICE_QM_VUSR_ALLOC_END_Q      8191 /* Last allocable queue */

/* QPEND definitions (devices dependent) */
#ifdef CONFIG_SOC_TMS320C6678
#define DEVICE_QM_PEND12                652
#define DEVICE_QM_PEND13                653
#define DEVICE_QM_PEND14                654
#define DEVICE_QM_PEND15                655
#define DEVICE_QM_PEND16                656
#define DEVICE_QM_PEND17                657
#define DEVICE_QM_PEND18                658
#define DEVICE_QM_PEND19                659
#define DEVICE_QM_PEND20                660
#define DEVICE_QM_PEND21                661
#endif
#define DEVICE_QM_PEND22                662
#define DEVICE_QM_PEND23                663
#define DEVICE_QM_PEND24                664
#define DEVICE_QM_PEND25                665
#define DEVICE_QM_PEND26                666
#define DEVICE_QM_PEND27                667
#define DEVICE_QM_PEND28                668
#define DEVICE_QM_PEND29                669
#define DEVICE_QM_PEND30                670
#define DEVICE_QM_PEND31                671

/* Generic free queue */
#define DEVICE_QM_FREE_Q		910 /* Free buffer desc queue */

/* PA queues */
#define DEVICE_QM_PA_TX_PDSP0_Q         (DEVICE_QM_PA_Q + 0)
#define DEVICE_QM_PA_TX_PDSP1_Q         (DEVICE_QM_PA_Q + 1)
#define DEVICE_QM_PA_TX_PDSP2_Q         (DEVICE_QM_PA_Q + 2)
#define DEVICE_QM_PA_TX_PDSP3_Q         (DEVICE_QM_PA_Q + 3)
#define DEVICE_QM_PA_TX_PDSP4_Q         (DEVICE_QM_PA_Q + 4)
#define DEVICE_QM_PA_TX_PDSP5_Q         (DEVICE_QM_PA_Q + 5)
#define DEVICE_QM_PA_TX_ETH_Q           (DEVICE_QM_PA_Q + 8)
#define DEVICE_QM_PA_CMD_FREE_Q         4001
#define DEVICE_QM_PA_CMD_CP_Q           4002

/* Ethernet (NetCP) queues */
#define DEVICE_QM_ETH_FREE_Q            DEVICE_QM_FREE_Q
#define DEVICE_QM_ETH_RX_FREE_Q         (DEVICE_QM_FREE_Q + 1)                              /* Ethernet Rx free desc queue */
#define DEVICE_QM_ETH_RX_Q		QM_HIGH_PRIO_CHAN_MAP(DEVICE_QM_ETH_ACC_RX_CHANNEL) /* Ethernet Rx queue (filled by PA) */
#define DEVICE_QM_ETH_TX_Q		DEVICE_QM_PA_TX_ETH_Q                               /* Ethernet Tx queue (for PA) */
#define DEVICE_QM_ETH_TX_CP_Q		QM_HIGH_PRIO_CHAN_MAP(DEVICE_QM_ETH_ACC_TX_CHANNEL) /* Ethernet Tx completion queue (filled by PA) */

#define DEVICE_QM_ETH_RX_Q_I(i)         QM_HIGH_PRIO_CHAN_MAP(		\
		QM_HIGH_PRIO_IDX_MAP(DEVICE_QM_ETH_ACC_RX_IDX, netcp_coreid(i))) /* Ethernet Rx queue per NetCP instance */
#define DEVICE_QM_ETH_TX_CP_Q_I(i)      QM_HIGH_PRIO_CHAN_MAP(\
		QM_HIGH_PRIO_IDX_MAP(DEVICE_QM_ETH_ACC_TX_IDX, netcp_coreid(i))) /* Ethernet Tx completion queue per NetCP instance */

#define DEVICE_QM_ETH_RX_FREE_Q_I(i)    (DEVICE_QM_ETH_RX_FREE_Q + (i))  /* Ethernet Rx free desc queue per NetCP instance */

/* RapidIO queues */
#define DEVICE_QM_RIO_FREE_Q            DEVICE_QM_FREE_Q
#define DEVICE_QM_RIO_RX_FREE_Q         (DEVICE_QM_FREE_Q + 256)
#define DEVICE_QM_RIO_TX_Q		DEVICE_QM_SRIO_Q /* RapidIO Tx queue */

/* Accumulator channel definitions */
#define DEVICE_QM_ETH_ACC_RX_IDX        0 /* Rx Ethernet accumulator channels index */
#define DEVICE_QM_ETH_ACC_TX_IDX        1 /* Tx Ethernet accumulator channels index */

/* Accumulator channels for NetCP */
#define DEVICE_QM_ETH_ACC_RX_CHANNEL    QM_HIGH_PRIO_IDX_MAP(	\
		DEVICE_QM_ETH_ACC_RX_IDX, get_coreid())
#define DEVICE_QM_ETH_ACC_TX_CHANNEL    QM_HIGH_PRIO_IDX_MAP(	\
		DEVICE_QM_ETH_ACC_TX_IDX, get_coreid())

#define DEVICE_QM_ETH_ACC_RX_CHANNEL_I(i)				\
	(QM_HIGH_PRIO_IDX_MAP(DEVICE_QM_ETH_ACC_RX_IDX, netcp_coreid(i)) + QM_HIGH_PRIO_CHANNEL_OFFSET)
#define DEVICE_QM_ETH_ACC_TX_CHANNEL_I(i)				\
	(QM_HIGH_PRIO_IDX_MAP(DEVICE_QM_ETH_ACC_TX_IDX, netcp_coreid(i)) + QM_HIGH_PRIO_CHANNEL_OFFSET)

#define DEVICE_QM_ETH_INTD_EOI_INDEX    QM_REG_INTD_EOI_HIGH_PRIO_INDEX

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
#define DEVICE_QM_PDSP_FIRMWARE "keystone/qmss_pdsp_acc48_be_1_0_2_0.fw"
#else
#define DEVICE_QM_PDSP_FIRMWARE "keystone/qmss_pdsp_acc48_le_1_0_2_0.fw"
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

