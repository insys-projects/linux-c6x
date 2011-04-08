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

#define QM_OK				0
#define QM_INVALID_LINKRAM_ALIGNMENT	-1
#define QM_INVALID_MEMREGION_ALIGNMENT	-2
#define QM_INVALID_LINKRAM_SIZE		-3
#define QM_INVALID_LINKRAM_RAM_SIZE	-4     


/* Memory alignment requirements (bytes) */
#define QM_LINKRAM_ALIGN	4 
#define QM_MEMR_ALIGN		16      /* Not specified in the doc */

/* The driver supports only a single descriptor size */
#define QM_DESC_SIZE_BYTES	64

/* QM setup configuration */
struct qm_config  {
	u32 link_ram_base;
	u32 link_ram_size;
	u32 mem_region_base;
	u32 mem_regnum_descriptors;
	u32 dest_q;       /* Where the initialized descriptors are placed */
};

struct qm_host_desc {
	/*
	 * Descriptor type, packet type, protocol
	 * specific region location, packet length
	 */
	u32	desc_info;  
	/* Source tag, Destination tag */
	u32	tag_info;
	/* EPIB present, PS valid word count, error flags,
	 * PS flags, return policy, return push policy, 
	 * packet return QM number, packet return queue number
	 */
	u32	packet_info;
	/* Number of valid data bytes in the buffer */
	u32	buff_len;
	/*
	 * Byte aligned memory address of the buffer
	 * associated with this descriptor
	 */
	u32	buff_ptr;
	/*
	 * 32-bit word aligned memory address of the
	 * next buffer descriptor
	 */
	u32	next_bdptr;
	/* Completion tag, original buffer size */
	u32	orig_buff_len;
	/* Original buffer pointer */
	u32	orig_buff_ptr;
	/* Optional EPIB word0 */
	u32	time_stamp;
	/* Optional EPIB word1 */
	u32	software_info0;
	/* Optional EPIB word2 */
	u32	software_info1;
	/* Optional EPIB word3 */
	u32	software_info2;
	/* Optional protocol specific data */
	u32	ps_data;
	/* SW data */
	u32     private;
};

#define DEVICE_QM
#define DEVICE_QM_MANAGER_BASE		0x02a68000
#define DEVICE_QM_DESC_SETUP_BASE	0x02a6a000
#define DEVICE_QM_MANAGER_QUEUES_BASE	0x02a20000
#define DEVICE_QM_MANAGER_Q_PROXY_BASE	0x02a40000
#define DEVICE_QM_QUEUE_STATUS_BASE	0x02a00000
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

#define DEVICE_QM_PA_CFG_Q		640 /* PA configuration queue */
#define DEVICE_QM_FREE_Q		910 /* Free buffer desc queue */
#define DEVICE_QM_RX_Q		        911 /* Ethernet Rx free desc queue */
#define DEVICE_QM_ETH_RX_Q		912 /* Ethernet Rx queue (filled by PA) */
#define DEVICE_QM_TX_Q	         	913 /* Ethernet Tx completion queue */
#define DEVICE_QM_ETH_TX_Q		648 /* Ethernet Tx queue (for PA) */

/* Prototypes */
struct qm_host_desc *hw_qm_queue_pop(u32 qnum);
void                 hw_qm_queue_push(struct qm_host_desc *hd, u32 qnum, u32 desc_size);
int                  hw_qm_setup(struct qm_config *cfg);
u32                  hw_qm_queue_count(u32 qnum);
void                 hw_qm_teardown(void);
int                  hw_qm_init_threshold(u32 qnum);

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
#endif /* __MACH_C6X_KEYSTONE_QMSS_H */

