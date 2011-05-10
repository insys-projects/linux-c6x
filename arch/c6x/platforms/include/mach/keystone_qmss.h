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

#define QM_ACC_CMD_ENABLE       0x81
#define QM_ACC_CMD_DISABLE      0x80

/* Accumulator command interface structure */
struct qm_acc_cmd_config {
	/* Accumulator channel affected (0-47) */
	u8                 channel;
	/* Accumulator channel command (enable or disable) */
	u8                 command;
	/*
	 * This field specifies which queues are to be included in the queue group.
	 * Bit 0 corresponds to the base queue index, and bit 31 corresponds to the base
	 * queue index plus 31. For any bit set in this mask, the corresponding queue index
	 * is included in the monitoring function.
	 */
	u32                queue_mask;
	/* Physical pointer to list ping/pong buffer. NULL when channel disabled */
	u32                list_addr;
	/* Queue Manager and Queue Number index to monitor. This serves as a base queue index when the
	 * channel in multi-queue mode, and must be a multiple of 32 when multi-queue mode is enabled. */
	u16                queue_index;
	/* Max entries per list buffer page */
	u16                max_entries;
	/* Number of 25us timer ticks to delay interrupt */
	u16                timer_count;
	/* Interrupt pacing mode: specifies when the interrupt should be trigerred */
	u8                 pacing_mode;
	/* List entry size: specifies the size of each data entry */
	u8                 list_entry_size;
	/* List count mode: the number of entries in the list */
        u8                 list_count_mode;
	/* Queue mode: monitor single or multiple queues */
	u8                 multi_queue_mode;
};

/* QM PDSP firmware download information structure */
struct pdsp_config {
	/* ID of the PDSP to download this firmware to */
	u32                id;
	/*
	 * Pointer to the firmware image, If the firmware pointer is NULL, do not
	 * download the firmware.
	 */ 
	void              *firmware;
	/* Size of firmware in bytes */
	u32                size;
};

/* QM setup configuration */
struct qm_config  {
	u32                link_ram_base;
	u32                link_ram_size;
	u32                mem_region_base;
	u32                mem_regnum_descriptors;
	/* Where the initialized descriptors are placed */
	u32                dest_q;
	/* PDSP firmware to load */
	struct pdsp_config pdsp_firmware[QMSS_MAX_PDSP];
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
 * Memory location where QM descriptors and accumulator lists reside
 */
#define DEVICE_QM_DESC_RAM_BASE_PHYS    RAM_MSM_BASE     /* Physical addr as viewed by PDSP */
#define DEVICE_QM_DESC_RAM_BASE         RAM_MSM_CO_BASE  /* Virtual addr as viewed by cores (must be coherent)*/

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

/* Prototypes */
struct qm_host_desc *hw_qm_queue_pop(u32 qnum);
void                 hw_qm_queue_push(struct qm_host_desc *hd, u32 qnum, u32 desc_size);
int                  hw_qm_setup(struct qm_config *cfg);
u32                  hw_qm_queue_count(u32 qnum);
void                 hw_qm_teardown(void);
int                  hw_qm_init_threshold(u32 qnum);
int                  hw_qm_download_firmware(u32 pdsp_id, void *image, u32 size);
u32                  hw_qm_program_accumulator(u32 pdsp_id, struct qm_acc_cmd_config *cfg);

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

static inline u32 qm_desc_ptov(u32 p) {
	return (p == 0) ?
		0 : p + (DEVICE_QM_DESC_RAM_BASE - DEVICE_QM_DESC_RAM_BASE_PHYS);
}

static inline u32 qm_desc_vtop(u32 v) {
	return (v == 0) ?
		0 : v - (DEVICE_QM_DESC_RAM_BASE - DEVICE_QM_DESC_RAM_BASE_PHYS);
}

#endif /* __MACH_C6X_KEYSTONE_QMSS_H */

