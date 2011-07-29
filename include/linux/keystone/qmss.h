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

#ifndef KEYSTONE_QMSS_H
#define KEYSTONE_QMSS_H

#define QM_OK				0
#define QM_INVALID_LINKRAM_ALIGNMENT	-1
#define QM_INVALID_MEMREGION_ALIGNMENT	-2
#define QM_INVALID_LINKRAM_SIZE		-3
#define QM_INVALID_LINKRAM_RAM_SIZE	-4     

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
	struct pdsp_config pdsp_firmware[QM_MAX_PDSP];
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

/*
 * Functions
 */
static inline struct qm_host_desc *hw_qm_queue_pop(u32 qnum)
{
	u32 uhd;

	/* Strip the descriptor size info */
	uhd = __raw_readl(DEVICE_QM_MANAGER_QUEUES_BASE + QM_REG_QUEUE_REGD(qnum));
	uhd = uhd & ~0xf;

	return (struct qm_host_desc *) qm_desc_ptov(uhd);
}

static inline void hw_qm_queue_push (struct qm_host_desc *hd, u32 qnum, u32 desc_size)
{
	u32 regd;

	regd = (qm_desc_vtop((u32) hd)) | ((desc_size >> 4) - 1);
	
	/* Push the descriptor onto the queue */
	__raw_writel(regd, (DEVICE_QM_MANAGER_QUEUES_BASE +
			    QM_REG_QUEUE_REGD(qnum)));
}

static inline u32 hw_qm_queue_count(u32 qnum)
{
	u32 rega;

	rega = __raw_readl(DEVICE_QM_QUEUE_STATUS_BASE +
			   QM_REG_QUEUE_REGA(qnum));
	rega = READ_BITFIELD (rega, QM_QA_ENTRY_COUNT_MSB,
			      QM_QA_ENTRY_COUNT_LSB);
	
	return rega;
}

static inline int hw_qm_init_threshold(u32 qnum)
{
	__raw_writel(0x81, (DEVICE_QM_QUEUE_STATUS_BASE +
			    QM_REG_STAT_CFG_REGD(qnum)));
	
	return 0;
}	

/* 
 * Prototypes
 */
int                  hw_qm_alloc_queue(u32 num);
void                 hw_qm_free_queue(u32 queue);
int                  hw_qm_setup(struct qm_config *cfg);
void                 hw_qm_teardown(void);
int                  hw_qm_download_firmware(u32 pdsp_id, void *image, u32 size);
u32                  hw_qm_program_accumulator(u32 pdsp_id, struct qm_acc_cmd_config *cfg);

#endif /* KEYSTONE_QMSS_H */
