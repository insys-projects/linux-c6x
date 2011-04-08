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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>

#include <mach/pa.h>
#include <mach/netcp.h>
#include <mach/keystone_qmss.h>

#include <asm/system.h>
#include <asm/bitops.h>

struct qm_host_desc *hw_qm_queue_pop(u32 qnum)
{
	struct qm_host_desc *hd;
	u32 uhd;

	/* Strip the descriptor size info */
	uhd = __raw_readl(DEVICE_QM_MANAGER_QUEUES_BASE + QM_REG_QUEUE_REGD(qnum));
	uhd = uhd & ~0xf;
	hd  = (struct qm_host_desc *)uhd;

	return (hd);
}

u32 hw_qm_queue_count(u32 qnum)
{
	u32 rega;

	rega = __raw_readl(DEVICE_QM_QUEUE_STATUS_BASE +
			   QM_REG_QUEUE_REGA(qnum));
	rega = READ_BITFIELD (rega, QM_QA_ENTRY_COUNT_MSB,
			      QM_QA_ENTRY_COUNT_LSB);
	
	return (rega);
}

int hw_qm_init_threshold(u32 qnum)
{
	__raw_writel(0x81, (DEVICE_QM_QUEUE_STATUS_BASE +
			    QM_REG_STAT_CFG_REGD(qnum)));
	
	return (0);
}	

void hw_qm_queue_push (struct qm_host_desc *hd, u32 qnum, u32 desc_size)
{
	u32 regd;

	regd = ((u32) hd | ((desc_size >> 4) - 1));

	/* Push the descriptor onto the queue */
	__raw_writel(regd, (DEVICE_QM_MANAGER_QUEUES_BASE +
			    QM_REG_QUEUE_REGD(qnum)));
}

int hw_qm_setup (struct qm_config *cfg)
{
	u32 v, w, x, i;
	    
	struct qm_host_desc *hd;

	/* Verify that alignment requirements */
	if ((cfg->link_ram_base & (QM_LINKRAM_ALIGN - 1)) != 0)
		return (QM_INVALID_LINKRAM_ALIGNMENT);

	if ((cfg->mem_region_base & (QM_MEMR_ALIGN-1)) != 0)
		return (QM_INVALID_MEMREGION_ALIGNMENT);

	/* Verify linkram sizing is in range */
	if ((cfg->link_ram_size & ~QM_REG_LINKRAM_SIZE_MAX_MASK) != 0)
		return (QM_INVALID_LINKRAM_SIZE);

	/*
	 * Verify there is enough linkram to cover
	 * the single memory region
	 */ 
	if (cfg->link_ram_size < cfg->mem_regnum_descriptors)
		return (QM_INVALID_LINKRAM_RAM_SIZE);

	/* Linking RAM info */
	__raw_writel(cfg->link_ram_base, (DEVICE_QM_MANAGER_BASE +
					  QM_REG_LINKRAM_BASE(0)));
	__raw_writel(cfg->link_ram_size, (DEVICE_QM_MANAGER_BASE +
					  QM_REG_LINKRAM_SIZE(0)));
	__raw_writel(0, (DEVICE_QM_MANAGER_BASE + QM_REG_LINKRAM_BASE(1)));

	
	/* Memory region 0 info */
	__raw_writel(cfg->mem_region_base, (DEVICE_QM_DESC_SETUP_BASE +
					    QM_REG_MEMR_BASE_ADDR(0)));
	__raw_writel(0, (DEVICE_QM_DESC_SETUP_BASE +
			 QM_REG_MEMR_START_IDX(0)));

	/*
	 * Calculate the 2 fields in the descriptor setup register.
	 * Bits 0-3 specifiy the total memory size rounded up to the
	 * next higher power of two, and is expresses as 2^(n - 5).
	 * So for example if you have 20 descriptors, the next higher
	 * power of 2 that exceeds this is 32, which is 2^5,
	 * so the value 0 (5-5) is placed in this field 
	 */
	v = fls(cfg->mem_regnum_descriptors) - 1;
	if (v >= 4)
		v = v - 4;
	else
		v = 0;

	/* Add the descriptor size field */
	QM_REG_VAL_DESC_SETUP_SET_DESC_SIZE(v, QM_DESC_SIZE_BYTES);
	__raw_writel(v, (DEVICE_QM_DESC_SETUP_BASE +
			 QM_REG_MEMR_DESC_SETUP(0))); 

	/* Now format the descriptors and put them in a queue */
	for (i = 0, v = cfg->mem_region_base;
		i < cfg->mem_regnum_descriptors;
		i++, v += QM_DESC_SIZE_BYTES) {
		
			hd = (struct qm_host_desc *)v;
			memset (hd, 0, sizeof(struct qm_host_desc));

			hd->desc_info = QM_DESC_DEFAULT_DESCINFO;
			hd->packet_info = QM_DESC_DEFAULT_PINFO;
        
			if (QM_DESC_INFO_GET_PSINFO_LOC(hd->desc_info) ==
					QM_DESC_PSINFO_IN_DESCR) {
				if (QM_PKT_INFO_GET_EPIB(hd->packet_info) ==
					QM_DESC_PINFO_EPIB)
					w = QM_DESC_SIZE_BYTES - 32 - 16;
				else
					w = QM_DESC_SIZE_BYTES - 32;
			} else
				w = 0;
        	
	QM_PKT_INFO_SET_PSINFO_SIZE(hd->packet_info, (w >> 2));
 
        /* Push the descriptor onto the queue */
        x = device_local_addr_to_global(v);

	__raw_writel(x, (DEVICE_QM_MANAGER_QUEUES_BASE +
			 QM_REG_QUEUE_REGD(cfg->dest_q)));
        }

	return (0);
} 

void hw_qm_teardown (void)
{
	u32 i;

	/* Linking RAM info */
	for (i = 0; i < DEVICE_QM_NUM_LINKRAMS; i++)  {
		__raw_writel(0, (DEVICE_QM_MANAGER_BASE +
				 QM_REG_LINKRAM_BASE(i)));

		__raw_writel(0, (DEVICE_QM_MANAGER_BASE +
				 QM_REG_LINKRAM_SIZE(i)));
	}

	/* Memory region info */
	for (i = 0; i < DEVICE_QM_NUM_MEMREGIONS; i++)  {
		__raw_writel(0, (DEVICE_QM_DESC_SETUP_BASE +
				 QM_REG_MEMR_BASE_ADDR(i)));
		__raw_writel(0, (DEVICE_QM_DESC_SETUP_BASE +
				 QM_REG_MEMR_START_IDX(i)));
		__raw_writel(0, (DEVICE_QM_DESC_SETUP_BASE +
				 QM_REG_MEMR_DESC_SETUP(i)));
	}
}
