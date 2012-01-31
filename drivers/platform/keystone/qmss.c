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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/bitmap.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>

#include <asm/system.h>
#include <asm/bitops.h>

#include <mach/keystone_qmss.h>
#include <linux/keystone/qmss.h>

#define QMSS_DRIVER_NAME    "TI KeyStone QMSS driver"
#define QMSS_DRIVER_VERSION "v1.1"

static DEFINE_MUTEX(qmss_mutex);

#define QUEUE_BITMAP_SIZE (DEVICE_QM_VUSR_ALLOC_END_Q - DEVICE_QM_VUSR_ALLOC_Q + 31)

static unsigned long queue_bitmap[(QUEUE_BITMAP_SIZE) >> 5];

static void __iomem *qm_base;

static inline void qm_write_reg(u32 val, int reg)
{
	__raw_writel(val, qm_base + reg);
}

static inline u32 qm_read_reg(int reg)
{
	return __raw_readl(qm_base + reg);
}

/*
 * Acknowledge accumulator interrupts for a given prioriy index and a given channel 
 */
void hw_qm_ack_interrupt(u32 index, u32 channel)
{
	if (qm_read_reg(DEVICE_QM_INTD_OFFSET + QM_REG_INTD_STATUS0) &
		    (1 << channel)) {
			qm_write_reg(1, QM_REG_INTD_COUNT_IRQ(channel));
			qm_write_reg(index + channel,
				     DEVICE_QM_INTD_OFFSET + QM_REG_INTD_EOI);
	}
}

int hw_qm_interrupt_status(u32 channel)
{
	if ((qm_read_reg(DEVICE_QM_INTD_OFFSET + QM_REG_INTD_STATUS0) &
	     (1 << channel)) == 0)
		return 0;
	else
		return 1;
}

/*
 * Queue number allocator
 */
int hw_qm_alloc_queue(u32 num)
{
	static int    initialized = 0;
	unsigned long start;

	mutex_lock(&qmss_mutex);

	if (!initialized) {
		bitmap_clear(queue_bitmap, 0, QUEUE_BITMAP_SIZE);
		initialized = 1;
	}

	start = bitmap_find_next_zero_area(queue_bitmap, QUEUE_BITMAP_SIZE, 0, num, 0);
	if (start > DEVICE_QM_VUSR_ALLOC_END_Q)
		return -ENOSPC;

	bitmap_set(queue_bitmap, start, num);

	mutex_unlock(&qmss_mutex);
	
	return start + DEVICE_QM_VUSR_ALLOC_Q;
}

void hw_qm_free_queue(u32 queue)
{
	int start = queue - DEVICE_QM_VUSR_ALLOC_Q;

	if ((start < 0) || (start > DEVICE_QM_VUSR_ALLOC_END_Q))
		return;

	mutex_lock(&qmss_mutex);
	bitmap_clear(queue_bitmap, start, 1);
	mutex_unlock(&qmss_mutex);
}

struct qm_host_desc *hw_qm_queue_pop(u32 qnum)
{
	u32 uhd;

	/* Strip the descriptor size info */
	uhd = qm_read_reg(DEVICE_QM_MANAGER_QUEUES_OFFSET + QM_REG_QUEUE_REGD(qnum));
	uhd = uhd & ~0xf;

	return (struct qm_host_desc *) qm_desc_ptov(uhd);
}

void hw_qm_queue_push (struct qm_host_desc *hd, u32 qnum, u32 desc_size)
{
	u32 regd;

	regd = (qm_desc_vtop((u32) hd)) | ((desc_size >> 4) - 1);
	
	/* Push the descriptor onto the queue */
	qm_write_reg(regd, (DEVICE_QM_MANAGER_QUEUES_OFFSET +
			    QM_REG_QUEUE_REGD(qnum)));
}

u32 hw_qm_queue_count(u32 qnum)
{
	u32 rega;

	rega = qm_read_reg(DEVICE_QM_QUEUE_STATUS_OFFSET +
			   QM_REG_QUEUE_REGA(qnum));
	rega = READ_BITFIELD (rega, QM_QA_ENTRY_COUNT_MSB,
			      QM_QA_ENTRY_COUNT_LSB);
	
	return rega;
}

int hw_qm_init_threshold(u32 qnum)
{
	qm_write_reg(0x81, (DEVICE_QM_QUEUE_STATUS_OFFSET +
			    QM_REG_STAT_CFG_REGD(qnum)));
	
	return 0;
}	

/*
 * This function programs the accumulator with values passed in the cfg structure
 */
u32 hw_qm_program_accumulator(u32 pdsp_id, struct qm_acc_cmd_config *cfg)
{
	volatile u32   *cmd;
	volatile u32   *p_cmd;
	volatile u32   *reg;
	u32             index;
	u8              result;

	if (cfg == NULL)
		return -EINVAL;

	/* Use kmalloc here to be sure that buffer is aligned on cache line */
	cmd = (volatile u32 *) kzalloc(QM_ACC_CMD_SIZE, GFP_KERNEL);
	if (cmd == NULL)
		return -1;

	cmd[0] = SET_BITFIELD(cmd[0], cfg->channel, 7, 0);
	cmd[0] = SET_BITFIELD(cmd[0], cfg->command, 15, 8);
	cmd[1] = cfg->queue_mask;
	cmd[2] = cfg->list_addr;
	cmd[3] = SET_BITFIELD(cmd[3], cfg->queue_index, 15, 0);
	cmd[3] = SET_BITFIELD(cmd[3], cfg->max_entries, 31, 16);
	cmd[4] = SET_BITFIELD(cmd[4], cfg->timer_count, 15, 0);
	cmd[4] = SET_BITFIELD(cmd[4], cfg->pacing_mode, 17, 16);
	cmd[4] = SET_BITFIELD(cmd[4], cfg->list_entry_size, 19, 18);
	cmd[4] = SET_BITFIELD(cmd[4], cfg->list_count_mode, 20, 20);
	cmd[4] = SET_BITFIELD(cmd[4], cfg->multi_queue_mode, 21, 21);
	
	/* Point to the accumulator command register's last word */
	reg = (uint32_t *) ((uint8_t *)(qm_base +
					DEVICE_QM_PDSP_CMD_OFFSET(pdsp_id)) + 4 * 4);
	
	/* Write command word last */
	p_cmd = cmd + 4;
	
	mutex_lock(&qmss_mutex);

	for (index = 0; index < QM_ACC_CMD_SIZE; index += 4)
		*reg-- = *p_cmd--;
	
	/* Wait for the command to clear */
	reg++;
	do {
		result = READ_BITFIELD(*reg, 15, 8); 
	} while (result != 0);

	mutex_unlock(&qmss_mutex);

	kfree((void *)cmd);
	
	result = (READ_BITFIELD(*reg, 31, 24));

	return !result;
}

/*
 * Downloads the PDSP firmware to QM PDSP.
 */
int hw_qm_download_firmware (u32 pdsp_id, void *image, u32 size)
{
	u32 i;

	if ((image == NULL) || (size == 0))
		return -EINVAL;

	mutex_lock(&qmss_mutex);

	/* Reset the PDSP */
	qm_write_reg(QM_REG_VAL_PDSP_CTL_DISABLE, DEVICE_QM_PDSP_CTRL_OFFSET(pdsp_id));
	
	/* Confirm PDSP has halted */
	while(qm_read_reg(DEVICE_QM_PDSP_CTRL_OFFSET(pdsp_id))
	      & QM_REG_VAL_PDSP_CTL_STATE);
	
	/* upload the firmware */
	pdsp_fw_put((u32 *)(qm_base + DEVICE_QM_PDSP_IRAM_OFFSET(pdsp_id)), (u32 *) image, size >> 2);
	
	/* Use the command register to sync the PDSP */
	qm_write_reg(0xFFFFFFFF, DEVICE_QM_PDSP_CMD_OFFSET(pdsp_id));
	
	/* Wait to the memory write to land */
	for (i = 0;
	     (i < 20000) && (qm_read_reg(DEVICE_QM_PDSP_CMD_OFFSET(pdsp_id)) != 0xFFFFFFFF);
	     i++);
	
	/* Reset the PC and enable PDSP */
	qm_write_reg(QM_REG_VAL_PDSP_CTL_ENABLE(0), DEVICE_QM_PDSP_CTRL_OFFSET(pdsp_id));

	/* Wait for the command register to clear */
	while (qm_read_reg(DEVICE_QM_PDSP_CMD_OFFSET(pdsp_id)));
	
	mutex_unlock(&qmss_mutex);
    
	return 0;
}

static int hw_qm_setup (struct qm_config *cfg)
{
	u32 v, w, x, i;
	    
	struct qm_host_desc *hd;

	/* Reset the QM PDSP */
	for (i = 0; i < QM_MAX_PDSP; i++)
		qm_write_reg(QM_REG_VAL_PDSP_CTL_DISABLE,
			     DEVICE_QM_PDSP_CTRL_OFFSET(cfg->pdsp_firmware[i].id));

	/* Verify that alignment requirements */
	if ((cfg->link_ram_base & (QM_LINKRAM_ALIGN - 1)) != 0)
		return QM_INVALID_LINKRAM_ALIGNMENT;

	if ((cfg->mem_region_base & (QM_MEMR_ALIGN-1)) != 0)
		return QM_INVALID_MEMREGION_ALIGNMENT;

	/* Verify linkram sizing is in range */
	if ((cfg->link_ram_size & ~QM_REG_LINKRAM_SIZE_MAX_MASK) != 0)
		return QM_INVALID_LINKRAM_SIZE;

	/*
	 * Verify there is enough linkram to cover
	 * the single memory region
	 */ 
	if (cfg->link_ram_size < cfg->mem_regnum_descriptors)
		return QM_INVALID_LINKRAM_RAM_SIZE;

	mutex_lock(&qmss_mutex);

	/* Linking RAM info */
	qm_write_reg(cfg->link_ram_base, (DEVICE_QM_MANAGER_OFFSET +
					  QM_REG_LINKRAM_BASE(0)));
	qm_write_reg(cfg->link_ram_size, (DEVICE_QM_MANAGER_OFFSET +
					  QM_REG_LINKRAM_SIZE(0)));
	qm_write_reg(0, (DEVICE_QM_MANAGER_OFFSET + QM_REG_LINKRAM_BASE(1)));

	
	/* Memory region 0 info */
	qm_write_reg(cfg->mem_region_base, (DEVICE_QM_DESC_SETUP_OFFSET +
					    QM_REG_MEMR_BASE_ADDR(0)));
	qm_write_reg(0, (DEVICE_QM_DESC_SETUP_OFFSET +
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
	QM_REG_VAL_DESC_SETUP_SET_DESC_SIZE(v, DEVICE_QM_DESC_SIZE_BYTES);
	qm_write_reg(v, (DEVICE_QM_DESC_SETUP_OFFSET +
			 QM_REG_MEMR_DESC_SETUP(0))); 
	
	/* Now format the descriptors and put them in a queue */
	for (i = 0, v = cfg->mem_region_base;
	     i < cfg->mem_regnum_descriptors;
	     i++, v += DEVICE_QM_DESC_SIZE_BYTES) {
		
		hd = (struct qm_host_desc *) qm_desc_ptov(v);
		memset (hd, 0, sizeof(struct qm_host_desc));

		hd->desc_info   = QM_DESC_DINFO_DEFAULT;
		hd->packet_info = QM_DESC_PINFO_DEFAULT;
		
		if (QM_DESC_DINFO_GET_PSINFO_LOC(hd->desc_info) ==
		    QM_DESC_DINFO_PSINFO_IN_DESCR) {
			if (QM_DESC_PINFO_GET_EPIB(hd->packet_info) ==
			    QM_DESC_PINFO_EPIB)
				w = DEVICE_QM_DESC_SIZE_BYTES - 32 - 16;
			else
				w = DEVICE_QM_DESC_SIZE_BYTES - 32;
		} else
			w = 0;
		
		QM_DESC_PINFO_SET_SIZE(hd->packet_info, (w >> 2));
		
		/* Push the descriptor onto the queue */
		x = device_local_addr_to_global(v);
		
		qm_write_reg(x, (DEVICE_QM_MANAGER_QUEUES_OFFSET +
				 QM_REG_QUEUE_REGD(cfg->dest_q)));
        }
	
	mutex_unlock(&qmss_mutex);
    
	/* Download optional firmware to QM PDSP */
	for (i = 0; i < QM_MAX_PDSP; i++) {
		if (cfg->pdsp_firmware[i].firmware != NULL) {
			int ret;

			ret = hw_qm_download_firmware(cfg->pdsp_firmware[i].id,
						      cfg->pdsp_firmware[i].firmware,
						      cfg->pdsp_firmware[i].size);
			if (ret != 0)
				return ret;
		}
	}
	return 0;
} 

static void hw_qm_teardown (void)
{
	u32 i;
	
	/* Linking RAM info */
	for (i = 0; i < DEVICE_QM_NUM_LINKRAMS; i++)  {
		qm_write_reg(0, (DEVICE_QM_MANAGER_OFFSET +
				 QM_REG_LINKRAM_BASE(i)));
		
		qm_write_reg(0, (DEVICE_QM_MANAGER_OFFSET +
				 QM_REG_LINKRAM_SIZE(i)));
	}

	/* Memory region info */
	for (i = 0; i < DEVICE_QM_NUM_MEMREGIONS; i++)  {
		qm_write_reg(0, (DEVICE_QM_DESC_SETUP_OFFSET +
				 QM_REG_MEMR_BASE_ADDR(i)));
		qm_write_reg(0, (DEVICE_QM_DESC_SETUP_OFFSET +
				 QM_REG_MEMR_START_IDX(i)));
		qm_write_reg(0, (DEVICE_QM_DESC_SETUP_OFFSET +
				 QM_REG_MEMR_DESC_SETUP(i)));
	}
}

static int __devinit qmss_probe(struct platform_device *pdev)
{
	struct qmss_platform_data *data = pdev->dev.platform_data;
	struct qm_config c_q_cfg;
	struct qm_config *q_cfg  = &c_q_cfg;
	const struct firmware *fw;
	u32 desc_ram;
	int res;

	qm_base = ioremap(DEVICE_QM_BASE, DEVICE_QM_SIZE);

	/* Only master core can initialize QMSS in a multi-Linux environment */
 	if (data->slave) {
		goto slave_core;
	}

	q_cfg->link_ram_base = data->link_ram_base;
	q_cfg->link_ram_size = data->link_ram_size;
	
	/* Allocate memory for descriptors */
	desc_ram = qm_mem_alloc(data->desc_ram_size,
				(u32*) &q_cfg->mem_region_base);
	if (!desc_ram) {
		printk(KERN_ERR "%s: descriptor memory allocation failed\n", 
		       __FUNCTION__);
		return -ENOMEM;
	}
	    
	q_cfg->mem_regnum_descriptors = data->desc_num;
	q_cfg->dest_q		      = data->free_queue;

	memset((void *) desc_ram, 0, data->desc_ram_size);
	
	/* Request QM accumulator firmware */
	res = request_firmware(&fw, data->qm_pdsp.firmware, &pdev->dev);
	if (res != 0) {
		printk(KERN_ERR "QM: Cannot find %s firmware\n",
		       data->qm_pdsp.firmware);
		return res;
	}

	/* load QM PDSP firmware for accumulators */
	q_cfg->pdsp_firmware[0].id       = data->qm_pdsp.pdsp; 
	q_cfg->pdsp_firmware[0].firmware = (unsigned int *) fw->data;
	q_cfg->pdsp_firmware[0].size     = fw->size;;
	q_cfg->pdsp_firmware[1].firmware = NULL;

	/* Initialize QM */
	hw_qm_setup(q_cfg);
	release_firmware(fw);

slave_core:
	printk("%s %s\n", QMSS_DRIVER_NAME, QMSS_DRIVER_VERSION);

	return 0;
}

static int __devexit qmss_remove(struct platform_device *pdev)
{
	hw_qm_teardown();

	iounmap(qm_base);

	return 0;
}

static struct platform_driver qmss_driver = {
	.driver = {
		.name	 = "keystone_qmss",
		.owner	 = THIS_MODULE,
	},
	.probe = qmss_probe,
	.remove = __devexit_p(qmss_remove),
};

static int __init qmss_init(void)
{
	return platform_driver_register(&qmss_driver);
}
subsys_initcall(qmss_init); /* should be initialized early */

static void __exit qmss_exit(void)
{
	platform_driver_unregister(&qmss_driver);
}
module_exit(qmss_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone QMSS driver");
