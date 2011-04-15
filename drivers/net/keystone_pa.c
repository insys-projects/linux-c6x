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
#include <linux/slab.h>

#include <mach/pa.h>
#include <mach/netcp.h>
#include <mach/keystone_qmss.h>

#define PA_CMD_SIZE 16

int keystone_pa_enable(struct pa_config *cfg)
{
	u32 i;
	u32 v;
	int done = 0;

	/* Disable all PDSPs */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++)
		__raw_writel(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
			     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(i)));
	
	/* Clear the mailbox registers for PDSP 0 */
	for (i = 0; i < PA_NUM_MAILBOX_SLOTS; i++)
		__raw_writel(0, (DEVICE_PA_BASE + PA_REG_MAILBOX_SLOT(0, i)));

	/* Give a few cycles for the disable */
	udelay(1000);

	/* Download the firmware */
	memcpy((unsigned int *)(DEVICE_PA_BASE + PA_MEM_PDSP_IRAM(0)),
	       pdsp_code, sizeof(pdsp_code));

	/* Reset the PC and enable PDSP0 */
	__raw_writel(PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(0),
		     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(0)));
	
	/* 
	 * Copy the two destination mac addresses to the mail box slots.
	 * Mailbox 4 must be written last since this write triggers the
	 * firmware to update the match information
	 */
	cfg->cmd_buf[0] = READ_BITFIELD(cfg->mac0_ms, 31, 24);
	cfg->cmd_buf[1] = READ_BITFIELD(cfg->mac0_ms, 23, 16);
	cfg->cmd_buf[2] = READ_BITFIELD(cfg->mac0_ms, 15,  8);
	cfg->cmd_buf[3] = READ_BITFIELD(cfg->mac0_ms, 7, 0);
	cfg->cmd_buf[4] = READ_BITFIELD(cfg->mac0_ls, 31, 24);
	cfg->cmd_buf[5] = READ_BITFIELD(cfg->mac0_ls, 23, 16);
	cfg->cmd_buf[6] = 0;
	cfg->cmd_buf[7] = 0;

	cfg->cmd_buf[8]  = READ_BITFIELD(cfg->mac1_ms, 31, 24);
	cfg->cmd_buf[9]  = READ_BITFIELD(cfg->mac1_ms, 23, 16);
	cfg->cmd_buf[10] = READ_BITFIELD(cfg->mac1_ms, 15,  8);
	cfg->cmd_buf[11] = READ_BITFIELD(cfg->mac1_ms, 7, 0);
	cfg->cmd_buf[12] = READ_BITFIELD(cfg->mac1_ls, 31, 24);
	cfg->cmd_buf[13] = READ_BITFIELD(cfg->mac1_ls, 23, 16);
	cfg->cmd_buf[14] = READ_BITFIELD(cfg->rx_qnum, 15, 8);
	cfg->cmd_buf[15] = READ_BITFIELD(cfg->rx_qnum, 7, 0);

	/*
	 * Give some delay then verify that the
	 * mailboxes have been cleared
	 */
	for (i = 0, done = 0;
	     ((i < DEVICE_PA_RUN_CHECK_COUNT) && (done == 0)); i++)  {
		udelay(1000);
		v = __raw_readl(DEVICE_PA_BASE + PA_REG_MAILBOX_SLOT(0, 3));
		if (v == 0)
			done = 1;
	}
	
	if (done == 0)
		return -1;
	
	return 0;
}
    
int keystone_pa_disable(void)
{
	u32 i, j;

	/* Disable all pdsps, clear all mailboxes */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++)  {
		__raw_writel(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
			     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(i)));

		for (j = 0; j < PA_NUM_MAILBOX_SLOTS; j++)
			__raw_writel(0, (DEVICE_PA_BASE + PA_REG_MAILBOX_SLOT(i, j)));
	}

	return 0;
}

int keystone_pa_config(u8* mac_addr)
{
	struct pa_config     pa_cfg;
	struct qm_host_desc *hd;
	int                  ret = 0;

	/*
	 * Filter everything except the desired mac address
	 * and the broadcast mac
	 */
	pa_cfg.mac0_ms = ((u32)mac_addr[0] << 24) |
			 ((u32)mac_addr[1] << 16) |
			 ((u32)mac_addr[2] << 8)  |
			 (u32)(mac_addr[3]);
	pa_cfg.mac0_ls = ((u32)mac_addr[4] << 24) |
			 ((u32)mac_addr[5] << 16);

	pa_cfg.mac1_ms = 0xffffffff;
	pa_cfg.mac1_ls = 0xffff0000;

	pa_cfg.rx_qnum = DEVICE_QM_ETH_RX_Q;

	/*
	 * Form the configuration command in a buffer
	 * linked to a descriptor
	 */
	hd = hw_qm_queue_pop(DEVICE_QM_ETH_FREE_Q);
	if (hd == NULL)
		return -ENOMEM;

	pa_cfg.cmd_buf = (u8*) kzalloc(max(PA_CMD_SIZE, L2_CACHE_BYTES), GFP_KERNEL);
	if (pa_cfg.cmd_buf == NULL)
		return -ENOMEM;

	ret = keystone_pa_enable(&pa_cfg);
	if (ret != 0) {
		kfree(pa_cfg.cmd_buf);
		return ret;
	}

	/* No coherency is assumed between PKTDMA and GEM L1/L2 caches */
	L2_cache_block_writeback((u32) pa_cfg.cmd_buf,
				 (u32) pa_cfg.cmd_buf + PA_CMD_SIZE);

	/* Send the command to the PA through the QM */
	hd->software_info0 = PA_MAGIC_ID;
	hd->buff_len       = PA_CMD_SIZE;
	hd->orig_buff_len  = PA_CMD_SIZE;
	hd->orig_buff_ptr  = (u32) pa_cfg.cmd_buf;
	hd->buff_ptr       = (u32) pa_cfg.cmd_buf;
	hd->private        = 0;
	QM_DESC_DESCINFO_SET_PKT_LEN(hd->desc_info, PA_CMD_SIZE);

	/* Set the return Queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_ETH_FREE_Q);

	hw_qm_queue_push(hd, DEVICE_QM_PA_CFG_Q, QM_DESC_SIZE_BYTES);
	
	kfree(pa_cfg.cmd_buf);

	return 0;
}

