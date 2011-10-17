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

#include <mach/keystone_qmss.h>
#include <mach/keystone_pa.h>
#include <mach/keystone_netcp.h>
#include <linux/keystone/pa.h>
#include <linux/keystone/qmss.h>

int keystone_pa_reset(void)
{
	u32 i;

	/* Reset and disable all PDSPs */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		__raw_writel(PA_REG_VAL_PDSP_CTL_RESET_PDSP,
			     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(i)));

		while((__raw_readl(DEVICE_PA_BASE + PA_REG_PDSP_CTL(i))
		       & PA_REG_VAL_PDSP_CTL_STATE));
	}

	/* Reset packet Id */
	__raw_writel(1, DEVICE_PA_BASE + PA_REG_PKTID_SOFT_RESET);

	/* Reset LUT2 */
	__raw_writel(1, DEVICE_PA_BASE + PA_REG_LUT2_SOFT_RESET);

	/* Reset statistic */
	__raw_writel(1, DEVICE_PA_BASE + PA_REG_STATS_SOFT_RESET);

	/* Reset timers */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		__raw_writel(0, (DEVICE_PA_BASE + PA_REG_TIMER_CTL(i)));
	}

	return 0;
}

int keystone_pa_enable(int pdsp)
{
	u32 i;
	int v;

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	/* Check the PDSP state */
	v = __raw_readl(DEVICE_PA_BASE + PA_REG_PDSP_CTL(pdsp));
	if (v & PA_REG_VAL_PDSP_CTL_STATE) {
		/* Already enabled */
		return 1;
	}

	/* Clear the mailboxes */
	for (i = 0; i < PA_NUM_MAILBOX_SLOTS; i++) {
		__raw_writel(0, (DEVICE_PA_BASE + PA_REG_MAILBOX_SLOT(pdsp, i)));
	}

	/* Enable PDSP */
	__raw_writel(PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(0),
		     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(pdsp)));

	return 0;
}
    
int keystone_pa_disable(int pdsp)
{
	u32 i, j;

	if (pdsp >= DEVICE_PA_NUM_PDSPS)
		return -EINVAL;

	if (pdsp < 0) {
		/* Disable all pdsps, clear all mailboxes */
		for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++)  {
			__raw_writel(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
				     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(i)));

			for (j = 0; j < PA_NUM_MAILBOX_SLOTS; j++)
				__raw_writel(0, (DEVICE_PA_BASE + PA_REG_MAILBOX_SLOT(i, j)));
		}
	} else {
		__raw_writel(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
			     (DEVICE_PA_BASE + PA_REG_PDSP_CTL(pdsp)));

		for (j = 0; j < PA_NUM_MAILBOX_SLOTS; j++)
			__raw_writel(0, (DEVICE_PA_BASE + PA_REG_MAILBOX_SLOT(pdsp, j)));

	}
	return 0;
}

/*
 * download/upload firmware
 */
int keystone_pa_get_firmware(int pdsp, unsigned int *buffer, int len)
{
	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_get(buffer, (u32 *)(DEVICE_PA_BASE + PA_MEM_PDSP_IRAM(pdsp)),
		    len >> 2);

	return 0;
}

int keystone_pa_set_firmware(int pdsp, const unsigned int *buffer, int len)
{
	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_put((u32 *)(DEVICE_PA_BASE + PA_MEM_PDSP_IRAM(pdsp)), buffer,
		    len >> 2);

	return 0;
}

int keystone_pa_config(int pdsp, const unsigned int *pdsp_code, int len, u8* mac_addr)
{
	struct pa_config     pa_cfg;
	struct qm_host_desc *hd;
	int                  ret = 0;

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	if ((len > PAGE_SIZE) || (len < 0))
		return -EINVAL;

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
	if (hd == NULL) {
		printk(KERN_DEBUG "PA: no more free desc hd = 0x%x\n", (u32) hd);
		return -ENOMEM;
	}

	/* Download the firmware in PDSP0 */
	keystone_pa_set_firmware(pdsp, pdsp_code, len);

	/* Enable PDSP0 */
	ret = keystone_pa_enable(pdsp);
	if (ret != 0) {
		printk(KERN_DEBUG "PA: enabling failed ret = %d\n", ret);
		return ret;
	}

	pa_cfg.cmd_buf = (u8*) kzalloc(max(PA_CMD_SIZE, L2_CACHE_BYTES), GFP_KERNEL);
	if (pa_cfg.cmd_buf == NULL)
		return -ENOMEM;

	/* 
	 * Give the two destination mac addresses to the firmware 
	 * to update the match information
	 */
	pa_cfg.cmd_buf[0] = READ_BITFIELD(pa_cfg.mac0_ms, 31, 24);
	pa_cfg.cmd_buf[1] = READ_BITFIELD(pa_cfg.mac0_ms, 23, 16);
	pa_cfg.cmd_buf[2] = READ_BITFIELD(pa_cfg.mac0_ms, 15,  8);
	pa_cfg.cmd_buf[3] = READ_BITFIELD(pa_cfg.mac0_ms, 7, 0);
	pa_cfg.cmd_buf[4] = READ_BITFIELD(pa_cfg.mac0_ls, 31, 24);
	pa_cfg.cmd_buf[5] = READ_BITFIELD(pa_cfg.mac0_ls, 23, 16);
	pa_cfg.cmd_buf[6] = 0;
	pa_cfg.cmd_buf[7] = 0;

	pa_cfg.cmd_buf[8]  = READ_BITFIELD(pa_cfg.mac1_ms, 31, 24);
	pa_cfg.cmd_buf[9]  = READ_BITFIELD(pa_cfg.mac1_ms, 23, 16);
	pa_cfg.cmd_buf[10] = READ_BITFIELD(pa_cfg.mac1_ms, 15,  8);
	pa_cfg.cmd_buf[11] = READ_BITFIELD(pa_cfg.mac1_ms, 7, 0);
	pa_cfg.cmd_buf[12] = READ_BITFIELD(pa_cfg.mac1_ls, 31, 24);
	pa_cfg.cmd_buf[13] = READ_BITFIELD(pa_cfg.mac1_ls, 23, 16);
	pa_cfg.cmd_buf[14] = READ_BITFIELD(pa_cfg.rx_qnum, 15, 8);
	pa_cfg.cmd_buf[15] = READ_BITFIELD(pa_cfg.rx_qnum, 7, 0);

	L2_cache_block_writeback((u32) pa_cfg.cmd_buf,
				 (u32) pa_cfg.cmd_buf + PA_CMD_SIZE);

	/* Send the command to the PA through the QM */
	hd->software_info0 = PA_MAGIC_ID;
	hd->buff_len       = PA_CMD_SIZE;
	hd->orig_buff_len  = PA_CMD_SIZE;
	hd->orig_buff_ptr  = (u32) pa_cfg.cmd_buf;
	hd->buff_ptr       = (u32) pa_cfg.cmd_buf;
	hd->private        = 0;
	QM_DESC_DINFO_SET_PKT_LEN(hd->desc_info, PA_CMD_SIZE);

	/* Set the return Queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_ETH_FREE_Q);

	hw_qm_queue_push(hd, DEVICE_QM_PA_CFG_Q, DEVICE_QM_DESC_SIZE_BYTES);

	kfree(pa_cfg.cmd_buf);
		
	return 0;
}
