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
 
unsigned int pdsp_code[] =  {
	0x2eff9196,
	0x85002096,
	0x0101f6f6,
	0x81002496,
	0xcf04fffe,
	0x2e808f86,
	0x240cecc2,
	0x2411e082,
	0x68e2ec05,
	0x59108926,
	0x24002104,
	0x2f000384,
	0x21000200,
	0x0101f7f7,
	0x81042497,
	0x24000c04,
	0x2f000384,
	0x2e808f86,
	0x24000004,
	0x240020c4,
	0x2f000384,
	0x2e808f8e,
	0x68e6fb04,
	0x68c7dc03,
	0x0101f8f8,
	0x21002400,
	0x68e6fd04,
	0x68c7de03,
	0x0101f9f9,
	0x21002400,
	0x0101fafa,
	0x810c249a,
	0x24002104,
	0x2f000384,
	0x8700e286,
	0x21000200,
	0x00f9f8e1,
	0x81082481,
	0x24002004,
	0x24000644,
	0x24000064,
	0x109e9ec5,
	0x2400b024,
	0x24000005,
	0x2f000384,
	0x8700e186,
	0x21000200,
	0x24000c04,
	0x2f000384,
	0x2e80878e,
	0x10eeeefb,
	0x10efeffc,
	0x10f0f0fd,
	0x10f1f1fe,
	0x24002104,
	0x2f000384,
	0x21000200
};

int hw_pa_enable(struct pa_config *cfg)
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

	/* download the firmware */
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
	cfg->cmd_buf[0] = BOOT_READ_BITFIELD(cfg->mac0_ms, 31, 24);
	cfg->cmd_buf[1] = BOOT_READ_BITFIELD(cfg->mac0_ms, 23, 16);
	cfg->cmd_buf[2] = BOOT_READ_BITFIELD(cfg->mac0_ms, 15,  8);
	cfg->cmd_buf[3] = BOOT_READ_BITFIELD(cfg->mac0_ms, 7, 0);
	cfg->cmd_buf[4] = BOOT_READ_BITFIELD(cfg->mac0_ls, 31, 24);
	cfg->cmd_buf[5] = BOOT_READ_BITFIELD(cfg->mac0_ls, 23, 16);
	cfg->cmd_buf[6] = 0;
	cfg->cmd_buf[7] = 0;

	cfg->cmd_buf[8]  = BOOT_READ_BITFIELD(cfg->mac1_ms, 31, 24);
	cfg->cmd_buf[9]  = BOOT_READ_BITFIELD(cfg->mac1_ms, 23, 16);
	cfg->cmd_buf[10] = BOOT_READ_BITFIELD(cfg->mac1_ms, 15,  8);
	cfg->cmd_buf[11] = BOOT_READ_BITFIELD(cfg->mac1_ms, 7, 0);
	cfg->cmd_buf[12] = BOOT_READ_BITFIELD(cfg->mac1_ls, 31, 24);
	cfg->cmd_buf[13] = BOOT_READ_BITFIELD(cfg->mac1_ls, 23, 16);
	cfg->cmd_buf[14] = BOOT_READ_BITFIELD(cfg->rx_qnum, 15, 8);
	cfg->cmd_buf[15] = BOOT_READ_BITFIELD(cfg->rx_qnum, 7, 0);

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
		return (-1);
	
	return (0);
}
    
int hw_pa_disable(void)
{
	u32 i, j;

	/* Disable all pdsps, clear all mailboxes */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++)  {
		__raw_writel(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
				(DEVICE_PA_BASE + PA_REG_PDSP_CTL(i)));

		for (j = 0; j < PA_NUM_MAILBOX_SLOTS; j++)
			__raw_writel(0, (DEVICE_PA_BASE +
				PA_REG_MAILBOX_SLOT(i, j)));

	}

	return (0);
}

int target_pa_config(void)
{
	struct pa_config	pa_cfg;
	struct qm_host_desc	*hd;
	u32 mac_a, mac_b;
	int i = 0;
	u8 mac_addr[6];
	
	/* Read the e-fuse mac address */
	mac_a = __raw_readl(0x2620110);
	mac_b = __raw_readl(0x2620114);
	mac_addr[0] = (mac_b >>  8) & 0xff;
	mac_addr[1] = (mac_b >>  0) & 0xff;
	mac_addr[2] = (mac_a >> 24) & 0xff;
	mac_addr[3] = (mac_a >> 16) & 0xff;
	mac_addr[4] = (mac_a >>  8) & 0xff;
	mac_addr[5] = (mac_a >>  0) & 0xff;

	/*
	 * Filter everything except the desired mac address
	 * and the broadcast mac
	 */
	pa_cfg.mac0_ms = ((u32)mac_addr[0] << 24) |
			 ((u32)mac_addr[1] << 16) |
			 ((u32)mac_addr[2] << 8) |
			 (u32)(mac_addr[3]);
	pa_cfg.mac0_ls = ((u32)mac_addr[4] << 24) |
			 ((u32)mac_addr[5] << 16);

	pa_cfg.mac1_ms = 0xffffffff;
	pa_cfg.mac1_ls = 0xffff0000;

	pa_cfg.rx_qnum = DEVICE_QM_RCV_Q;

	/*
	 * Form the configuration command in a buffer
	 * linked to a descriptor
	 */
	hd = hw_qm_queue_pop(DEVICE_QM_LNK_BUF_Q);
	pa_cfg.cmd_buf = (u8 *)hd->orig_buff_ptr;

	i = hw_pa_enable(&pa_cfg);
	if (i != 0)
		return (i);

	/* Send the command to the PA through the QM */
	hd->software_info0 = PA_MAGIC_ID;
	hd->buff_len = 16;
	
	QM_DESC_DESCINFO_SET_PKT_LEN(hd->desc_info, 16);

	/* Set the return Queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_LNK_BUF_Q);

	hw_qm_queue_push(hd, DEVICE_QM_PA_CFG_Q, QM_DESC_SIZE_BYTES);
	
	return (0);
}

