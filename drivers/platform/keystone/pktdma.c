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
#include <linux/types.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <mach/keystone_qmss.h>
#include <mach/keystone_pktdma.h>
#include <linux/keystone/pktdma.h>
#include <linux/keystone/qmss.h>

/* 
 * Disable all rx channels and clear all the flow registers
 * The teardown is initiated and polled for completion. The function will
 * return an error if the teardown is never complete, but will not stay
 * in the function forever.
 */
int pktdma_rx_disable(struct pktdma_rx_cfg *cfg)
{
	u32 i, v;
	u32 done;

	for (i = 0; i < cfg->n_rx_chans; i++) {
		/* If enabled, set the teardown bit */
		v = __raw_readl(cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(i));
		if ((v & PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE) == PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE ) {
			v = v | PKTDMA_REG_VAL_RCHAN_A_RX_TDOWN;
			__raw_writel(v, cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(i));
		}
	}

	/* Poll for completion */
	for (i = 0, done = 0; ( (i < cfg->tdown_poll_count) && (done == 0) ); i++) {
		udelay(1000);
		done = 1;
		v = __raw_readl(cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(i));
		if ((v & PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE) == PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE)
			done = 0;
	}

	if (done == 0)
		return -1;

	/* Clear all of the flow registers */
	for (i = 0; i < cfg->nrx_flows; i++)  {
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_A, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_B, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_C, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_D, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_E, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_F, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_G, i));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_H, i));
	}

	return 0;
}

/*
 * The transmit channels are disabled
 */
int pktdma_tx_disable(struct pktdma_tx_cfg *cfg)
{
	u32 i, v;

	for (i = 0; i < cfg->n_tx_chans; i++) {
		v = __raw_readl(cfg->tx_base + PKTDMA_REG_TCHAN_CFG_REG_A(i));
		
		if ((v & PKTDMA_REG_VAL_TCHAN_A_TX_ENABLE) ==
		    PKTDMA_REG_VAL_TCHAN_A_TX_ENABLE) {
			v = v | PKTDMA_REG_VAL_TCHAN_A_TX_TDOWN;
			__raw_writel(v, cfg->tx_base + PKTDMA_REG_TCHAN_CFG_REG_A(i));
		}
	}

	return 0;
}

/*
 * Configure the PKTDMA receive
 */
int pktdma_rx_config(struct pktdma_rx_cfg *cfg)
{
	struct qm_acc_cmd_config acc_cmd_cfg;
	u32                      v;
	u32                      i;
	int                      ret;

	if (pktdma_rx_disable(cfg) != 0)
		return -1;
	
	if (cfg->use_acc) {
		/*
		 * Setup Rx accumulator configuration for receive 
		 */
		acc_cmd_cfg.channel          = cfg->acc_channel;
		acc_cmd_cfg.command          = QM_ACC_CMD_ENABLE;
		acc_cmd_cfg.queue_mask       = 0;  /* none */
		acc_cmd_cfg.list_addr        = cfg->acc_list_phys_addr;
		acc_cmd_cfg.queue_index      = cfg->queue_rx;
		acc_cmd_cfg.max_entries      = cfg->acc_threshold + 1;
		/* In the future these parameters should be given in config too */
		acc_cmd_cfg.timer_count      = 40;
		acc_cmd_cfg.pacing_mode      = 1;  /* last interrupt mode */
		acc_cmd_cfg.list_entry_size  = 0;  /* D registers */
		acc_cmd_cfg.list_count_mode  = 0;  /* NULL terminate mode */
		acc_cmd_cfg.multi_queue_mode = 0;  /* single queue */
		
		ret = hw_qm_program_accumulator(0, &acc_cmd_cfg);
		
		if (ret != 0) {
			printk(KERN_ERR "%s: PKTDMA accumulator config failed (%d)\n",
			       __FUNCTION__, ret);
			return ret;
		}
	}
	
	/*
	 * Configure the flow
	 * The flow is configured to not pass extended info
	 * or psinfo, with descriptor type host
	 */
	v = PKTDMA_REG_VAL_MAKE_RX_FLOW_A(1,                     /* extended info passed */
					  1,                     /* psinfo passed */
					  0,
					  PKTDMA_DESC_TYPE_HOST,  /* Host type descriptor */
					  0,                     /* PS located in descriptor */
					  0,                     /* SOP offset */
					  cfg->qmnum_rx,
					  cfg->queue_rx);        /* Rx packet destination queue */
	
	__raw_writel(v, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_A, 0));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_B_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_B, 0));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_C_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_C, 0));

	v = PKTDMA_REG_VAL_MAKE_RX_FLOW_D(cfg->qmnum_free_buf,
					  cfg->queue_free_buf,
					  cfg->qmnum_free_buf,
					  cfg->queue_free_buf);

	__raw_writel(v, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_D, 0));
	
	/* Register E uses the same setup as D */
	__raw_writel(v, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_E, 0));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_F_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_F, 0));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_G_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_G, 0));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_H_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_H, 0));
	
	/* Enable the rx channels */
	for (i = 0; i < cfg->n_rx_chans; i++) 
		__raw_writel(PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE,
			     cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(i));
	
	return ret;
}

/*
 * The transmit channels are enabled
 */
int pktdma_tx_config(struct pktdma_tx_cfg *cfg)
{
	struct qm_acc_cmd_config acc_cmd_cfg;
	u32                      i;
	int                      ret;

	if (cfg->use_acc) {
		/*
		 * Setup Tx accumulator configuration for transmit
		 */
		acc_cmd_cfg.channel          = cfg->acc_channel;
		acc_cmd_cfg.command          = QM_ACC_CMD_ENABLE;
		acc_cmd_cfg.queue_mask       = 0;  /* none */
		acc_cmd_cfg.list_addr        = cfg->acc_list_phys_addr;
		acc_cmd_cfg.queue_index      = cfg->queue_tx;
		acc_cmd_cfg.max_entries      = cfg->acc_threshold + 1;
		/* In the future these parameters should be given in config too */
		acc_cmd_cfg.timer_count      = 40;
		acc_cmd_cfg.pacing_mode      = 1;  /* last interrupt mode */
		acc_cmd_cfg.list_entry_size  = 0;  /* D registers */
		acc_cmd_cfg.list_count_mode  = 0;  /* NULL terminate mode */
		acc_cmd_cfg.multi_queue_mode = 0;  /* single queue */
	
		ret = hw_qm_program_accumulator(0, &acc_cmd_cfg);
		
		if (ret != 0) {
			printk(KERN_ERR "%s: PKTDMA accumulator config failed (%d)\n",
			       __FUNCTION__, ret);
			return ret;
		}
	}

	/* Disable loopback in the tx direction */
	__raw_writel(PKTDMA_REG_VAL_EMU_CTL_NO_LOOPBACK,
		     cfg->gbl_ctl_base + PKTDMA_REG_EMU_CTL);

	/* Enable all channels. The current state isn't important */
	for (i = 0; i < cfg->n_tx_chans; i++) {
		__raw_writel(0, cfg->tx_base + PKTDMA_REG_TCHAN_CFG_REG_B(i));  /* Priority */
		__raw_writel(PKTDMA_REG_VAL_TCHAN_A_TX_ENABLE,
			     cfg->tx_base + PKTDMA_REG_TCHAN_CFG_REG_A(i));
	}

	return 0;
}
