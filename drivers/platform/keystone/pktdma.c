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
	u32 i, v, c, f;
	u32 done;
	int ret = 0;

	for (c = cfg->rx_chan; c < cfg->n_rx_chans; c++) {
		/* If enabled, set the teardown bit */
		v = __raw_readl(cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(c));
		if ((v & PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE) == PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE ) {
			v = v | PKTDMA_REG_VAL_RCHAN_A_RX_TDOWN;
			__raw_writel(v, cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(c));
		}


		/* Poll for completion */
		for (i = 0, done = 0; ( (i < cfg->tdown_poll_count) && (done == 0) ); i++) {
		    udelay(1000);
		    done = 1;
		    v = __raw_readl(cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(c));
		    if ((v & PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE) == PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE)
			done = 0;
		}

		if (done == 0)
		    ret = -1;
	}

	/* Clear all of the flow registers */
	for (f = cfg->rx_flow; f < cfg->n_rx_flows; f++)  {
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_A, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_B, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_C, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_D, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_E, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_F, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_G, f));
		__raw_writel(0, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_H, f));
	}

	return ret;
}

/*
 * The transmit channels are disabled
 */
int pktdma_tx_disable(struct pktdma_tx_cfg *cfg)
{
	u32 i, v;

	for (i = cfg->tx_chan; i < cfg->n_tx_chans; i++) {
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
 * Configure the flow
 */
void pktdma_flow_config(struct pktdma_rx_cfg *cfg,
			int flow,
			u32 queue_rx,
			u32 queue_free_buf)
{
	u32 v;

	/*
	 * Configure the flow
	 * The flow is configured to not pass extended info
	 * or psinfo, with descriptor type host
	 */
	v = PKTDMA_REG_VAL_MAKE_RX_FLOW_A(1,                     /* extended info passed */
					  1,                     /* psinfo passed */
					  0,
					  PKTDMA_DESC_TYPE_HOST, /* Host type descriptor */
					  0,                     /* PS located in descriptor */
					  0,                     /* SOP offset */
					  cfg->qmnum_rx,
					  queue_rx);             /* Rx packet destination queue */
	
	__raw_writel(v, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_A, flow));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_B_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_B, flow));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_C_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_C, flow));

	v = PKTDMA_REG_VAL_MAKE_RX_FLOW_D(cfg->qmnum_free_buf,
					  queue_free_buf,
					  cfg->qmnum_free_buf,
					  queue_free_buf);

	__raw_writel(v, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_D, flow));
	
	/* Register E uses the same setup as D */
	__raw_writel(v, cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_E, flow));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_F_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_F, flow));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_G_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_G, flow));
	
	__raw_writel(PKTDMA_REG_VAL_RX_FLOW_H_DEFAULT,
		     cfg->flow_base + PKTDMA_RX_FLOW_CFG(PKTDMA_RX_FLOW_REG_H, flow));

}

/*
 * Configure the PKTDMA receive and enable rx channels
 */
int pktdma_rx_config(struct pktdma_rx_cfg *cfg)
{
	struct qm_acc_cmd_config acc_cmd_cfg;
	u32 i, c, f;
	int ret = 0;

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
		acc_cmd_cfg.queue_index      = cfg->queue_rx[0]; /* First queue */
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
	
	/* Configue rx flows */
	for (f = cfg->rx_flow, i = 0; f < cfg->n_rx_flows; f++, i++)  {
		pktdma_flow_config(cfg, f,
				   cfg->queue_rx[i],
				   cfg->queue_free_buf[i]);
	}

	/* Enable the rx channels */
	for (c = cfg->rx_chan; c < cfg->n_rx_chans; c++) 
		__raw_writel(PKTDMA_REG_VAL_RCHAN_A_RX_ENABLE,
			     cfg->rx_base + PKTDMA_REG_RCHAN_CFG_REG_A(c));
	
	return ret;
}

/*
 * Configure the PKTDMA transmit and enable tx channels
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
	for (i = cfg->tx_chan; i < cfg->n_tx_chans; i++) {
		__raw_writel(0, cfg->tx_base + PKTDMA_REG_TCHAN_CFG_REG_B(i));  /* Priority */
		__raw_writel(PKTDMA_REG_VAL_TCHAN_A_TX_ENABLE,
			     cfg->tx_base + PKTDMA_REG_TCHAN_CFG_REG_A(i));
	}

	return 0;
}
