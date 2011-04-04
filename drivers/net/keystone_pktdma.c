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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

#include <mach/pa.h>
#include <mach/netcp.h>
#include <mach/keystone_qmss.h>

#include <asm/system.h>

#include "keystone_pktdma.h"

#define DPRINTK(fmt, args...) printk(KERN_DEBUG "NETCP: [%s] " fmt, __FUNCTION__, ## args)

#define DEVICE_NUM_RX_DESCS	64
#define DEVICE_NUM_TX_DESCS	64
#define DEVICE_NUM_DESCS	(DEVICE_NUM_RX_DESCS + DEVICE_NUM_TX_DESCS)

#define KEYSTONE_CPSW_MIN_PACKET_SIZE	64
#define KEYSTONE_CPSW_MAX_PACKET_SIZE	(1500 + 14 + 4 + 4)
static int rx_packet_max = KEYSTONE_CPSW_MAX_PACKET_SIZE;
module_param(rx_packet_max, int, 0);
MODULE_PARM_DESC(rx_packet_max, "maximum receive packet size (bytes)");

struct pktdma_private {
	struct net_device *dev;
};

struct keystone_cpsw_priv {
	spinlock_t			lock;
	struct platform_device		*pdev;
	struct net_device		*ndev;
	struct resource			*res;
	struct napi_struct		napi;
	struct device			*dev;
	struct keystone_platform_data	data;
//	struct cpsw_regs __iomem	*regs;
//	struct cpsw_hw_stats __iomem	*hw_stats;
//	struct cpsw_host_regs __iomem	*host_port_regs;
	u32				msg_enable;
	struct net_device_stats		stats;
	int				rx_packet_max;
	int				host_port;
	struct clk			*clk;
};

/* The linking RAM */
u8 qm_linkram_buf[DEVICE_NUM_DESCS * 2 * (sizeof(u32) / sizeof(u8))];

/* The CPPI RAM */
u8 qm_cppi_buf[QM_DESC_SIZE_BYTES * DEVICE_NUM_DESCS];

static struct emac_config config = { 0, { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }};

inline long _hex_chartol (char c)
{
	if ((c >= '0') && (c <= '9')) return (c - '0');
	if ((c >= 'A') && (c <= 'F')) return (c - 'A') + 10;
	if ((c >= 'a') && (c <= 'f')) return (c - 'a') + 10;
	
	return -1;
}

static unsigned long _hex_strtoul (const char* str, const char** end)
{
	unsigned long ul = 0;
	long          ud;
	while ((ud = _hex_chartol(*str)) >= 0) {
		ul = (ul << 4) | ud;
		str++;
	}
	*end = str;
	return ul;
}

static int __init get_mac_addr_from_cmdline(char *str)
{
	const char *start = (const char *) str;
	const char *end;
	int count;

	for (count = 0; count < 6; count++) {
		config.enetaddr[count] = _hex_strtoul(start, &end);
		if (end == start)
			return 0;
		if ((*end != ':') && (count != 5))
			return 0;
		start = end + 1;
	}
	return 1;
}

__setup("emac_addr=", get_mac_addr_from_cmdline);

void target_init_qs(struct net_device *ndev)
{
	u32 i;
	struct qm_host_desc *hd;
	struct sk_buff   *skb;

	for (i = 0; i < DEVICE_NUM_RX_DESCS; i++) {
		skb = netdev_alloc_skb_ip_align(ndev, KEYSTONE_CPSW_MAX_PACKET_SIZE);
		hd = hw_qm_queue_pop(DEVICE_QM_FREE_Q);
		hd->buff_len        = skb_tailroom(skb);
		hd->orig_buffer_len = skb_tailroom(skb);
		hd->buff_ptr        = (u32)skb->data;
		hd->next_bdptr      = 0;
		hd->orig_buff_ptr   = (u32)skb;
		hw_qm_queue_push (hd, DEVICE_QM_LNK_BUF_Q, QM_DESC_SIZE_BYTES);
	}

	for (i = 0; i < DEVICE_NUM_TX_DESCS; i++) {
		hd		    = hw_qm_queue_pop(DEVICE_QM_FREE_Q);
		hd->buff_len        = 0;
		hd->buff_ptr        = 0;
		hd->next_bdptr      = 0;
		hd->orig_buffer_len = 0;
		hd->orig_buff_ptr   = 0;
	
		hw_qm_queue_push(hd, DEVICE_QM_TX_Q, QM_DESC_SIZE_BYTES);
	}
}

static int keystone_ndo_start_xmit(struct sk_buff *skb,
				   struct net_device *ndev)
{
	int ret, i;
	struct qm_host_desc		*hd;

	ret = skb_padto(skb, KEYSTONE_CPSW_MIN_PACKET_SIZE);
	if (unlikely(ret < 0)) {
		printk("packet pad failed");
		goto fail;
	}

	for (i = 0, hd = NULL; hd == NULL; i++, udelay(1000))
		hd = hw_qm_queue_pop(DEVICE_QM_TX_Q);

	if (hd == NULL)
		return (-1);

	QM_DESC_DESCINFO_SET_PKT_LEN(hd->desc_info, skb->len);
	
	hd->buff_len		= skb->len;
	hd->orig_buffer_len	= skb->len;
	hd->buff_ptr		= (u32)skb->data;
	hd->orig_buff_ptr	= (u32)skb;
    
	/* Return the descriptor back to the transmit queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_TX_Q);
	
	hw_qm_queue_push (hd, DEVICE_QM_ETH_TX_Q, QM_DESC_SIZE_BYTES);

	return NETDEV_TX_OK;
fail:
	netif_stop_queue(ndev);
	return NETDEV_TX_BUSY;
}

/* 
 * Disable all rx channels and clear all the flow registers
 * The teardown is initiated and polled for completion. The function will
 * return an error if the teardown is never complete, but will not stay
 * in the function forever.
 */
int cpdma_rx_disable(struct cpdma_rx_cfg *cfg)
{
	u32 i, v;
	u32 done;

	for (i = 0; i < cfg->n_rx_chans; i++) {
		/* If enabled, set the teardown bit */
		v = __raw_readl(cfg->rx_base + CPDMA_REG_RCHAN_CFG_REG_A(i));
		if ((v & CPDMA_REG_VAL_RCHAN_A_RX_ENABLE) == CPDMA_REG_VAL_RCHAN_A_RX_ENABLE ) {
			v = v | CPDMA_REG_VAL_RCHAN_A_RX_TDOWN;
			__raw_writel(cfg->rx_base + CPDMA_REG_RCHAN_CFG_REG_A(i), v);
		}
	}

	/* Poll for completion */
	for (i = 0, done = 0; ( (i < cfg->tdown_poll_count) && (done == 0) ); i++) {
		udelay(1000);
		done = 1;
		v = __raw_readl(cfg->rx_base + CPDMA_REG_RCHAN_CFG_REG_A(i));
		if ((v & CPDMA_REG_VAL_RCHAN_A_RX_ENABLE) == CPDMA_REG_VAL_RCHAN_A_RX_ENABLE)
			done = 0;
	}

	if (done == 0)
		return (-1);

	/* Clear all of the flow registers */
	for (i = 0; i < cfg->nrx_flows; i++)  {
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_A, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_B, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_C, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_D, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_E, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_F, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_G, i), 0);
        __raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_H, i), 0);
	}

	return (0);
}

/*
 * Configure the cpdma receive direction for boot loader
 * The receive configuration for boot consists of a single flow configuration
 * which is stored as flow configuration 0. All extended info and psinfo
 * is stripped
 */
int cpdma_rx_config(struct cpdma_rx_cfg *cfg)
{
	u32 v;
	u32 i;
	int ret = 0;

	if (cpdma_rx_disable(cfg) != 0)
		ret = -1;

	/*
	 * Configure the flow
	 * The flow is configured to not pass extended info
	 * or psinfo, with descriptor type host
	 */
	v = CPDMA_REG_VAL_MAKE_RX_FLOW_A(1,                     /* extended info passed */
                                         1,                     /* psinfo passed */
                                         0,                     
                                         CPDMA_DESC_TYPE_HOST,  /* Host type descriptor */
                                         0,                     /* PS located in descriptor */
                                         0,                     /* SOP offset */
                                         cfg->qmnum_rx,            
                                         cfg->queue_rx);        /* Rx packet destination queue */


	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_A, 0),
			v);

	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_B, 0),
			CPDMA_REG_VAL_RX_FLOW_B_DEFAULT);
	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_C, 0),
			CPDMA_REG_VAL_RX_FLOW_C_DEFAULT);

	v = CPDMA_REG_VAL_MAKE_RX_FLOW_D(cfg->qmnum_free_buf,
                                        cfg->queue_free_buf,
                                        cfg->qmnum_free_buf,
                                        cfg->queue_free_buf);

	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_D, 0), v);
  
    
	/* Register E uses the same setup as D */
	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_E, 0), v);



	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_F, 0),
			CPDMA_REG_VAL_RX_FLOW_F_DEFAULT);
	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_G, 0),
			CPDMA_REG_VAL_RX_FLOW_G_DEFAULT);
	__raw_writel(cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_H, 0),
			CPDMA_REG_VAL_RX_FLOW_H_DEFAULT);

	/* Enable the rx channels */
	for (i = 0; i < cfg->n_rx_chans; i++) 
		__raw_writel(cfg->rx_base + CPDMA_REG_RCHAN_CFG_REG_A(i),
				CPDMA_REG_VAL_RCHAN_A_RX_ENABLE);

	return (ret);
}

/*
 * The transmit channels are enabled
 */
int cpdma_tx_config(struct cpdma_tx_cfg *cfg)
{
	u32 i;

	/* Disable loopback in the tx direction */
	__raw_writel(cfg->gbl_ctl_base + CPDMA_REG_EMU_CTL,
			CPDMA_REG_VAL_EMU_CTL_NO_LOOPBACK);

	/* Enable all channels. The current state isn't important */
	for (i = 0; i < cfg->n_tx_chans; i++) {
		__raw_writel(cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_B(i), 0);  /* Priority */
		__raw_writel(cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_A(i),
				CPDMA_REG_VAL_TCHAN_A_TX_ENABLE);
	}

	return (0);
}

/*
 * The transmit channels are disabled
 */
int cpdma_tx_disable(struct cpdma_tx_cfg *cfg)
{
	u32 i, v;

	for (i = 0; i < cfg->n_tx_chans; i++) {
		v = __raw_readl(cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_A(i));

		if ((v & CPDMA_REG_VAL_TCHAN_A_TX_ENABLE) ==
				CPDMA_REG_VAL_TCHAN_A_TX_ENABLE) {
			v = v | CPDMA_REG_VAL_TCHAN_A_TX_TDOWN;
			__raw_writel(cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_A(i), v);
		 }
	}

	return (0);
}

/*
 * Reset the the gmac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
int mac_sl_reset(u16 port)
{
	u32 i, v;

	/* Set the soft reset bit */
	__raw_writel(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_RESET,
			CPGMAC_REG_RESET_VAL_RESET);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_RESET);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) != CPGMAC_REG_RESET_VAL_RESET)
			return (0);
	}

	/* Timeout on the reset */
	return (GMACSL_RET_WARN_RESET_INCOMPLETE);
}

/*
 * Configure the mac sliver
 */
int mac_sl_config(u16 port, struct mac_sliver *cfg)
{
	u32 v, i;
	int ret = GMACSL_RET_OK;

	if (cfg->max_rx_len > CPGMAC_REG_MAXLEN_LEN) {
		cfg->max_rx_len = CPGMAC_REG_MAXLEN_LEN;
		ret = GMACSL_RET_WARN_MAXLEN_TOO_BIG;
	}

	/* Must wait if the device is undergoing reset */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_RESET);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) != CPGMAC_REG_RESET_VAL_RESET)
			break;
	}

	if (i == DEVICE_EMACSL_RESET_POLL_COUNT)
		return (GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE);

	__raw_writel(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_MAXLEN,
			cfg->max_rx_len);
	__raw_writel(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_CTL, cfg->ctl);

	return (ret);
}

int cpmac_drv_start(void)
{
	struct mac_sliver cfg;

	cfg.max_rx_len	= MAX_SIZE_STREAM_BUFFER;
	cfg.ctl		= GMACSL_ENABLE | GMACSL_RX_ENABLE_EXT_CTL;

        mac_sl_reset(0);
        mac_sl_config(0, &cfg);

	return (0);
}
#if 0
int cpmac_drv_stop(void)
{
	mac_sl_reset(0);
	return (0);
}

int target_mac_rcv(u8 *buffer)
{
	Int32           pktSizeBytes; 
	qmHostDesc_t   *hd;

	hd = hwQmQueuePop (DEVICE_QM_RCV_Q);
	if (hd == NULL)
		return (0);

	pktSizeBytes = QM_DESC_DESCINFO_GET_PKT_LEN(hd->descInfo);
	iblMemcpy ((void *)buffer, (void *)hd->buffPtr, pktSizeBytes);
	
	hd->buffLen = hd->origBufferLen;
	hd->buffPtr = hd->origBuffPtr;

	hwQmQueuePush (hd, DEVICE_QM_LNK_BUF_Q, QM_DESC_SIZE_BYTES);

	return (pktSizeBytes);
}
#endif
static void keystone_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}

/*
 * Open the device
 */
static int  keystone_ndo_open(struct net_device *dev)
{
	netif_start_queue(dev);

	return 0;
}

/*
 * Close the device
 */
static int keystone_ndo_stop(struct net_device *dev)
{
	netif_stop_queue(dev);

	return 0;
}

static const struct net_device_ops keystone_netdev_ops = {
	.ndo_open		= keystone_ndo_open,
	.ndo_stop		= keystone_ndo_stop,
	.ndo_start_xmit		= keystone_ndo_start_xmit,
	.ndo_change_rx_flags	= keystone_ndo_change_rx_flags,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
//	.ndo_tx_timeout		= keystone_ndo_tx_timeout,
#ifdef CONFIG_NET_POLL_CONTROLLER
//	.ndo_poll_controller	= keystone_ndo_poll_controller,
#endif
};

static int __devinit pktdma_probe(struct platform_device *pdev)
{
	struct keystone_platform_data	*data = pdev->dev.platform_data;
	struct net_device		*ndev;
	struct keystone_cpsw_priv	*priv;
	int                             ret = 0; 
#ifdef EMAC_ARCH_HAS_MAC_ADDR
	char				hw_emac_addr[6];
#endif
	struct cpdma_rx_cfg		c_rx_cfg;
	struct cpdma_tx_cfg		c_tx_cfg;
	struct qm_config		c_q_cfg;
	struct cpdma_rx_cfg		*rx_cfg = &c_rx_cfg;
	struct cpdma_tx_cfg		*tx_cfg = &c_tx_cfg;
	struct qm_config		*q_cfg  = &c_q_cfg; 

	if (!data) {
		pr_err("keystone cpsw: platform data missing\n");
		return -ENODEV;
	}

	ndev = alloc_etherdev(sizeof(struct keystone_cpsw_priv));
	if (!ndev) {
		pr_err("keystone cpsw: error allocating net_device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ndev);
	priv = netdev_priv(ndev);
	spin_lock_init(&priv->lock);
	priv->data = *data;
	priv->pdev = pdev;
	priv->ndev = ndev;
	priv->dev  = &ndev->dev;
	priv->rx_packet_max = max(rx_packet_max, 128);

	cpmac_drv_start();
	
	rx_cfg->rx_base = DEVICE_PA_CDMA_RX_CHAN_CFG_BASE;
	rx_cfg->n_rx_chans = DEVICE_PA_CDMA_RX_NUM_CHANNELS;
	rx_cfg->flow_base = DEVICE_PA_CDMA_RX_FLOW_CFG_BASE;
	rx_cfg->nrx_flows = DEVICE_PA_CDMA_RX_NUM_FLOWS;
	rx_cfg->qmnum_free_buf = 0;
	rx_cfg->queue_free_buf = DEVICE_QM_LNK_BUF_Q;
	rx_cfg->qmnum_rx = 0;
	rx_cfg->queue_rx = DEVICE_QM_RCV_Q;
	rx_cfg->tdown_poll_count = DEVICE_RX_CDMA_TIMEOUT_COUNT;

	tx_cfg->gbl_ctl_base = DEVICE_PA_CDMA_GLOBAL_CFG_BASE;
	tx_cfg->tx_base = DEVICE_PA_CDMA_TX_CHAN_CFG_BASE;
	tx_cfg->n_tx_chans = DEVICE_PA_CDMA_TX_NUM_CHANNELS;

	cpdma_rx_config(rx_cfg);
	cpdma_tx_config(tx_cfg);
	
	q_cfg->link_ram_base		= 0x2a800000;
	q_cfg->link_ram_size		= 0x2000;
	q_cfg->mem_region_base		= RAM_MSM_BASE + 0x100000; //  0x10860000;
	q_cfg->mem_regnum_descriptors	= DEVICE_NUM_DESCS;
	q_cfg->dest_q			= DEVICE_QM_FREE_Q;

	hw_qm_setup(q_cfg);
	
	target_init_qs(ndev);

#ifdef EMAC_ARCH_HAS_MAC_ADDR
	/* SoC or board hw has MAC address */
	if (config.enetaddr[0] == 0 && config.enetaddr[1] == 0 &&
	    config.enetaddr[2] == 0 && config.enetaddr[3] == 0 &&
	    config.enetaddr[4] == 0 && config.enetaddr[5] == 0) {
		if (!emac_arch_get_mac_addr(hw_emac_addr))
			for (i = 0; i <= 5; i++)
				config.enetaddr[i] = hw_emac_addr[i] & 0xff;
	}
#endif

//	if (is_valid_ether_addr(data->mac_addr))
		memcpy(ndev->dev_addr, config.enetaddr, ETH_ALEN);
//	else
//		random_ether_addr(ndev->dev_addr);

	ndev->flags |= IFF_ALLMULTI;	

	ndev->netdev_ops = &keystone_netdev_ops;

	/* register the network device */
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->dev, "error registering net device\n");
		ret = -ENODEV;
		goto clean_ndev_ret;
	}

	return 0;

clean_ndev_ret:
	free_netdev(ndev);
	return ret;
}

static int __devexit pktdma_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	
	platform_set_drvdata(pdev, NULL);

	hw_qm_teardown();
	free_netdev(ndev);

	return 0;
}

static int pktdma_suspend(struct device *dev)
{
	return 0;
}

static int pktdma_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pktdma_pm_ops = {
	.suspend	= pktdma_suspend,
	.resume		= pktdma_resume,
};

static struct platform_driver pktdma_driver = {
	.driver = {
		.name	 = "keystone_pktdma",
		.owner	 = THIS_MODULE,
		.pm	 = &pktdma_pm_ops,
	},
	.probe = pktdma_probe,
	.remove = __devexit_p(pktdma_remove),
};

static int __init pktdma_init(void)
{
	return platform_driver_register(&pktdma_driver);
}
late_initcall(pktdma_init);

static void __exit pktdma_exit(void)
{
	platform_driver_unregister(&pktdma_driver);
}
module_exit(pktdma_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone Packet DMA driver");
