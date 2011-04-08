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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/timer.h>

#include <mach/pa.h>
#include <mach/netcp.h>
#include <mach/keystone_qmss.h>

#include <asm/system.h>

#include "keystone_pktdma.h"

#undef EMAC_DEBUG
#ifdef EMAC_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "NETCP: [%s] " fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define DEVICE_POLLING_PERIOD_MSEC      10
#define DEVICE_NUM_RX_DESCS	        64
#define DEVICE_NUM_TX_DESCS	        64
#define DEVICE_NUM_DESCS	        (DEVICE_NUM_RX_DESCS + DEVICE_NUM_TX_DESCS)

#define KEYSTONE_CPSW_MIN_PACKET_SIZE	ETH_ZLEN
#define KEYSTONE_CPSW_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

static int rx_packet_max = KEYSTONE_CPSW_MAX_PACKET_SIZE;
module_param(rx_packet_max, int, 0);
MODULE_PARM_DESC(rx_packet_max, "maximum receive packet size (bytes)");

static struct timer_list poll_timer;

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

static inline long _hex_chartol (char c)
{
	if ((c >= '0') && (c <= '9')) return (c - '0');
	if ((c >= 'A') && (c <= 'F')) return (c - 'A') + 10;
	if ((c >= 'a') && (c <= 'f')) return (c - 'a') + 10;
	
	return -1;
}

static u8 _hex_strtoul (const char* str, const char** end)
{
	unsigned long ul = 0;
	long          ud;
	while ((ud = _hex_chartol(*str)) >= 0) {
		ul = (ul << 4) | ud;
		str++;
	}
	*end = str;
	return (u8) ul;
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

/*
 * Initialize queues used by the PKTDMA
 */
static void cpdma_init_qs(struct net_device *ndev)
{
	u32 i;
	struct qm_host_desc *hd;
	struct sk_buff      *skb;

	for (i = 0; i < DEVICE_NUM_RX_DESCS; i++) {
		skb = netdev_alloc_skb_ip_align(ndev, KEYSTONE_CPSW_MAX_PACKET_SIZE);
		hd = hw_qm_queue_pop(DEVICE_QM_FREE_Q);
		hd->buff_len       = skb_tailroom(skb);
		hd->orig_buff_len  = skb_tailroom(skb);
		hd->buff_ptr       = (u32)skb->data;
		hd->next_bdptr     = 0;
		hd->orig_buff_ptr  = (u32)skb->data;
		hd->private        = (u32)skb;
		hw_qm_queue_push(hd, DEVICE_QM_RX_Q, QM_DESC_SIZE_BYTES);
	}
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
			__raw_writel(v, cfg->rx_base + CPDMA_REG_RCHAN_CFG_REG_A(i));
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
		return -1;

	/* Clear all of the flow registers */
	for (i = 0; i < cfg->nrx_flows; i++)  {
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_A, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_B, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_C, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_D, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_E, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_F, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_G, i));
		__raw_writel(0, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_H, i));
	}

	return 0;
}

/*
 * Configure the CPDMA receive
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


	__raw_writel(v, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_A, 0));

	__raw_writel(CPDMA_REG_VAL_RX_FLOW_B_DEFAULT,
		     cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_B, 0));

	__raw_writel(CPDMA_REG_VAL_RX_FLOW_C_DEFAULT,
		     cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_C, 0));

	v = CPDMA_REG_VAL_MAKE_RX_FLOW_D(cfg->qmnum_free_buf,
					 cfg->queue_free_buf,
					 cfg->qmnum_free_buf,
					 cfg->queue_free_buf);

	__raw_writel(v, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_D, 0));
    
	/* Register E uses the same setup as D */
	__raw_writel(v, cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_E, 0));
	
	__raw_writel(CPDMA_REG_VAL_RX_FLOW_F_DEFAULT,
		     cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_F, 0));
	
	__raw_writel(CPDMA_REG_VAL_RX_FLOW_G_DEFAULT,
		     cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_G, 0));
	
	__raw_writel(CPDMA_REG_VAL_RX_FLOW_H_DEFAULT,
		     cfg->flow_base + CPDMA_RX_FLOW_CFG(CPDMA_RX_FLOW_REG_H, 0));
	
	/* Enable the rx channels */
	for (i = 0; i < cfg->n_rx_chans; i++) 
		__raw_writel(CPDMA_REG_VAL_RCHAN_A_RX_ENABLE,
			     cfg->rx_base + CPDMA_REG_RCHAN_CFG_REG_A(i));

	return ret;
}

/*
 * The transmit channels are enabled
 */
int cpdma_tx_config(struct cpdma_tx_cfg *cfg)
{
	u32 i;

	/* Disable loopback in the tx direction */
	__raw_writel(CPDMA_REG_VAL_EMU_CTL_NO_LOOPBACK,
		     cfg->gbl_ctl_base + CPDMA_REG_EMU_CTL);

	/* Enable all channels. The current state isn't important */
	for (i = 0; i < cfg->n_tx_chans; i++) {
		__raw_writel(0, cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_B(i));  /* Priority */
		__raw_writel(CPDMA_REG_VAL_TCHAN_A_TX_ENABLE,
			     cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_A(i));
	}

	return 0;
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
			__raw_writel(v, cfg->tx_base + CPDMA_REG_TCHAN_CFG_REG_A(i));
		}
	}

	return 0;
}

/*
 * Reset the the gmac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
int mac_sl_reset(u16 port)
{
	u32 i, v;

	/* Set the soft reset bit */
	__raw_writel(CPGMAC_REG_RESET_VAL_RESET, 
		     DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_RESET);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_RESET);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) != CPGMAC_REG_RESET_VAL_RESET)
			return 0;
	}

	/* Timeout on the reset */
	return GMACSL_RET_WARN_RESET_INCOMPLETE;
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
		return GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE;

	__raw_writel(cfg->max_rx_len, DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_MAXLEN);
	__raw_writel(cfg->ctl, DEVICE_EMACSL_BASE(port) + CPGMACSL_REG_CTL);

	return ret;
}

static int cpmac_drv_start(void)
{
	struct mac_sliver cfg;
	int i;

	cfg.max_rx_len	= MAX_SIZE_STREAM_BUFFER;
	cfg.ctl		= GMACSL_ENABLE | GMACSL_RX_ENABLE_EXT_CTL;
       
	for (i = 0; i < DEVICE_N_GMACSL_PORTS; i++)  {
		mac_sl_reset(i);
		mac_sl_config(i, &cfg);
        }

	return 0;
}

static int cpmac_drv_stop(void)
{
	int i;

	for (i = 0; i < DEVICE_N_GMACSL_PORTS; i++)
		mac_sl_reset(i);

	return 0;
}
 
/*
 * Pop an incoming packet
 */
static int pktdma_rx(struct net_device *ndev)
{
	int                  pkt_size;
	struct qm_host_desc *hd;
	struct sk_buff      *skb;
	struct sk_buff      *skb_rcv;

	hd = hw_qm_queue_pop(DEVICE_QM_ETH_RX_Q);
	if (hd == NULL)
		return 0;

	pkt_size = QM_DESC_DESCINFO_GET_PKT_LEN(hd->desc_info);

	/* Get the incoming skbuff */
	skb_rcv = (struct sk_buff *) hd->private;

	DPRINTK("received a packet of len %d (skb=0x%x, hd=0x%x) buffer = 0x%x\n",
		pkt_size, skb_rcv, hd, hd->desc_info);

	/* Prepare the received sk_buff */
	skb_rcv->dev = ndev;
	skb_put(skb_rcv, pkt_size);
	skb_rcv->protocol = eth_type_trans(skb_rcv, ndev);

	/* Cache sync here */
	L2_cache_block_invalidate((u32) hd->orig_buff_ptr,
				  (u32) hd->orig_buff_ptr + pkt_size);
			
	/* Allocate a new skb */
	skb = netdev_alloc_skb_ip_align(ndev, KEYSTONE_CPSW_MAX_PACKET_SIZE);
	if (skb != NULL) {
		/* Attach the new one */
		hd->buff_len      = skb_tailroom(skb);
		hd->orig_buff_len = skb_tailroom(skb);
		hd->buff_ptr      = (u32)skb->data;
		hd->next_bdptr    = 0;
		hd->orig_buff_ptr = (u32)skb->data;
		hd->private       = (u32)skb;
		hw_qm_queue_push(hd, DEVICE_QM_RX_Q, QM_DESC_SIZE_BYTES);
	}

        /* Give back old sk_buff */
	netif_rx(skb_rcv);

	ndev->last_rx = jiffies;

	/* Fill statistic */
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += pkt_size;

	return pkt_size;
}

/*
 * Release a transmitted packet
 */
static int pktdma_tx(struct net_device *ndev)
{	
	struct qm_host_desc *hd;
	
	hd = hw_qm_queue_pop(DEVICE_QM_TX_Q);
	if (hd != NULL) {
		DPRINTK("Tx packet relaxed (skbuff=0x%x, hd=0x%x)\n",
			hd->private, hd);

		if (hd->private)
			/* Free the skbuff associated to this packet */
			dev_kfree_skb_irq((struct sk_buff*) hd->private);

		if (netif_running(ndev))
			netif_wake_queue(ndev);

		/* Send packet back to available free buffer queue */
		hw_qm_queue_push(hd, DEVICE_QM_FREE_Q, QM_DESC_SIZE_BYTES);

		return 1;
	}
	return 0;
}

/*
 * PKTDMA interrupt handling functions
 */
static irqreturn_t pktdma_rx_interrupt(int irq, void * netdev_id)
{
	struct net_device *ndev = (struct net_device *) netdev_id;

	for (;;) {
		if (pktdma_rx(ndev) == 0)
			break;
	}

	return IRQ_HANDLED;
}

static irqreturn_t pktdma_tx_interrupt(int irq, void * netdev_id)
{
	struct net_device         *ndev = (struct net_device *) netdev_id;

	for (;;) {
		if (pktdma_tx(ndev) == 0);
		break;
	}
	
	return IRQ_HANDLED;
}

/*
 * Poll the device
 */
static void keystone_ndo_poll_controller(struct net_device *ndev)
{
	// disable_netcp_irq()

	pktdma_rx_interrupt(0, ndev);
	pktdma_tx_interrupt(0, ndev);

	// enable_netcp_irq();

	/* Ugly hack */
	mod_timer(&poll_timer,
		  jiffies + msecs_to_jiffies(DEVICE_POLLING_PERIOD_MSEC));
}

/*
 * Push an outcoming packet
 */
static int keystone_ndo_start_xmit(struct sk_buff *skb,
				   struct net_device *ndev)
{
	int                  ret;
	unsigned             pkt_len = skb->len;
	struct qm_host_desc *hd;

	if (unlikely(pkt_len < KEYSTONE_CPSW_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, KEYSTONE_CPSW_MIN_PACKET_SIZE);
		if (unlikely(ret < 0)) {
			printk("packet pad failed");
			goto fail;
		}
		pkt_len = KEYSTONE_CPSW_MIN_PACKET_SIZE;
	}

	hd  = hw_qm_queue_pop(DEVICE_QM_FREE_Q);
	if (hd == NULL) {
		DPRINTK("no packet retrieved\n");
		return -1;
	}

	/* No coherency is assumed between PKTDMA and L2 cache */
	L2_cache_block_writeback((u32) skb->data,
				 (u32) skb->data + skb->len);

	QM_DESC_DESCINFO_SET_PKT_LEN(hd->desc_info, pkt_len);
	
	hd->buff_len		= pkt_len;
	hd->orig_buff_len	= skb->len;
	hd->buff_ptr		= (u32)skb->data;
	hd->orig_buff_ptr	= (u32)skb->data;
	hd->private             = (u32)skb;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += pkt_len;
	ndev->trans_start     = jiffies;

	DPRINTK("transmitting packet of len %d (skb=0x%x, hd=0x%x)\n",
		skb->len, skb, hd);

	/* Return the descriptor back to the transmit queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_TX_Q);
	
	hw_qm_queue_push(hd, DEVICE_QM_ETH_TX_Q, QM_DESC_SIZE_BYTES);

	return NETDEV_TX_OK;
fail:
	netif_stop_queue(ndev);
	return NETDEV_TX_BUSY;
}

/*
 * Change receive flags
 */
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
static int keystone_ndo_open(struct net_device *ndev)
{
	/* Start CPMAC */
	cpmac_drv_start();

	netif_start_queue(ndev);

	DPRINTK("starting to poll\n");

	/* Ugly hack: use timeout to poll */
	init_timer(&poll_timer);
	poll_timer.function = keystone_ndo_poll_controller;
	poll_timer.data     = (unsigned long) ndev;
	mod_timer(&poll_timer,
		  jiffies + msecs_to_jiffies(DEVICE_POLLING_PERIOD_MSEC));

	return 0;
}

/*
 * Close the device
 */
static int keystone_ndo_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	
	cpmac_drv_stop();

	return 0;
}

/*
 * Called by the kernel to send a packet out into the void
 * of the net.
 */
static void keystone_ndo_tx_timeout(struct net_device *ndev)
{
	printk(KERN_WARNING "%s: transmit timed out\n", ndev->name);
	ndev->stats.tx_errors++;
	ndev->trans_start = jiffies;
	netif_wake_queue(ndev);
}

static const struct net_device_ops keystone_netdev_ops = {
	.ndo_open		= keystone_ndo_open,
	.ndo_stop		= keystone_ndo_stop,
	.ndo_start_xmit		= keystone_ndo_start_xmit,
	.ndo_change_rx_flags	= keystone_ndo_change_rx_flags,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= keystone_ndo_tx_timeout,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= keystone_ndo_poll_controller,
#endif
};

static int __devinit pktdma_probe(struct platform_device *pdev)
{
	struct keystone_platform_data	*data = pdev->dev.platform_data;
	struct net_device		*ndev;
	struct keystone_cpsw_priv	*priv;
	int                             i, ret = 0;
#ifdef EMAC_ARCH_HAS_MAC_ADDR
	u8				hw_emac_addr[6];
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

	if (is_valid_ether_addr(config.enetaddr))
		memcpy(ndev->dev_addr, config.enetaddr, ETH_ALEN);
	else
		random_ether_addr(ndev->dev_addr);

	/* Use internal link RAM according to SPRUGR9B section 4.1.1.3 */
	q_cfg->link_ram_base		= 0x00080000;
	q_cfg->link_ram_size		= 0x3FFF;

	/* Use L2 lower 256KB memory for descriptors */
	q_cfg->mem_region_base		= RAM_SRAM_BASE;
	q_cfg->mem_regnum_descriptors	= DEVICE_NUM_DESCS;
	q_cfg->dest_q			= DEVICE_QM_FREE_Q;

	/* Initialize QM */
	hw_qm_setup(q_cfg);

	/* Configure Rx and Tx PKTDMA */
	rx_cfg->rx_base          = DEVICE_PA_CDMA_RX_CHAN_CFG_BASE;
	rx_cfg->n_rx_chans       = DEVICE_PA_CDMA_RX_NUM_CHANNELS;
	rx_cfg->flow_base        = DEVICE_PA_CDMA_RX_FLOW_CFG_BASE;
	rx_cfg->nrx_flows        = DEVICE_PA_CDMA_RX_NUM_FLOWS;
	rx_cfg->qmnum_free_buf   = 0;
	rx_cfg->queue_free_buf   = DEVICE_QM_RX_Q;
	rx_cfg->qmnum_rx         = 0;
	rx_cfg->queue_rx         = DEVICE_QM_ETH_RX_Q;
	rx_cfg->tdown_poll_count = DEVICE_RX_CDMA_TIMEOUT_COUNT;

	cpdma_rx_config(rx_cfg);

	tx_cfg->gbl_ctl_base     = DEVICE_PA_CDMA_GLOBAL_CFG_BASE;
	tx_cfg->tx_base          = DEVICE_PA_CDMA_TX_CHAN_CFG_BASE;
	tx_cfg->n_tx_chans       = DEVICE_PA_CDMA_TX_NUM_CHANNELS;

	cpdma_tx_config(tx_cfg);

	/* Initialize PKTDMA queues */
	cpdma_init_qs(ndev);

	/* Configure the PA */
	ret = keystone_pa_config(ndev->dev_addr);
	if (ret != 0) {
		printk("PA init failed \n");
		return ret;
	}

	/* Streaming switch configuration */
	streaming_switch_setup();

	/* Setup Ethernet driver function */
	ether_setup(ndev);
	
	/* Register the network device */
	ndev->dev_id     = 0;
	ndev->flags     |= IFF_ALLMULTI;	
	ndev->netdev_ops = &keystone_netdev_ops;
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
module_init(pktdma_init);

static void __exit pktdma_exit(void)
{
	platform_driver_unregister(&pktdma_driver);
}
module_exit(pktdma_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone Packet DMA driver");
