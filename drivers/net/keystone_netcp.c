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
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/mii.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/msmc.h>
#include <asm/mdio.h>

#include <mach/keystone_qmss.h>
#include <mach/keystone_netcp.h>
#include <mach/keystone_cpsw.h>
#include <linux/keystone/qmss.h>
#include <linux/keystone/pa.h>
#include <linux/keystone/pktdma.h>
#include <linux/keystone/cpsw.h>

#define NETCP_DRIVER_NAME    "TI KeyStone NetCP Driver"
#define NETCP_DRIVER_VERSION "v1.2"
#undef NETCP_DEBUG
#ifdef NETCP_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_CRIT "NETCP: [%s] " fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define NETCP_DEBUG (NETIF_MSG_HW	| NETIF_MSG_WOL		|	\
		     NETIF_MSG_DRV	| NETIF_MSG_LINK	|	\
		     NETIF_MSG_IFUP	| NETIF_MSG_INTR	|	\
		     NETIF_MSG_PROBE	| NETIF_MSG_TIMER	|	\
		     NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	|	\
		     NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	|	\
		     NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	|	\
		     NETIF_MSG_RX_STATUS)

#define DEVICE_NAPI_WEIGHT              16
#define DEVICE_POLLING_PERIOD_MSEC      10
#define DEVICE_TX_LOOP_FREE_QUEUE       20  /* number of loop to spin on free queue */
#define DEVICE_TX_TIMEOUT               40  /* transmit timeout */
#define DEVICE_NUM_RX_DESCS             64

#define NETCP_MIN_PACKET_SIZE	        ETH_ZLEN
#define NETCP_MAX_PACKET_SIZE	        (VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

static void __iomem	*mac_sl;

static inline void mac_sl_write_reg(u32 val, int reg)
{
	__raw_writel(val, mac_sl + reg);
}

static inline u32 mac_sl_read_reg(int reg)
{
	return __raw_readl(mac_sl + reg);
}


static int rx_packet_max = NETCP_MAX_PACKET_SIZE;
module_param(rx_packet_max, int, 0);
MODULE_PARM_DESC(rx_packet_max, "maximum receive packet size (bytes)");

static int debug_level;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "keystone NetCP debug level (NETIF_MSG bits)");

static struct timer_list poll_timer;

struct netcp_priv {
	spinlock_t			lock;
	struct platform_device	       *pdev;
	struct net_device	       *ndev;
	struct resource		       *res;
	struct napi_struct		napi;
	struct device		       *dev;
	struct netcp_platform_data	data;
	u32				msg_enable;
	struct net_device_stats		stats;
	struct mii_if_info              mii;
	int				rx_packet_max;
	int				host_port;
	struct clk		       *clk;
};

#ifdef EMAC_ARCH_HAS_INTERRUPT
/* The QM accumulator RAM */
static u32 *acc_list_addr_rx;
static u32 *acc_list_addr_tx;
static u32 *acc_list_phys_addr_rx;
static u32 *acc_list_phys_addr_tx;
#endif
static u32 netcp_irq_enabled = 0;

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
 * Cleanup queues used by the QMSS/PKTDMA
 */
static void netcp_cleanup_qs(struct net_device *ndev)
{
	struct qm_host_desc *hd = NULL;

	while ((hd = hw_qm_queue_pop(DEVICE_QM_ETH_RX_Q)) && (hd != NULL))
		hw_qm_queue_push(hd, DEVICE_QM_ETH_FREE_Q, DEVICE_QM_DESC_SIZE_BYTES);
}

/*
 * Initialize queues used by the QMSS/PKTDMA
 */
static void netcp_init_qs(struct net_device *ndev)
{
	u32 i;
	struct qm_host_desc *hd;
	struct sk_buff      *skb;

	for (i = 0; i < DEVICE_NUM_RX_DESCS; i++) {
		skb = netdev_alloc_skb_ip_align(ndev, NETCP_MAX_PACKET_SIZE);

		hd = hw_qm_queue_pop(DEVICE_QM_ETH_FREE_Q);

		hd->buff_len       = skb_tailroom(skb);
		hd->orig_buff_len  = skb_tailroom(skb);
		hd->buff_ptr       = (u32)skb->data;
		hd->next_bdptr     = 0;
		hd->orig_buff_ptr  = (u32)skb->data;
		hd->private        = (u32)skb;

		hw_qm_queue_push(hd, DEVICE_QM_ETH_RX_FREE_Q, DEVICE_QM_DESC_SIZE_BYTES);		

		L2_cache_block_writeback_invalidate((u32) skb->data,
						    (u32) skb->data + NETCP_MAX_PACKET_SIZE);
	}
}

/*
 * Reset the the gmac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
int mac_sl_reset(u16 port)
{
	u32 i, v;

	/* Set the soft reset bit */
	mac_sl_write_reg(CPGMAC_REG_RESET_VAL_RESET,
		     DEVICE_EMACSL_PORT(port) + CPGMACSL_REG_RESET);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = mac_sl_read_reg(DEVICE_EMACSL_PORT(port) + CPGMACSL_REG_RESET);
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
		v = mac_sl_read_reg(DEVICE_EMACSL_PORT(port) + CPGMACSL_REG_RESET);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) != CPGMAC_REG_RESET_VAL_RESET)
			break;
	}

	if (i == DEVICE_EMACSL_RESET_POLL_COUNT)
		return GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE;

	mac_sl_write_reg(cfg->max_rx_len, DEVICE_EMACSL_PORT(port) + CPGMACSL_REG_MAXLEN);
	mac_sl_write_reg(cfg->ctl, DEVICE_EMACSL_PORT(port) + CPGMACSL_REG_CTL);

	return ret;
}

static int cpmac_init(void)
{
	mac_sl = ioremap(DEVICE_EMACSL_BASE, 0x100);

	return 0;
}

static int cpmac_drv_start(void)
{
	struct mac_sliver cfg;
	int i;

	cfg.max_rx_len	= MAX_SIZE_STREAM_BUFFER;
	cfg.ctl		= GMACSL_ENABLE | GMACSL_RX_ENABLE_EXT_CTL |
				GMACSL_ENABLE_FULL_DUPLEX | 
				GMACSL_ENABLE_GIG_MODE;
       
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
 * Interrupt management 
 * 
 * Because NetCP interrupts do not exactly match the behavior of a standard NIC
 * and cannot be easily enable/disable, we use a software switch instead.
 *
 * Real interrupt re-enabling for NAPI is done within netcp_*_irq_ack().
 */
static inline void netcp_irq_disable(struct net_device *ndev)
{
	netcp_irq_enabled = 0;
}

static inline void netcp_irq_enable(struct net_device *ndev)
{
	netcp_irq_enabled = 1;
}

static inline void netcp_rx_irq_ack(struct net_device *ndev)
{
#ifdef EMAC_ARCH_HAS_INTERRUPT
	if (netcp_irq_enabled) {
		/* Ack Rx interrupt */
		if (__raw_readl(DEVICE_QM_INTD_BASE + QM_REG_INTD_STATUS0) &
		    (1 << DEVICE_QM_ETH_ACC_RX_CHANNEL)) {
			__raw_writel(1, QM_REG_INTD_COUNT_IRQ(DEVICE_QM_ETH_ACC_RX_CHANNEL));
			__raw_writel(QM_REG_INTD_EOI_HIGH_PRIO_INDEX + DEVICE_QM_ETH_ACC_RX_CHANNEL,
				     DEVICE_QM_INTD_BASE + QM_REG_INTD_EOI);
		}
	}
#endif
}

static inline void netcp_tx_irq_ack(struct net_device *ndev)
{
#ifdef EMAC_ARCH_HAS_INTERRUPT
	if (netcp_irq_enabled) {
		/* Ack Tx interrupt */
		if (__raw_readl(DEVICE_QM_INTD_BASE + QM_REG_INTD_STATUS0) &
		    (1 << DEVICE_QM_ETH_ACC_TX_CHANNEL)) {
			__raw_writel(1, QM_REG_INTD_COUNT_IRQ(DEVICE_QM_ETH_ACC_TX_CHANNEL));
			__raw_writel(QM_REG_INTD_EOI_HIGH_PRIO_INDEX + DEVICE_QM_ETH_ACC_TX_CHANNEL,
				     DEVICE_QM_INTD_BASE + QM_REG_INTD_EOI);
		}
	}
#endif
}

/*
 * Release transmitted packets
 */
static int netcp_tx(struct net_device *ndev)
{
	struct qm_host_desc *hd       = NULL;
	int                  released = 0;
#ifdef EMAC_ARCH_HAS_INTERRUPT
	static u32          *acc_list = 0;
	u32                 *acc_list_p;

	if ((__raw_readl(DEVICE_QM_INTD_BASE + QM_REG_INTD_STATUS0) &
	    (1 << DEVICE_QM_ETH_ACC_TX_CHANNEL)) == 0)
		return 0;

	/* Accumulator ping pong buffer management */
	if (acc_list == acc_list_addr_tx)
		acc_list = acc_list_addr_tx + (DEVICE_TX_INT_THRESHOLD + 1);
	else
		acc_list = acc_list_addr_tx;

	acc_list_p = acc_list;

/* With interrupt support: get the descriptors from the accumulator list */
#define	NETCP_TX_LOOP_ITERATOR()  ((struct qm_host_desc *) qm_desc_ptov(*acc_list_p++))
#else
/* Without interrupt support: get the descriptors directly from the queue */
#define NETCP_TX_LOOP_ITERATOR() hw_qm_queue_pop(DEVICE_QM_ETH_TX_CP_Q)
#endif

	/* Main loop */
	while ((hd = NETCP_TX_LOOP_ITERATOR()) && (hd != NULL)) {
		struct sk_buff *skb = (struct sk_buff*) hd->private;

		/* Release the attached sk_buff */
		if (skb) {
			DPRINTK("tx packet relaxed (skbuff=0x%x, hd=0x%x)\n",
				skb, hd);
			
			hd->private = 0;

			/* Free the skbuff associated to this packet */
			dev_kfree_skb_irq(skb);
		}

		/* Give back packets to free queue */
		hw_qm_queue_push(hd, DEVICE_QM_ETH_FREE_Q, DEVICE_QM_DESC_SIZE_BYTES);

		released++;
	}
	
	/* Check if the free queue is no more empty in case of tx queue stopped */
	if (unlikely(netif_queue_stopped(ndev) && released)) {
		ndev->trans_start = jiffies;
		netif_wake_queue(ndev);
	}

	return 1;
}

/*
 * Pop an incoming packet
 */
static int netcp_rx(struct net_device *ndev,
		     unsigned int *work_done,
		     unsigned int work_to_do)
{
	int                  pkt_size = 0;
	struct qm_host_desc *hd       = NULL;
	struct sk_buff      *skb;
	struct sk_buff      *skb_rcv;
#ifdef EMAC_ARCH_HAS_INTERRUPT
	static u32          *acc_list = 0;
	u32                 *acc_list_p;

	if ((__raw_readl(DEVICE_QM_INTD_BASE + QM_REG_INTD_STATUS0) &
	     (1 << DEVICE_QM_ETH_ACC_RX_CHANNEL)) == 0)
		return 0;

	/* Accumulator ping pong buffer management */
	if (acc_list == acc_list_addr_rx)
		acc_list = acc_list_addr_rx + (DEVICE_RX_INT_THRESHOLD + 1);
	else
		acc_list = acc_list_addr_rx;

	acc_list_p = acc_list;

/* With interrupt support: get the descriptors from the accumulator list */
#define	NETCP_RX_LOOP_ITERATOR() ((struct qm_host_desc *) qm_desc_ptov(*acc_list_p++))
#else
/* Without interrupt support: get the descriptors directly from the queue */
#define NETCP_RX_LOOP_ITERATOR() hw_qm_queue_pop(DEVICE_QM_ETH_RX_Q)
#endif

	/* Main loop */
	while ((hd = NETCP_RX_LOOP_ITERATOR()) && (hd != NULL)) {

		if (unlikely(work_done && *work_done >= work_to_do))
			return 0;

		pkt_size = QM_DESC_DINFO_GET_PKT_LEN(hd->desc_info);
		if (unlikely(pkt_size == 0))
			return 0;

		/* Get the incoming skbuff */
		skb_rcv = (struct sk_buff *) hd->private;

		DPRINTK("received a packet of len %d (skb=0x%x, hd=0x%x) buffer = 0x%x\n",
			pkt_size, skb_rcv, hd, hd->desc_info);

		/* Prepare the received sk_buff */
		skb_rcv->dev = ndev;
		skb_put(skb_rcv, pkt_size);
		skb_rcv->protocol = eth_type_trans(skb_rcv, ndev);
		
		/* Allocate a new skb */
		skb = netdev_alloc_skb_ip_align(ndev, NETCP_MAX_PACKET_SIZE);
		if (skb != NULL) {
			/* Attach the new one */
			hd->buff_len      = skb_tailroom(skb);
			hd->orig_buff_len = skb_tailroom(skb);
			hd->buff_ptr      = (u32)skb->data;
			hd->next_bdptr    = 0;
			hd->orig_buff_ptr = (u32)skb->data;
			hd->private       = (u32)skb;
			hw_qm_queue_push(hd, DEVICE_QM_ETH_RX_FREE_Q, DEVICE_QM_DESC_SIZE_BYTES);

			L2_cache_block_writeback_invalidate((u32) skb->data,
							    (u32) skb->data + NETCP_MAX_PACKET_SIZE);
		}
		
		/* Fill statistic */
		ndev->last_rx = jiffies;
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += pkt_size;
		
		/* Push to kernel received sk_buff */
		netif_receive_skb(skb_rcv);
		if (work_done)
			(*work_done)++;
	}
	return 1;
}

/*
 * Rx interrupt handling functions
 */
static irqreturn_t netcp_rx_interrupt(int irq, void *netdev_id)
{
	struct net_device         *ndev = (struct net_device *) netdev_id;
	struct netcp_priv *p    = netdev_priv(ndev);

	if (likely(napi_schedule_prep(&p->napi))) {
		__napi_schedule(&p->napi);
	}
	return IRQ_HANDLED;
}

/*
 * Tx interrupt handling functions
 */
static irqreturn_t netcp_tx_interrupt(int irq, void *netdev_id)
{
	struct net_device *ndev = (struct net_device *) netdev_id;

	netcp_tx(ndev);
	netcp_tx_irq_ack(ndev);

	return IRQ_HANDLED;
}

/*
 * NAPI poll
 */
static int netcp_poll(struct napi_struct *napi, int budget)
{
	struct netcp_priv *p = 
		container_of(napi, struct netcp_priv, napi);
	unsigned int work_done = 0;

	netcp_rx(p->ndev, &work_done, budget);
	
	/* If budget not fully consumed, exit the polling mode */
	if (work_done < budget) {
		napi_complete(napi);
		netcp_rx_irq_ack(p->ndev);
	}

	return work_done;
}

/*
 * Poll the device
 */
static void netcp_ndo_poll_controller(struct net_device *ndev)
{
	netcp_irq_disable(ndev);
	netcp_rx_interrupt(ndev->irq, ndev);
	netcp_tx_interrupt(ndev->irq, ndev);
	netcp_irq_enable(ndev);

	if (ndev->irq == -1) {
		mod_timer(&poll_timer,
			  jiffies + msecs_to_jiffies(DEVICE_POLLING_PERIOD_MSEC));
	}
}

/*
 * Push an outcoming packet
 */
static int netcp_ndo_start_xmit(struct sk_buff *skb,
				   struct net_device *ndev)
{
	int                  ret;
	unsigned             pkt_len = skb->len;
	struct qm_host_desc *hd;
	int                  loop = 0;

	if (unlikely(pkt_len < NETCP_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, NETCP_MIN_PACKET_SIZE);
		if (unlikely(ret < 0)) {
			printk(KERN_WARNING "%s: packet padding failed\n", ndev->name);
			netif_stop_queue(ndev);
			return NETDEV_TX_BUSY;
		}
		pkt_len = NETCP_MIN_PACKET_SIZE;
	}

	/* Spin a little bit if free queue is empty until we get free desc */
	while(((hd = hw_qm_queue_pop(DEVICE_QM_ETH_FREE_Q)) == NULL)
	      && (++loop < DEVICE_TX_LOOP_FREE_QUEUE));

	/* 
	 * If even while spinning we does not have new free desc, let's stop the 
         * tx queue and wait for a more long time using a timer timeout.
	 */
	if (hd == NULL) {
		DPRINTK("no free descriptor retrieved, loop = %d\n", loop);

		ndev->stats.tx_dropped++;
		ndev->trans_start = jiffies;

		/* Stop the queue */
		netif_stop_queue(ndev);

		return NETDEV_TX_BUSY;
	}

	L2_cache_block_writeback((u32) skb->data,
				 (u32) skb->data + pkt_len);

	QM_DESC_DINFO_SET_PKT_LEN(hd->desc_info, pkt_len);
	
	hd->buff_len		= pkt_len;
	hd->orig_buff_len	= pkt_len;
	hd->buff_ptr		= (u32) skb->data;
	hd->orig_buff_ptr	= (u32) skb->data;
	hd->private             = (u32) skb;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += pkt_len;
	ndev->trans_start     = jiffies;

	DPRINTK("transmitting packet of len %d (skb=0x%x, hd=0x%x)\n",
		skb->len, skb, hd);

	/* Return the descriptor back to the tx completion queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, DEVICE_QM_ETH_TX_CP_Q);
	
	hw_qm_queue_push(hd, DEVICE_QM_ETH_TX_Q, DEVICE_QM_DESC_SIZE_BYTES);

	return NETDEV_TX_OK;
}

/*
 * Change receive flags
 */
static void netcp_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}

/*
 * Open the device
 */
static int netcp_ndo_open(struct net_device *ndev)
{
	struct netcp_priv *p = netdev_priv(ndev);
	
	/* Start EMAC */
	cpmac_drv_start();

	netif_wake_queue(ndev);

	napi_enable(&p->napi);

	DPRINTK("starting interface\n");

	if (ndev->irq == -1) {
		/* Use timeout to poll */
		init_timer(&poll_timer);
		poll_timer.function = netcp_ndo_poll_controller;
		poll_timer.data     = (unsigned long) ndev;
		mod_timer(&poll_timer,
			  jiffies + msecs_to_jiffies(DEVICE_POLLING_PERIOD_MSEC));
	} else
		netcp_irq_enable(ndev);

	return 0;
}

/*
 * Close the device
 */
static int netcp_ndo_stop(struct net_device *ndev)
{
	struct netcp_priv *p = netdev_priv(ndev);

	netcp_irq_disable(ndev);

	napi_disable(&p->napi);
	netif_stop_queue(ndev);
	
	cpmac_drv_stop();

	return 0;
}

/*
 * Transmit timeout: if tx queue is stopped since a long time
 */
static void netcp_ndo_tx_timeout(struct net_device *ndev)
{
	printk(KERN_WARNING "%s: transmit timed out\n", ndev->name);
	ndev->stats.tx_errors++;
	ndev->trans_start = jiffies;
	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

/*
 * Standard MII ioctl()
 */
static int netcp_ndo_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	struct netcp_priv *priv = netdev_priv(ndev);

	if (!netif_running(ndev))
		return -EINVAL;
	
	return generic_mii_ioctl(&priv->mii, if_mii(req), cmd, NULL);
}

/*
 * MDIO management
 */
static int netcp_mdio_read(struct net_device *dev, int phy_id, int reg)
{	
	int val;
	int ack = 0;

	mdio_phy_wait();
	mdio_phy_read(reg, phy_id);
	mdio_phy_wait_res_ack(val, ack);

	if (!ack)
		val = -1;

	return val;
}

static void netcp_mdio_write(struct net_device *dev, int phy_id, int reg, int value)
{

	mdio_phy_wait();
	mdio_phy_write(reg, phy_id, value);
	mdio_phy_wait();
	
	return;
}

static void netcp_mdio_init(void)
{
	mdio_set_reg(MDIO_CONTROL, MDIO_B_ENABLE | (VBUSCLK & MDIO_M_CLKDIV));
}

/*
 * Ethtool management
 */
static int netcp_get_settings(struct net_device *ndev, struct ethtool_cmd *ecmd)
{
	struct netcp_priv *priv = netdev_priv(ndev);
	
	mii_ethtool_gset(&priv->mii, ecmd);
	
	return 0;
}

static int netcp_set_settings(struct net_device *ndev, struct ethtool_cmd *ecmd)
{
	struct netcp_priv *priv = netdev_priv(ndev);
	
	mii_ethtool_sset(&priv->mii, ecmd);
	
	return 0;
}

static int netcp_nway_reset(struct net_device *ndev)
{
	struct netcp_priv *priv = netdev_priv(ndev);

	return mii_nway_restart(&priv->mii);
}

static u32 netcp_get_link(struct net_device *ndev)
{
	struct netcp_priv *priv = netdev_priv(ndev);

	return mii_link_ok(&priv->mii);
}

static void netcp_get_drvinfo(struct net_device *ndev,
			      struct ethtool_drvinfo *info)
{
	struct netcp_priv *priv = netdev_priv(ndev);
	strcpy(info->driver, NETCP_DRIVER_NAME);
	strcpy(info->version, NETCP_DRIVER_VERSION);
	sprintf(info->fw_version, "%d", priv->data.pa_pdsp.firmware_version);
}

static u32 netcp_get_msglevel(struct net_device *ndev)
{
	struct netcp_priv *priv = netdev_priv(ndev);
	return priv->msg_enable;
}

static void netcp_set_msglevel(struct net_device *ndev, u32 value)
{
	struct netcp_priv *priv = netdev_priv(ndev);
	priv->msg_enable = value;
}

static int netcp_flash_device(struct net_device *netdev, struct ethtool_flash *efl)
{
	char                  file_name[ETHTOOL_FLASH_MAX_FILENAME];
	int                   pdsp;
	int                   res = 0;
	const struct firmware *fw;

	file_name[ETHTOOL_FLASH_MAX_FILENAME - 1] = 0;
	strcpy(file_name, efl->data);
	pdsp = efl->region;

	/* Request the firmware */
	res = request_firmware(&fw, file_name, &netdev->dev);
	if (res)
		return res;

	/* Stop the wanted PDSP, load the firmware then enable it */
	res = keystone_pa_disable(pdsp);
	if (res)
		goto error;
	
	res = keystone_pa_set_firmware(pdsp, (const unsigned int*) fw->data,
				       fw->size);
	if (res)
		goto error;
	
	return keystone_pa_enable(pdsp);
error:
	release_firmware(fw);
	return res;
}

/*
 * Statistic management
 */
struct netcp_ethtool_stat {
	char desc[ETH_GSTRING_LEN];
	int type;
	int size;
	int offset;
};

#define FIELDINFO(_struct, field)       FIELD_SIZEOF(_struct, field),	\
		                                offsetof(_struct, field)
#define CPSW_STATSA_INFO(field) 	"CPSW_A:"#field, CPSW_STATSA_MODULE,\
					FIELDINFO(struct keystone_cpsw_stats,\
						field)
#define CPSW_STATSB_INFO(field) 	"CPSW_B:"#field, CPSW_STATSB_MODULE,\
					FIELDINFO(struct keystone_cpsw_stats,\
						field)

static const struct netcp_ethtool_stat et_stats[] = {
	/* CPSW module A */
	{CPSW_STATSA_INFO(rx_good_frames)},
	{CPSW_STATSA_INFO(rx_broadcast_frames)},
	{CPSW_STATSA_INFO(rx_multicast_frames)},
	{CPSW_STATSA_INFO(rx_pause_frames)},
	{CPSW_STATSA_INFO(rx_crc_errors)},
	{CPSW_STATSA_INFO(rx_align_code_errors)},
	{CPSW_STATSA_INFO(rx_oversized_frames)},
	{CPSW_STATSA_INFO(rx_jabber_frames)},
	{CPSW_STATSA_INFO(rx_undersized_frames)},
	{CPSW_STATSA_INFO(rx_fragments)},
	{CPSW_STATSA_INFO(rx_bytes)},
	{CPSW_STATSA_INFO(tx_good_frames)},
	{CPSW_STATSA_INFO(tx_broadcast_frames)},
	{CPSW_STATSA_INFO(tx_multicast_frames)},
	{CPSW_STATSA_INFO(tx_pause_frames)},
	{CPSW_STATSA_INFO(tx_deferred_frames)},
	{CPSW_STATSA_INFO(tx_collision_frames)},
	{CPSW_STATSA_INFO(tx_single_coll_frames)},
	{CPSW_STATSA_INFO(tx_mult_coll_frames)},
	{CPSW_STATSA_INFO(tx_excessive_collisions)},
	{CPSW_STATSA_INFO(tx_late_collisions)},
	{CPSW_STATSA_INFO(tx_underrun)},
	{CPSW_STATSA_INFO(tx_carrier_senser_errors)},
	{CPSW_STATSA_INFO(tx_bytes)},
	{CPSW_STATSA_INFO(tx_64byte_frames)},
	{CPSW_STATSA_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSA_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSA_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSA_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSA_INFO(tx_1024byte_frames)},
	{CPSW_STATSA_INFO(net_bytes)},
	{CPSW_STATSA_INFO(rx_sof_overruns)},
	{CPSW_STATSA_INFO(rx_mof_overruns)},
	{CPSW_STATSA_INFO(rx_dma_overruns)},
	/* CPSW module B */
	{CPSW_STATSB_INFO(rx_good_frames)},
	{CPSW_STATSB_INFO(rx_broadcast_frames)},
	{CPSW_STATSB_INFO(rx_multicast_frames)},
	{CPSW_STATSB_INFO(rx_pause_frames)},
	{CPSW_STATSB_INFO(rx_crc_errors)},
	{CPSW_STATSB_INFO(rx_align_code_errors)},
	{CPSW_STATSB_INFO(rx_oversized_frames)},
	{CPSW_STATSB_INFO(rx_jabber_frames)},
	{CPSW_STATSB_INFO(rx_undersized_frames)},
	{CPSW_STATSB_INFO(rx_fragments)},
	{CPSW_STATSB_INFO(rx_bytes)},
	{CPSW_STATSB_INFO(tx_good_frames)},
	{CPSW_STATSB_INFO(tx_broadcast_frames)},
	{CPSW_STATSB_INFO(tx_multicast_frames)},
	{CPSW_STATSB_INFO(tx_pause_frames)},
	{CPSW_STATSB_INFO(tx_deferred_frames)},
	{CPSW_STATSB_INFO(tx_collision_frames)},
	{CPSW_STATSB_INFO(tx_single_coll_frames)},
	{CPSW_STATSB_INFO(tx_mult_coll_frames)},
	{CPSW_STATSB_INFO(tx_excessive_collisions)},
	{CPSW_STATSB_INFO(tx_late_collisions)},
	{CPSW_STATSB_INFO(tx_underrun)},
	{CPSW_STATSB_INFO(tx_carrier_senser_errors)},
	{CPSW_STATSB_INFO(tx_bytes)},
	{CPSW_STATSB_INFO(tx_64byte_frames)},
	{CPSW_STATSB_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSB_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSB_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSB_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSB_INFO(tx_1024byte_frames)},
	{CPSW_STATSB_INFO(net_bytes)},
	{CPSW_STATSB_INFO(rx_sof_overruns)},
	{CPSW_STATSB_INFO(rx_mof_overruns)},
	{CPSW_STATSB_INFO(rx_dma_overruns)},
};

#define ETHTOOL_STATS_NUM ARRAY_SIZE(et_stats)

static void netcp_get_stat_strings(struct net_device *netdev, uint32_t stringset,
				   uint8_t *data)
{
	int i;
	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
			memcpy(data, et_stats[i].desc, ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		break;
	}
}

static int netcp_get_sset_count(struct net_device *netdev, int stringset)
{
	switch (stringset) {
	case ETH_SS_TEST:
		return 0;
	case ETH_SS_STATS:
		return ETHTOOL_STATS_NUM;
	default:
		return -EINVAL;
	}
}

static void netcp_get_ethtool_stats(struct net_device *netdev,
				    struct ethtool_stats *stats, uint64_t *data)
{
	struct keystone_cpsw_stats *cpsw_statsa = cpsw_get_stats(CPSW_STATSA_MODULE);
	struct keystone_cpsw_stats *cpsw_statsb = cpsw_get_stats(CPSW_STATSB_MODULE);

	void *p = NULL;
	int i;

	for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
		switch (et_stats[i].type) {
		case CPSW_STATSA_MODULE:
			p = cpsw_statsa;
			break;
		case CPSW_STATSB_MODULE:
			p  = cpsw_statsb;
			break;
		}

		p = (u8 *)p + et_stats[i].offset;
		data[i] = (et_stats[i].size == sizeof(u64)) ?
			*(u64 *)p: *(u32 *)p;
	}

	return;
}

static const struct ethtool_ops netcp_ethtool_ops = {
	.get_settings      = netcp_get_settings,
	.set_settings      = netcp_set_settings,
	.get_drvinfo	   = netcp_get_drvinfo,
	.get_msglevel	   = netcp_get_msglevel,
	.set_msglevel	   = netcp_set_msglevel,
	.nway_reset	   = netcp_nway_reset,
	.get_link	   = netcp_get_link,
	.flash_device	   = netcp_flash_device,
	.get_strings       = netcp_get_stat_strings,
	.get_sset_count    = netcp_get_sset_count,
	.get_ethtool_stats = netcp_get_ethtool_stats,
};

static const struct net_device_ops keystone_netdev_ops = {
	.ndo_open		= netcp_ndo_open,
	.ndo_stop		= netcp_ndo_stop,
	.ndo_start_xmit		= netcp_ndo_start_xmit,
	.ndo_change_rx_flags	= netcp_ndo_change_rx_flags,
	.ndo_do_ioctl           = netcp_ndo_ioctl,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= netcp_ndo_tx_timeout,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= netcp_ndo_poll_controller,
#endif
};

static int __devinit netcp_probe(struct platform_device *pdev)
{
	struct netcp_platform_data *data = pdev->dev.platform_data;
	struct net_device	   *ndev;
	struct netcp_priv          *priv;
	int                         i, ret = 0;
#ifdef EMAC_ARCH_HAS_MAC_ADDR
	u8			    hw_emac_addr[6];
#endif
	void __iomem               *pa_cdma_base_addr;
	struct pktdma_rx_cfg	    c_rx_cfg;
	struct pktdma_tx_cfg	    c_tx_cfg;
	struct pktdma_rx_cfg	   *rx_cfg = &c_rx_cfg;
	struct pktdma_tx_cfg	   *tx_cfg = &c_tx_cfg;
	const struct firmware      *fw;
	u32                         acc_list_num_rx;
	u32                         acc_list_num_tx;
	u32                         acc_list_size; 
	u32                         queue_free_buf[DEVICE_PA_CDMA_RX_NUM_FLOWS];
	u32                         queue_rx[DEVICE_PA_CDMA_RX_NUM_FLOWS];

	if (!data) {
		pr_err("KeyStone NetCP: platform data missing\n");
		return -ENODEV;
	}

	ndev = alloc_etherdev(sizeof(struct netcp_priv));
	if (!ndev) {
		pr_err("KeyStone NetCP: error allocating net_device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ndev);
	priv = netdev_priv(ndev);
	spin_lock_init(&priv->lock);
	priv->data = *data;
	priv->pdev = pdev;
	priv->ndev = ndev;
	priv->dev  = &ndev->dev;
	priv->msg_enable = netif_msg_init(debug_level, NETCP_DEBUG);
	priv->rx_packet_max = max(rx_packet_max, 128);

	/* MII/MDIO init */
	priv->msg_enable        = NETIF_MSG_LINK;
	priv->mii.phy_id        = data->phy_id;
	priv->mii.phy_id_mask   = 0x1f;
	priv->mii.reg_num_mask  = 0x1f;
	priv->mii.force_media   = 0;
	priv->mii.full_duplex   = 0;
	priv->mii.supports_gmii = 1;
	priv->mii.dev	        = ndev;
	priv->mii.mdio_read     = netcp_mdio_read;
	priv->mii.mdio_write    = netcp_mdio_write;

	netcp_mdio_init();

	pktdma_region_init(DEVICE_PA_CDMA_BASE,
			   DEVICE_PA_CDMA_SIZE,
			   &pa_cdma_base_addr);

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

	/* Configure Rx and Tx PKTDMA */
	rx_cfg->base_addr        = (u32) pa_cdma_base_addr;
	rx_cfg->rx_base_offset   = DEVICE_PA_CDMA_RX_CHAN_CFG_OFFSET;
	rx_cfg->rx_chan          = DEVICE_PA_CDMA_RX_FIRST_CHANNEL;
	rx_cfg->n_rx_chans       = DEVICE_PA_CDMA_RX_NUM_CHANNELS;
	rx_cfg->flow_base_offset = DEVICE_PA_CDMA_RX_FLOW_CFG_OFFSET;
	rx_cfg->rx_flow          = DEVICE_PA_CDMA_RX_FIRST_FLOW;
	rx_cfg->n_rx_flows       = DEVICE_PA_CDMA_RX_NUM_FLOWS;
	rx_cfg->qmnum_free_buf   = 0;
	rx_cfg->queue_free_buf   = &queue_free_buf[0];
	rx_cfg->qmnum_rx         = 0;
	rx_cfg->queue_rx         = &queue_rx[0];
	rx_cfg->tdown_poll_count = DEVICE_RX_CDMA_TIMEOUT_COUNT;
#ifndef EMAC_ARCH_HAS_INTERRUPT
	rx_cfg->use_acc          = 0;
#else
	rx_cfg->use_acc          = 1;
	rx_cfg->acc_threshold    = DEVICE_RX_INT_THRESHOLD;
	rx_cfg->acc_channel      = DEVICE_QM_ETH_ACC_RX_CHANNEL;

	queue_free_buf[0]        = DEVICE_QM_ETH_RX_FREE_Q;
	queue_rx[0]              = DEVICE_QM_ETH_RX_Q;

	/* Set the accumulator list memory in the descriptor memory */
	acc_list_num_rx = ((rx_cfg->acc_threshold + 1) * 2);
	acc_list_num_tx = ((tx_cfg->acc_threshold + 1) * 2);
	acc_list_size   =  (acc_list_num_rx + acc_list_num_tx) * sizeof(long);

	acc_list_addr_rx = (u32*) qm_mem_alloc(acc_list_size,
					       (u32*)&acc_list_phys_addr_rx);
	if (acc_list_addr_rx == NULL) {
		printk(KERN_ERR "%s: accumulator memory allocation failed\n",
		       __FUNCTION__);
		return -ENOMEM;
	}
	memset((void *) acc_list_addr_rx, 0, acc_list_size);

	acc_list_addr_tx      = acc_list_addr_rx + acc_list_num_rx;
	acc_list_phys_addr_tx = acc_list_phys_addr_rx + acc_list_num_rx;
	
	rx_cfg->acc_list_addr      = (u32) acc_list_addr_rx;
	rx_cfg->acc_list_phys_addr = (u32) acc_list_phys_addr_rx;
#endif

	pktdma_rx_config(rx_cfg);

	tx_cfg->base_addr               = (u32) pa_cdma_base_addr;
	tx_cfg->gbl_ctl_base_offset     = DEVICE_PA_CDMA_GLOBAL_CFG_OFFSET;
	tx_cfg->tx_base_offset          = DEVICE_PA_CDMA_TX_CHAN_CFG_OFFSET;
	tx_cfg->tx_chan                 = DEVICE_PA_CDMA_TX_FIRST_CHANNEL;
	tx_cfg->n_tx_chans		= DEVICE_PA_CDMA_TX_NUM_CHANNELS;
	tx_cfg->queue_tx		= DEVICE_QM_ETH_TX_CP_Q;
#ifndef EMAC_ARCH_HAS_INTERRUPT
	tx_cfg->use_acc			= 0;
#else
	tx_cfg->use_acc			= 1;
	tx_cfg->acc_threshold		= DEVICE_TX_INT_THRESHOLD;
	tx_cfg->acc_channel		= DEVICE_QM_ETH_ACC_TX_CHANNEL;
	
	tx_cfg->acc_list_addr      = (u32) acc_list_addr_tx;
	tx_cfg->acc_list_phys_addr = (u32) acc_list_phys_addr_tx;
#endif

	pktdma_tx_config(tx_cfg);

	/* Initialize QMSS/PKTDMA queues */
	netcp_cleanup_qs(ndev);
	netcp_init_qs(ndev);

	cpmac_init();
	/* Stop EMAC */
	cpmac_drv_stop();

	/* Streaming switch configuration */
	streaming_switch_setup();

	/* Reset the PA */
	ret = keystone_pa_reset();
	if (ret != 0) {
		printk(KERN_ERR "%s: PA reset failed d\n", __FUNCTION__);
		return ret;
	}

	/* Request PA firmware */
	ret = request_firmware(&fw, data->pa_pdsp.firmware, &pdev->dev);
	if (ret != 0) {
		printk(KERN_ERR "PA: Cannot find  %s firmware\n", data->pa_pdsp.firmware);
		return ret;
	}

	/* Configure the PA */
	ret = keystone_pa_config(data->pa_pdsp.pdsp, (const unsigned int*) fw->data,
				 fw->size, ndev->dev_addr);
	if (ret != 0) {
		printk(KERN_ERR "%s: PA init failed\n", __FUNCTION__);
		return ret;
	}

	release_firmware(fw);

	/* Setup Ethernet driver function */
	ether_setup(ndev);

	/* NAPI register */
	netif_napi_add(ndev, &priv->napi, netcp_poll, DEVICE_NAPI_WEIGHT);

#ifdef EMAC_ARCH_HAS_INTERRUPT
	/* Register interrupts */
	ret = request_irq(data->rx_irq, netcp_rx_interrupt, 0, ndev->name, ndev);
	if (ret == 0) {
		ret = request_irq(data->tx_irq, netcp_tx_interrupt, 0, ndev->name, ndev);
		if (ret != 0) {
			printk(KERN_ERR "%s: irq %d register failed (%d)\n",
			       __FUNCTION__, data->tx_irq, ret);
			return -EINVAL;
		}
		ndev->irq = data->rx_irq;
	} else {
		printk(KERN_WARNING "%s: irq mode failed (%d), use polling\n", 
		       __FUNCTION__, ret);
		ndev->irq = -1;
	}

#else
	ndev->irq = -1;
#endif

	/* Register the network device */
	ndev->dev_id         = 0;
	ndev->watchdog_timeo = DEVICE_TX_TIMEOUT;
	ndev->netdev_ops     = &keystone_netdev_ops;

	SET_ETHTOOL_OPS(ndev, &netcp_ethtool_ops);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = register_netdev(ndev);
	if (ret) {
		printk(KERN_ERR "%s: error registering net device\n",__FUNCTION__);
		ret = -ENODEV;
		goto clean_ndev_ret;
	}

	printk("%s: %s %s\n",
	       ndev->name, NETCP_DRIVER_NAME, NETCP_DRIVER_VERSION);

	return 0;

clean_ndev_ret:
	free_netdev(ndev);
	return ret;
}

static int __devexit netcp_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	
	platform_set_drvdata(pdev, NULL);

	free_netdev(ndev);

	return 0;
}

static int netcp_suspend(struct device *dev)
{
	return 0;
}

static int netcp_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops netcp_pm_ops = {
	.suspend	= netcp_suspend,
	.resume		= netcp_resume,
};

static struct platform_driver netcp_driver = {
	.driver = {
		.name	 = "keystone_netcp",
		.owner	 = THIS_MODULE,
		.pm	 = &netcp_pm_ops,
	},
	.probe = netcp_probe,
	.remove = __devexit_p(netcp_remove),
};

static int __init netcp_init(void)
{
	return platform_driver_register(&netcp_driver);
}
module_init(netcp_init);

static void __exit netcp_exit(void)
{
	platform_driver_unregister(&netcp_driver);
}
module_exit(netcp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone NetCP driver");
