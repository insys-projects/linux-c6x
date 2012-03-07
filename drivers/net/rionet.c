/*
 * rionet - Ethernet driver over RapidIO messaging services
 *
 * Copyright 2005 MontaVista Software, Inc.
 * Matt Porter <mporter@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/slab.h>
#include <linux/rio_ids.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>

#define DRV_NAME        "rionet"
#define DRV_VERSION     "0.2"
#define DRV_AUTHOR      "Matt Porter <mporter@kernel.crashing.org>"
#define DRV_DESC        "Ethernet over RapidIO"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

#define RIONET_DEFAULT_MSGLEVEL \
			(NETIF_MSG_DRV          | \
			 NETIF_MSG_LINK         | \
			 NETIF_MSG_RX_ERR       | \
			 NETIF_MSG_TX_ERR)

#define RIONET_DOORBELL_JOIN	0x1000
#define RIONET_DOORBELL_LEAVE	0x1001

#define RIONET_MAILBOX		0

#define RIONET_TX_RING_SIZE	CONFIG_RIONET_TX_SIZE
#define RIONET_RX_RING_SIZE	CONFIG_RIONET_RX_SIZE

#if defined(CONFIG_RAPIDIO_TCI648X) || defined(CONFIG_TI_KEYSTONE_RAPIDIO)
#include <asm/rio.h>
#define RIONET_MSG_SIZE         MACH_RIO_MAX_MSG_SIZE
#else 
#define RIONET_MSG_SIZE         RIO_MAX_MSG_SIZE
#endif

static LIST_HEAD(rionet_peers);
static LIST_HEAD(ndev_list);

struct rionet_private {
	struct rio_mport *mport;
	struct sk_buff *rx_skb[RIONET_RX_RING_SIZE];
	struct sk_buff *tx_skb[RIONET_TX_RING_SIZE];
	int rx_slot;
	int tx_slot;
	int tx_cnt;
	int ack_slot;
	spinlock_t lock;
	spinlock_t tx_lock;
	u32 msg_enable;
};

struct rionet_peer {
	struct list_head node;
	struct rio_dev *rdev;
	struct resource *res;
};

struct rionet_ndev {
	struct list_head node;
	struct net_device *ndev;
	struct rio_mport *mport;
};

static int rionet_check = 0;
static int rionet_capable = 1;

/*
 * This is a fast lookup table for translating TX
 * Ethernet packets into a destination RIO device. It
 * could be made into a hash table to save memory depending
 * on system trade-offs.
 */
static struct rio_dev **rionet_active;

#define is_rionet_capable(pef, src_ops, dst_ops)		\
			((pef & RIO_PEF_INB_MBOX) &&		\
			 (pef & RIO_PEF_INB_DOORBELL) &&	\
			 (src_ops & RIO_SRC_OPS_DOORBELL) &&	\
			 (dst_ops & RIO_DST_OPS_DOORBELL))
#define dev_rionet_capable(dev) \
	is_rionet_capable(dev->pef, dev->src_ops, dev->dst_ops)

#define RIONET_MAC_MATCH(x)	(((x)[0] == 0x00) \
                              && ((x)[1] == 0x01) \
			      && ((x)[2] == 0x00) \
			      && ((x)[3] == 0x01))
#define RIONET_GET_DESTID(x)	(((x)[4] << 8) | ((x)[5]))

static int rionet_rx_clean(struct net_device *ndev)
{
	int i;
	int count = 0;
	int error = 0;
	struct rionet_private *rnet = netdev_priv(ndev);
	void *data;

	i = rnet->rx_slot;

	do {
		if (!rnet->rx_skb[i])
			continue;

		if (!(data = rio_get_inb_message(rnet->mport, RIONET_MAILBOX)))
			break;

		rnet->rx_skb[i]->data = data;
		skb_put(rnet->rx_skb[i], RIONET_MSG_SIZE);
		rnet->rx_skb[i]->protocol =
		    eth_type_trans(rnet->rx_skb[i], ndev);
		error = netif_rx(rnet->rx_skb[i]);

		if (error == NET_RX_DROP) {
			ndev->stats.rx_dropped++;
		} else {
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += RIONET_MSG_SIZE;
		}
		count++;
	} while ((i = (i + 1) % RIONET_RX_RING_SIZE) != rnet->rx_slot);

	return count;
}

static void rionet_rx_fill(struct net_device *ndev, int count)
{
	int i;
	struct rionet_private *rnet = netdev_priv(ndev);

	i = rnet->rx_slot;
	do {
		rnet->rx_skb[i] = netdev_alloc_skb_ip_align(ndev, RIONET_MSG_SIZE);
		if (!rnet->rx_skb[i])
			break;

		rio_add_inb_buffer(rnet->mport, RIONET_MAILBOX,
				   rnet->rx_skb[i]->data);
	} while ((i = (i + 1) % RIONET_RX_RING_SIZE) !=
		 ((rnet->rx_slot + count) % RIONET_RX_RING_SIZE));

	rnet->rx_slot = i;
}

static int rionet_queue_tx_msg(struct sk_buff *skb, struct net_device *ndev,
			       struct rio_dev *rdev)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	rio_add_outb_message(rnet->mport, rdev, 0, skb->data, skb->len);
	rnet->tx_skb[rnet->tx_slot] = skb;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	if (++rnet->tx_cnt == RIONET_TX_RING_SIZE)
		netif_stop_queue(ndev);

	++rnet->tx_slot;
	rnet->tx_slot &= (RIONET_TX_RING_SIZE - 1);

	if (netif_msg_tx_queued(rnet))
		printk(KERN_INFO "%s: queued skb %8.8x len %8.8x\n", DRV_NAME,
		       (u32) skb, skb->len);

	return 0;
}

static int rionet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	int i;
	struct rionet_private *rnet = netdev_priv(ndev);
	struct ethhdr *eth = (struct ethhdr *)skb->data;
	u16 destid;
	unsigned long flags;

	local_irq_save(flags);
	if (!spin_trylock(&rnet->tx_lock)) {
		local_irq_restore(flags);
		return NETDEV_TX_LOCKED;
	}

	if ((rnet->tx_cnt + 1) > RIONET_TX_RING_SIZE) {
		netif_stop_queue(ndev);
		spin_unlock_irqrestore(&rnet->tx_lock, flags);
		printk(KERN_ERR "%s: BUG! Tx Ring full when queue awake!\n",
		       ndev->name);
		return NETDEV_TX_BUSY;
	}

	if (eth->h_dest[0] & 0x01) {
		for (i = 0; i < RIO_MAX_ROUTE_ENTRIES(rnet->mport->sys_size);
		     i++)
			if (rionet_active[i])
				rionet_queue_tx_msg(skb, ndev,
						    rionet_active[i]);
	} else if (RIONET_MAC_MATCH(eth->h_dest)) {
		destid = RIONET_GET_DESTID(eth->h_dest);
		if (rionet_active[destid])
			rionet_queue_tx_msg(skb, ndev, rionet_active[destid]);
	}

	spin_unlock_irqrestore(&rnet->tx_lock, flags);

	return NETDEV_TX_OK;
}

#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
static void rionet_dbell_event(struct rio_mport *mport, void *dev_id, u16 sid, u16 tid,
			       u16 info)
{
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = netdev_priv(ndev);
	struct rionet_peer *peer;

	if (netif_msg_intr(rnet))
		printk(KERN_INFO "%s: doorbell sid %4.4x tid %4.4x info %4.4x",
		       DRV_NAME, sid, tid, info);
	if (info == RIONET_DOORBELL_JOIN) {
		if (!rionet_active[sid]) {
			list_for_each_entry(peer, &rionet_peers, node) {
				if (peer->rdev->destid == sid)
					rionet_active[sid] = peer->rdev;
			}
			rio_mport_send_doorbell(mport, sid,
						RIONET_DOORBELL_JOIN);
		}
	} else if (info == RIONET_DOORBELL_LEAVE) {
		rionet_active[sid] = NULL;
	} else {
		if (netif_msg_intr(rnet))
			printk(KERN_WARNING "%s: unhandled doorbell\n",
			       DRV_NAME);
	}
}
#endif

static void rionet_inb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int slot)
{
	int n;
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = netdev_priv(ndev);

	if (netif_msg_intr(rnet))
		printk(KERN_INFO "%s: inbound message event, mbox %d slot %d\n",
		       DRV_NAME, mbox, slot);

	spin_lock(&rnet->lock);
	if ((n = rionet_rx_clean(ndev)) != 0)
		rionet_rx_fill(ndev, n);
	spin_unlock(&rnet->lock);
}

static void rionet_outb_msg_event(struct rio_mport *mport, void *dev_id, int mbox, int slot)
{
	struct net_device *ndev = dev_id;
	struct rionet_private *rnet = netdev_priv(ndev);

	spin_lock(&rnet->lock);

	if (netif_msg_intr(rnet))
		printk(KERN_INFO
		       "%s: outbound message event, mbox %d slot %d\n",
		       DRV_NAME, mbox, slot);

	while (rnet->tx_cnt && (rnet->ack_slot != slot)) {
		/* dma unmap single */
		dev_kfree_skb_irq(rnet->tx_skb[rnet->ack_slot]);
		rnet->tx_skb[rnet->ack_slot] = NULL;
		++rnet->ack_slot;
		rnet->ack_slot &= (RIONET_TX_RING_SIZE - 1);
		rnet->tx_cnt--;
	}

	if (rnet->tx_cnt < RIONET_TX_RING_SIZE)
		netif_wake_queue(ndev);

	spin_unlock(&rnet->lock);
}

static int rionet_open(struct net_device *ndev)
{
	int i, rc = 0;
	struct rionet_peer *peer, *tmp;
	u32 pwdcsr;
	struct rionet_private *rnet = netdev_priv(ndev);

	if (netif_msg_ifup(rnet))
		printk(KERN_INFO "%s: open\n", DRV_NAME);

#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
	if ((rc = rio_request_inb_dbell(rnet->mport,
					(void *)ndev,
					RIONET_DOORBELL_JOIN,
					RIONET_DOORBELL_LEAVE,
					rionet_dbell_event)) < 0)
		goto out;
#endif
	if ((rc = rio_request_inb_mbox(rnet->mport,
				       (void *)ndev,
				       RIONET_MAILBOX,
				       RIONET_RX_RING_SIZE,
				       rionet_inb_msg_event)) < 0)
		goto out;

	if ((rc = rio_request_outb_mbox(rnet->mport,
					(void *)ndev,
					RIONET_MAILBOX,
					RIONET_TX_RING_SIZE,
					rionet_outb_msg_event)) < 0)
		goto out;

	/* Initialize inbound message ring */
	for (i = 0; i < RIONET_RX_RING_SIZE; i++)
		rnet->rx_skb[i] = NULL;
	rnet->rx_slot = 0;
	rionet_rx_fill(ndev, RIONET_RX_RING_SIZE);

	rnet->tx_slot = 0;
	rnet->tx_cnt = 0;
	rnet->ack_slot = 0;

	netif_carrier_on(ndev);
	netif_start_queue(ndev);

	list_for_each_entry_safe(peer, tmp, &rionet_peers, node) {
#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
		if (!(peer->res = rio_request_outb_dbell(peer->rdev,
							 RIONET_DOORBELL_JOIN,
							 RIONET_DOORBELL_LEAVE)))
		{
			printk(KERN_ERR "%s: error requesting doorbells\n",
			       DRV_NAME);
			continue;
		}

		/*
		 * If device has initialized inbound doorbells,
		 * send a join message
		 */
		rio_read_config_32(peer->rdev, RIO_WRITE_PORT_CSR, &pwdcsr);
		if (pwdcsr & RIO_DOORBELL_AVAIL)
			rio_send_doorbell(peer->rdev, RIONET_DOORBELL_JOIN);
#else
		/* Read configuration again (it may have changed) */
		rio_read_config_32(peer->rdev, RIO_PEF_CAR, &peer->rdev->pef);
		rio_read_config_32(peer->rdev, RIO_SRC_OPS_CAR, &peer->rdev->src_ops);
		rio_read_config_32(peer->rdev, RIO_DST_OPS_CAR, &peer->rdev->dst_ops);

#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
		/* Hack for adding INB_MBOX and INB_DOORBELL on KeyStone devices */
		peer->rdev->pef |= RIO_PEF_INB_DOORBELL | RIO_PEF_INB_MBOX;
#endif
		if (dev_rionet_capable(peer->rdev)) {
			rionet_active[peer->rdev->destid] = peer->rdev;
		} 
#endif
	}

      out:
	return rc;
}

static int rionet_close(struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);
	struct rionet_peer *peer, *tmp;
	int i;

	if (netif_msg_ifup(rnet))
		printk(KERN_INFO "%s: close\n", DRV_NAME);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	for (i = 0; i < RIONET_RX_RING_SIZE; i++)
		if (rnet->rx_skb[i])
			kfree_skb(rnet->rx_skb[i]);

	list_for_each_entry_safe(peer, tmp, &rionet_peers, node) {
		if (rionet_active[peer->rdev->destid]) {
#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
			rio_send_doorbell(peer->rdev, RIONET_DOORBELL_LEAVE);
#endif
			rionet_active[peer->rdev->destid] = NULL;
		}
#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
		rio_release_outb_dbell(peer->rdev, peer->res);
#endif
	}

#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
	rio_release_inb_dbell(rnet->mport, RIONET_DOORBELL_JOIN,
			      RIONET_DOORBELL_LEAVE);
#endif
	rio_release_inb_mbox(rnet->mport, RIONET_MAILBOX);
	rio_release_outb_mbox(rnet->mport, RIONET_MAILBOX);

	return 0;
}

static void rionet_remove(struct rio_dev *rdev)
{
	struct rionet_ndev *ndev_entry, *tmp;
	struct net_device  *ndev = NULL;
	struct rionet_peer *peer, *tmp2;

	if ((!rdev) || (!rdev->net))
		goto out;

	free_pages((unsigned long)rionet_active, rdev->net->hport->sys_size ?
		   ilog2(sizeof(void *)) + 4 : 0);

	list_for_each_entry_safe(ndev_entry, tmp, &ndev_list, node) {
		if (ndev_entry->mport == rdev->net->hport)
			ndev = ndev_entry->ndev;
	}

	if (ndev) {
		unregister_netdev(ndev);
		kfree(ndev);
	}
out:
	list_for_each_entry_safe(peer, tmp2, &rionet_peers, node) {
		list_del(&peer->node);
		kfree(peer);
	}
}

static void rionet_get_drvinfo(struct net_device *ndev,
			       struct ethtool_drvinfo *info)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->fw_version, "n/a");
	strcpy(info->bus_info, rnet->mport->name);
}

static u32 rionet_get_msglevel(struct net_device *ndev)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	return rnet->msg_enable;
}

static void rionet_set_msglevel(struct net_device *ndev, u32 value)
{
	struct rionet_private *rnet = netdev_priv(ndev);

	rnet->msg_enable = value;
}

static const struct ethtool_ops rionet_ethtool_ops = {
	.get_drvinfo = rionet_get_drvinfo,
	.get_msglevel = rionet_get_msglevel,
	.set_msglevel = rionet_set_msglevel,
	.get_link = ethtool_op_get_link,
};

static const struct net_device_ops rionet_netdev_ops = {
	.ndo_open		= rionet_open,
	.ndo_stop		= rionet_close,
	.ndo_start_xmit		= rionet_start_xmit,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};

static int rionet_setup_netdev(struct rio_mport *mport)
{
	int rc = 0;
	struct net_device *ndev = NULL;
	struct rionet_private *rnet;
	struct rionet_ndev *ndev_entry;
	u16 device_id;

	/* Allocate our net_device structure */
	ndev = alloc_etherdev(sizeof(struct rionet_private));
	if (ndev == NULL) {
		printk(KERN_INFO "%s: could not allocate ethernet device.\n",
		       DRV_NAME);
		rc = -ENOMEM;
		goto out;
	}

	rionet_active = (struct rio_dev **)__get_free_pages(GFP_KERNEL,
			mport->sys_size ? ilog2(sizeof(void *)) + 4 : 0);
	if (!rionet_active) {
		rc = -ENOMEM;
		goto out;
	}
	memset((void *)rionet_active, 0, sizeof(void *) *
				RIO_MAX_ROUTE_ENTRIES(mport->sys_size));

	/* Set up private area */
	rnet = netdev_priv(ndev);
	rnet->mport = mport;

	/* Set the default MAC address */
	device_id = rio_local_get_device_id(mport);
	ndev->dev_addr[0] = 0x00;
	ndev->dev_addr[1] = 0x01;
	ndev->dev_addr[2] = 0x00;
	ndev->dev_addr[3] = 0x01;
	ndev->dev_addr[4] = device_id >> 8;
	ndev->dev_addr[5] = device_id & 0xff;

	ndev->netdev_ops = &rionet_netdev_ops;
#if defined(CONFIG_RAPIDIO_TCI648X) || defined(CONFIG_TI_KEYSTONE_RAPIDIO)
	ndev->mtu = ETH_FRAME_LEN - 14;
#else
	ndev->mtu = RIONET_MSG_SIZE - 14;
#endif
	ndev->features = NETIF_F_LLTX;
	SET_ETHTOOL_OPS(ndev, &rionet_ethtool_ops);

	spin_lock_init(&rnet->lock);
	spin_lock_init(&rnet->tx_lock);

	rnet->msg_enable = RIONET_DEFAULT_MSGLEVEL;

	rc = register_netdev(ndev);
	if (rc != 0)
		goto out;

	if (!(ndev_entry = kmalloc(sizeof(struct rionet_ndev), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto out;
	}

	ndev_entry->ndev  = ndev;
	ndev_entry->mport = mport;
	list_add_tail(&ndev_entry->node, &ndev_list);

	printk("%s: %s %s Version %s, MAC %pM\n",
	       ndev->name,
	       DRV_NAME,
	       DRV_DESC,
	       DRV_VERSION,
	       ndev->dev_addr);

      out:
	return rc;
}

/*
 * XXX Make multi-net safe
 */
static int rionet_probe(struct rio_dev *rdev, const struct rio_device_id *id)
{
	int rc = -ENODEV;
	u32 lpef, lsrc_ops, ldst_ops;
	struct rionet_peer *peer;

	/* If local device is not rionet capable, give up quickly */
	if (!rionet_capable)
		goto out;

	/*
	 * First time through, make sure local device is rionet
	 * capable, setup netdev, and set flags so this is skipped
	 * on later probes
	 */
	if (!rionet_check) {
		rio_local_read_config_32(rdev->net->hport, RIO_PEF_CAR, &lpef);
		rio_local_read_config_32(rdev->net->hport, RIO_SRC_OPS_CAR,
					 &lsrc_ops);
		rio_local_read_config_32(rdev->net->hport, RIO_DST_OPS_CAR,
					 &ldst_ops);
		if (!is_rionet_capable(lpef, lsrc_ops, ldst_ops)) {
			printk(KERN_ERR
			       "%s: local device is not network capable\n",
			       DRV_NAME);
			rionet_check = 1;
			rionet_capable = 0;
			goto out;
		}

		rc = rionet_setup_netdev(rdev->net->hport);
		rionet_check = 1;
	}

#if !defined(CONFIG_RAPIDIO_TCI648X) && !defined(CONFIG_TI_KEYSTONE_RAPIDIO)
	/*
	 * If the remote device has mailbox/doorbell capabilities,
	 * add it to the peer list.
	 */
	if (dev_rionet_capable(rdev)) {
		if (!(peer = kmalloc(sizeof(struct rionet_peer), GFP_KERNEL))) {
			rc = -ENOMEM;
			goto out;
		}
		peer->rdev = rdev;
		list_add_tail(&peer->node, &rionet_peers);
	}
#else
	if (!(rdev->pef & RIO_PEF_SWITCH)) {
		if (!(peer = kmalloc(sizeof(struct rionet_peer), GFP_KERNEL))) {
			rc = -ENOMEM;
			goto out;
		}
		peer->rdev = rdev;

		printk("Using %s (vid %4.4x did %4.4x)\n",
		       rio_name(rdev), rdev->vid, rdev->did);

		list_add_tail(&peer->node, &rionet_peers);
	}
#endif

      out:
	return rc;
}

static struct rio_device_id rionet_id_table[] = {
	{RIO_DEVICE(RIO_ANY_ID, RIO_ANY_ID)}
};

static struct rio_driver rionet_driver = {
	.name = "rionet",
	.id_table = rionet_id_table,
	.probe = rionet_probe,
	.remove = rionet_remove,
};

static int __init rionet_init(void)
{
	return rio_register_driver(&rionet_driver);
}

static void __exit rionet_exit(void)
{
	rio_unregister_driver(&rionet_driver);
}

module_init(rionet_init);
module_exit(rionet_exit);
