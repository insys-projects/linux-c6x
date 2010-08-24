/*
 *  linux/arch/c6x/kernel/emac.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/emac.h>
#include <asm/mdio.h>

#undef EMAC_DEBUG

extern int mdio_init(void);

static struct timer_list emac_timer;
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

static int __init get_mac_addr(char *str)
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

__setup("emac_addr=", get_mac_addr);

/*
 * Get the device statistic
 */
static struct net_device_stats *emac_get_stats(struct net_device *dev)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;
	unsigned int reg;
	unsigned int dummy;
	int i;

	emac_set_stat(ep->stats.multicast, EMAC_RXMCASTFRAMES);
	emac_set_stat(ep->stats.collisions, EMAC_TXCOLLISION);
	emac_set_stat(ep->stats.rx_length_errors, EMAC_RXOVERSIZED);
	emac_set_stat(ep->stats.rx_length_errors, EMAC_RXUNDERSIZED);
	emac_set_stat(ep->stats.rx_crc_errors, EMAC_RXCRCERRORS);
	emac_set_stat(ep->stats.rx_frame_errors, EMAC_RXALIGNCODEERRORS);
	emac_set_stat(ep->stats.tx_carrier_errors, EMAC_TXCARRIERSLOSS);
	emac_set_stat(ep->stats.tx_fifo_errors, EMAC_TXUNDERRUN);
	emac_set_stat(ep->stats.tx_window_errors, EMAC_TXLATECOLL);

	/* ACK statistic by write-to-decrement */
	reg = EMAC_RXGOODFRAMES;
	for (i = 0; i < EMAC_NUM_STATREGS; i++) {
		dummy = emac_get_reg(reg);
		emac_set_reg(reg, dummy);
		reg += 4;
	}

	return &ep->stats;
}

/*
 * Receive packets
 */
static int emac_rx(struct net_device *dev, struct emac_desc *desc_ack)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;
	volatile struct emac_desc *desc = ep->cur_rx;
	ushort pkt_len;
	u32 pkt_flags;
	int loop = 1;

	while(loop) {
		struct sk_buff *skb;

		pkt_len   = desc->packet_flags_len & 0xffff;
		pkt_flags = desc->packet_flags_len & 0xffff0000;

		skb = dev_alloc_skb(pkt_len + 2);
		if (skb == NULL) {
			printk(KERN_WARNING "%s: Memory squeeze, dropping packet.\n", dev->name);
			ep->stats.rx_dropped++;
		} else {
			/*
			 * Copy the buffer in a skbuff and send it
			 * to the local descriptor queue.
			 */
			skb_reserve(skb, 2);   /* 16 bit alignment */

			skb->dev = dev;
			skb_put(skb, pkt_len);

			/* No coherency is assumed between EDMA and L2 cache */
			L2_cache_block_invalidate((u32) desc->buff,
						  (u32) desc->buff + pkt_len);

			eth_copy_and_sum(skb, desc->buff, pkt_len, 0);

			skb->protocol = eth_type_trans(skb, dev);
#ifdef EMAC_DEBUG
			printk("%s: receiving packet of %d len, proto 0x%x\n",
			       dev->name, pkt_len, skb->protocol);
#endif
			netif_rx(skb);
			dev->last_rx = jiffies;

			/* Fill statistic */
			ep->stats.rx_packets++;
			ep->stats.rx_bytes += pkt_len;
		}

		/* Descriptor is now available */
		desc->buff_offset_len = ep->packet_mtu;
		desc->packet_flags_len = EMAC_DESC_FLAG_OWNER;

		/* Check if it is the last descriptor which has been received */
		if (desc == desc_ack)
			loop = 0;

		/* Loop in the ring */
		desc = desc->next;
	}
	ep->cur_rx = (struct emac_desc *) desc;

	/* Check if the receiver stopped */
	if (pkt_flags & EMAC_DESC_FLAG_EOQ)
		emac_set_reg(EMAC_RX0HDP, (u32) ep->cur_rx);

	return 0;
}

/*
 * Transmit packets ACK
 */
static int emac_tx(struct net_device *dev, struct emac_desc *desc_ack)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;
	volatile struct emac_desc *desc = ep->dirty_tx;
	int loop = 1;

	while(loop) {
#ifdef EMAC_DEBUG
		printk("%s: ACK desc = 0x%x desc_ack = 0x%x, skb_dirty =%d\n",
		       dev->name, desc, desc_ack, ep->skb_dirty);
#endif
		if (ep->tx_skbuff[ep->skb_dirty] == NULL)
			printk(KERN_ERR "%s: SKB NULL desc =0x%x desc_ack =0x%x, skb_dirty=%d count=%d\n",
			       dev->name, desc, desc_ack, ep->skb_dirty, ep->count_tx);
		else
			/* Free the skbuff associated to this packet */
			dev_kfree_skb_irq(ep->tx_skbuff[ep->skb_dirty]);

		ep->tx_skbuff[ep->skb_dirty] = NULL;
		ep->skb_dirty = (ep->skb_dirty + 1) & TX_RING_MOD_MASK;
		ep->count_tx--;

		/* Check if it is the last acknowledged descriptor */
		if (desc++ == desc_ack)
			loop = 0;

		/* Check end of the ring */
		if (desc > ep->tx_desc_base + TX_RING_SIZE)
			desc = ep->tx_desc_base;
	}

	if (netif_running(dev))
		netif_wake_queue(dev);

	/* Since we have freed up a buffer, the ring is no longer full */
	if (ep->tx_full) {
		ep->tx_full = 0;
#ifdef EMAC_DEBUG
		printk("%s: wake up queue count_tx = %d cur_tx = 0x%x head_tx\n",
		       dev->name, ep->count_tx, ep->cur_tx, ep->head_tx);
#endif
		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
	}

	/* If this is the end of the queue */
	if (desc_ack->packet_flags_len & EMAC_DESC_FLAG_EOQ) {

		/* If there is some waiting packets, start the new queue */
		if (ep->count_tx > 0)
			emac_set_reg(EMAC_TX0HDP, ep->head_tx);

		/* Set the new head */
		ep->head_tx = ep->cur_tx;
	}

	/* Update dirty tx */
	ep->dirty_tx = (struct emac_desc *) desc;

	return 0;
}
/*
 * Transmit the content of a skbuff
 */
static int emac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;
	volatile struct emac_desc *desc;
	volatile struct emac_desc *prev_desc = NULL;
	ushort pkt_len = skb->len;
	unsigned long flags;

	if (ep->tx_full) {
		printk(KERN_WARNING "%s: tx queue full\n", dev->name);
		ep->stats.tx_dropped++;
		return 1;
	}

	/* Pad short frame */
	if (pkt_len <= ETH_ZLEN) {
		pkt_len = ETH_ZLEN;
		skb = skb_padto(skb, ETH_ZLEN);
		if(skb == NULL) {
			netif_wake_queue(dev);
			return 0;
		}
	}

	spin_lock_irqsave(&ep->lock, flags);

	desc = ep->cur_tx;

	/* Set buffer length and pointer */
	desc->buff             = skb->data;
	desc->packet_flags_len = pkt_len;
	desc->buff_offset_len  = pkt_len;
	desc->next             = NULL;         /* close the queue*/

	/* No coherency is assumed between EDMA and L2 cache */
	L2_cache_block_writeback((u32) skb->data,
				 (u32) skb->data + pkt_len);

#ifdef EMAC_DEBUG
	printk("%s: sending packet of %d len, skb_curr = %x, buffer = 0x%x\n",
	       dev->name, pkt_len, ep->skb_cur, skb->data);
	if (*((short *) (skb->data + 12)) == 0x8) {
		char * d;
		printk("%s: IP packet\n");
		for (d = (skb->data + 14); d < (skb->data + 14 + pkt_len); d += 16) {
			printk("%x %x %x %x %x %x %x %x\n",
			       *((short *)d),
			       *((short *)d + 1),
			       *((short *)d + 2),
			       *((short *)d + 3),
			       *((short *)d + 4),
			       *((short *)d + 5),
			       *((short *)d + 6),
			       *((short *)d + 7));
		}
	}
#endif
	/* Save skb */
	ep->tx_skbuff[ep->skb_cur] = skb;
	ep->skb_cur = (ep->skb_cur + 1) & TX_RING_MOD_MASK;
	ep->stats.tx_packets++;
	ep->stats.tx_bytes += pkt_len;
	dev->trans_start = jiffies;

	desc->packet_flags_len |= EMAC_DESC_FLAG_SOP
		| EMAC_DESC_FLAG_EOP
		| EMAC_DESC_FLAG_OWNER;

	/* Get the previous element of the ring if we are not the head */
	if (desc != ep->head_tx) {
		if (desc == ep->tx_desc_base)
			prev_desc = ep->tx_desc_base + TX_RING_SIZE;
		else
			prev_desc = desc - 1;

		/* Link the buffer to the previous one in the list */
		prev_desc->next = (struct emac_desc *) desc;
	}

	ep->count_tx++;

	/* Update current pointer */
	ep->cur_tx = (struct emac_desc *) desc + 1;

	/* Check end of ring */
	if (ep->cur_tx > ep->tx_desc_base + TX_RING_SIZE)
		ep->cur_tx = ep->tx_desc_base;

	/*
	 * If we are the new head and there is no descriptor to acknowledge, start
         * the new head.
	 */
	if ((desc == ep->head_tx) && (ep->count_tx == 1)) {
		emac_set_reg(EMAC_TX0HDP, ep->head_tx);
		ep->head_tx = ep->cur_tx; /* set the new head */
	}

	/* Check ring oll over: do not reach the not yet acknowledged packets */
	if (ep->count_tx == TX_RING_SIZE) {
		ep->tx_full = 1;
#ifdef EMAC_DEBUG
		printk("%s: tx queue full count_tx = %d cur_tx = 0x%x head_tx\n",
		       dev->name, ep->count_tx, ep->cur_tx, ep->head_tx);
#endif
		netif_stop_queue(dev);
	}
	spin_unlock_irqrestore(&ep->lock, flags);

	return 0;
}

/*
 * Main EMAC interrupt management
 */
static void emac_interrupt(int irq, void * dev_id, struct pt_regs * regs)
{
	struct net_device *dev = dev_id;
	struct emac_private *ep = (struct emac_private *) dev->priv;
	struct emac_desc *desc;
	unsigned long irq_flags;
	unsigned long status;

	/* Disable EMAC/MDIO interrupts  */
	emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN);

	irq_flags = emac_get_reg(EMAC_MACINVECTOR);

	if (irq_flags & EMAC_B_STATPEND)
		(void) emac_get_stats(dev);

	if (irq_flags & EMAC_B_HOSTPEND) {
		unsigned long hash1, hash2, promisc;

		/* Handle Host error */
		status = emac_get_reg(EMAC_MACSTATUS);

		if ((status & EMAC_M_RXERRCODE) == (EMAC_V_OWNERSHIP <<  EMAC_S_RXERRCODE)) {
			printk(KERN_ERR "%s: EMAC rx ring is full\n", dev->name);
			ep->stats.rx_dropped++;
		} else
			printk(KERN_ERR "%s: EMAC fatal host error 0x%x\n", dev->name, status);

		printk(KERN_ERR "%s: Error head=0x%x desc=0x%x dirty=0x%x skb_dirty=%d count=%d\n",
		       dev->name, ep->head_tx, ep->cur_tx, ep->dirty_tx, ep->skb_dirty, ep->count_tx);

		ep->fatal_error = 1;

		/* Get rx state */
		hash1 = emac_get_reg(EMAC_MACHASH1);
		hash2 = emac_get_reg(EMAC_MACHASH2);
		promisc = emac_get_reg(EMAC_RXMBPENABLE) & EMAC_B_RXCAFEN;

		/* Reset EMAC */
		emac_close(dev);
		emac_reset(dev);
		emac_open(dev);

		/* Restore rx state */
		emac_set_reg(EMAC_MACHASH1, hash1);
		emac_set_reg(EMAC_MACHASH2, hash2);
		if (promisc)
			emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN);

		return; /* EMAC/MDIO interrupts already enabled by emac_open() */
	}

	if (irq_flags & EMAC_B_TXPEND0) {
		/* ACK the interrupt */
		desc = (struct emac_desc *) emac_get_reg(EMAC_TX0INTACK);
		emac_set_reg(EMAC_TX0INTACK, desc);

		/* Handle transmit ACK event */
		emac_tx(dev_id, desc);
	}

	if (irq_flags & EMAC_B_RXPEND0) {
		/* ACK the interrupt */
		desc = (struct emac_desc *) emac_get_reg(EMAC_RX0INTACK);
		emac_set_reg(EMAC_RX0INTACK, desc);

		/* Handle receive event */
		emac_rx(dev_id, desc);
	}

	/* Enable EMAC/MDIO interrupts */
	emac_setbit_reg(EMAC_EWCTL, EMAC_B_INTEN);

	return;
}

/*
 * Reset the EMAC software ring.
 */
void emac_reset_ring(struct emac_private *ep)
{
	struct emac_desc *desc;
	int i;
	desc = (struct emac_desc *) EMAC_DSC_BASE;
	ep->cur_rx = desc;

	/* Reset rx ring */
	for (i =0; i < RX_RING_SIZE; desc++, i++)
		desc->packet_flags_len = EMAC_DESC_FLAG_OWNER;

	/* Reset tx ring */
	desc = (struct emac_desc *) (EMAC_DSC_BASE + (RX_RING_SIZE << 4));
	ep->cur_tx       = desc;
	ep->dirty_tx     = desc;
	ep->head_tx      = desc;
	ep->skb_cur      = 0;
	ep->skb_dirty    = 0;
	ep->tx_full      = 0;
	ep->fatal_error  = 0;
	ep->count_tx     = 0;

	for (i = 0; i < TX_RING_SIZE; i++) {
		desc->buff             = 0;
		desc->buff_offset_len  = 0;
		desc->packet_flags_len = 0;
		desc->next             = NULL;
		ep->tx_skbuff[i]       = NULL;
		desc++;
	}
}

/*
 * Reset the EMAC hardware.
 */
void emac_reset(struct net_device *dev)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;

	unsigned long val;
	volatile u32 *preg;
	int res = 0;
	int i;

	/* Disable EMAC/MDIO interrupts and reset the EMAC module */
	emac_set_reg(EMAC_EWCTL, EMAC_B_EMACRST);

	/* Wait about 100 cycles */
	for (i = 0; i< 5; i++)
		val = emac_get_reg(EMAC_EWCTL);

	/* Remove reset of the EMAC module */
	emac_clearbit_reg(EMAC_EWCTL, EMAC_B_EMACRST);

	/* Wait about 100 cycles */
	for (i = 0; i< 5; i++)
		val = emac_get_reg(EMAC_EWCTL);

	/* Set EMAC priority */
	emac_set_reg(EMAC_EWTRCTRL, 0x13);

	/* Set interrupt timer count (CPU clock / 4) */
	emac_set_reg(EMAC_EWINTTCNT, 0);

	/* Init HDPs to NULL */
	emac_set_reg(EMAC_TX0HDP, 0);
	emac_set_reg(EMAC_TX1HDP, 0);
	emac_set_reg(EMAC_TX2HDP, 0);
	emac_set_reg(EMAC_TX3HDP, 0);
	emac_set_reg(EMAC_TX4HDP, 0);
	emac_set_reg(EMAC_TX5HDP, 0);
	emac_set_reg(EMAC_TX6HDP, 0);
	emac_set_reg(EMAC_TX7HDP, 0);
	emac_set_reg(EMAC_RX0HDP, 0);
	emac_set_reg(EMAC_RX1HDP, 0);
	emac_set_reg(EMAC_RX2HDP, 0);
	emac_set_reg(EMAC_RX3HDP, 0);
	emac_set_reg(EMAC_RX4HDP, 0);
	emac_set_reg(EMAC_RX5HDP, 0);
	emac_set_reg(EMAC_RX6HDP, 0);
	emac_set_reg(EMAC_RX7HDP, 0);

	/* Setup MAC address */
	emac_set_reg(EMAC_MACADDRL0, config.enetaddr[5]);
	dev->dev_addr[5] = config.enetaddr[5];
	emac_set_reg(EMAC_MACADDRM, config.enetaddr[4]);
	dev->dev_addr[4] = config.enetaddr[4];

	val = 0;
	for (i = 3; i >= 0; i--) {
		val = (val << 8) | config.enetaddr[i];
		dev->dev_addr[i] = config.enetaddr[i];
	}
	emac_set_reg(EMAC_MACADDRH, val);

	/* Set buffer offset */
	emac_set_reg(EMAC_RXBUFFEROFFSET, 0);

	/* Reset RX (M)ulticast (B)roadcast (P)romiscuous Enable register */
	emac_set_reg(EMAC_RXMBPENABLE, 0);
	emac_set_reg(EMAC_MACHASH1, 0);
	emac_set_reg(EMAC_MACHASH2, 0);

	/* Clear Unicast RX on channel 0-7 */
	emac_set_reg(EMAC_RXUNICASTCLEAR, 0xff);

	if (ep->mode_flags & EMAC_CONFIG_RXCRC) {
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXPASSCRC);
		ep->packet_mtu = PKT_MTU_CRC;
	} else
		ep->packet_mtu = PKT_MTU_NOCRC;

	/* If PASSERROR is set, enable both ERROR and short frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSERROR)
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCEFEN | EMAC_B_RXCSFEN);

	/* If PASSCONTROL is set, enable control frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSCONTROL)
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCMFEN);

	/* Set the channel configuration to priority if requested */
	if (ep->mode_flags & EMAC_CONFIG_CHPRIORITY)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_TXPTYPE);

	/* Set MAC loopback if requested */
	if(ep->mode_flags & EMAC_CONFIG_MACLOOPBACK)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_LOOPBACK);

	/* Enable TX and RX channel interrupts then host interrupts */
	emac_set_reg(EMAC_RXINTMASKCLEAR, 0xff);
	emac_set_reg(EMAC_TXINTMASKCLEAR, 0xff);
	emac_set_reg(EMAC_RXINTMASKSET, 1);  /* Receive channel 0 int enable */
	emac_set_reg(EMAC_TXINTMASKSET, 1);  /* Transmit channel 0 int enable */
	emac_set_reg(EMAC_MACINTMASKSET, EMAC_B_HOSTERRINT | EMAC_B_STATINT);

	/* Set receive filter: unicast, multicast and broadcast */
	emac_set_reg(EMAC_RXUNICASTSET, 1); /* Unicast for channel 0 */
	emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_MULTEN);
	emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_BROADEN);

	/* Reset transmit/receive buffers */
	(void) emac_reset_ring(ep);

	/* Enable RX, TX and MII */
	emac_set_reg(EMAC_TXCONTROL, 1);
	emac_set_reg(EMAC_RXCONTROL, 1);
	emac_set_reg(EMAC_MACCONTROL, EMAC_B_MIIEN);
}

/*
 * Open the device
 */
static int emac_open(struct net_device *dev)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;

	/* Enable global EMAC interrupt */
	emac_setbit_reg(EMAC_EWCTL, EMAC_B_INTEN);

	/* Startup RX */
	emac_set_reg(EMAC_RX0HDP, ep->cur_rx);

	netif_start_queue(dev);
	return 0;
}

/*
 * Close the device
 */
static int emac_close(struct net_device *dev)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;
	unsigned int tmp;

	netif_stop_queue(dev);

	/* Disable EMAC/MDIO interrupts  */
	emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN);

	/* Teardown RX and TX */
	emac_set_reg(EMAC_RXTEARDOWN, 0);
	emac_set_reg(EMAC_TXTEARDOWN, 0);

	if (!ep->fatal_error) {
		/* Wait for the teardown to complete */
		for(tmp = 0; tmp != 0xfffffffc; tmp = emac_get_reg(EMAC_RX0INTACK));
		emac_set_reg(EMAC_RX0INTACK, tmp);

		for(tmp = 0; tmp != 0xfffffffc; tmp = emac_get_reg(EMAC_TX0INTACK));
		emac_set_reg(EMAC_TX0INTACK, tmp);
	}

	/* Disable RX, TX and MII */
	emac_set_reg(EMAC_TXCONTROL, 0);
	emac_set_reg(EMAC_RXCONTROL, 0);
	emac_set_reg(EMAC_MACCONTROL, 0);

	/* Clean up the software ring */
	emac_reset_ring(ep);

	return 0;
}

/*
 * Initialize the EMAC software ring.
 * For the rx ring, we initialize the packet descriptors with pre-allocated
 * skbuff to the packet MTU size.
 * For the tx ring, we only initialize it with NULL values.
 */
int __init emac_setup_ring(struct emac_private *ep)
{
	struct emac_desc *desc;
	u8 *buff;
	int i, mtu;

	/* Setup rx ring*/
	desc = (struct emac_desc *) EMAC_DSC_BASE;
	ep->rx_desc_base = desc;
	ep->cur_rx       = desc;

	mtu  = (ep->packet_mtu + 0x3) & ~0x3;

	/* Allocate ring buffers */
	buff = (u8 *) kmalloc(mtu * RX_RING_SIZE, GFP_KERNEL);

	if (!buff)
		return 1;

	for (i = 0; i < RX_RING_SIZE; i++) {
		desc->buff             = (u8 *) buff;
		desc->buff_offset_len  = ep->packet_mtu;
		desc->packet_flags_len = EMAC_DESC_FLAG_OWNER;
		desc->next             = desc + 1;
		desc++;
		buff += mtu;
	}

	/* Loop the ring */
	(desc - 1)->next = ep->rx_desc_base;

	/* Setup tx ring */
	desc = (struct emac_desc *) (EMAC_DSC_BASE + (RX_RING_SIZE << 4));
	ep->tx_desc_base = desc;
	ep->cur_tx       = desc;
	ep->dirty_tx     = desc;
	ep->head_tx      = desc;
	ep->skb_cur      = 0;
	ep->skb_dirty    = 0;
	ep->tx_full      = 0;
	ep->fatal_error  = 0;
	ep->count_tx     = 0;

	for (i = 0; i < TX_RING_SIZE; i++) {
		desc->buff             = 0;
		desc->buff_offset_len  = 0;
		desc->packet_flags_len = 0;
		desc->next             = NULL;
		ep->tx_skbuff[i]       = NULL;
		desc++;
	}
	return 0;
}

/*
 * Called by the kernel to send a packet out into the void
 * of the net.
 */
static void emac_timeout(struct net_device *dev)
{
	struct emac_private *ep = (struct emac_private *) dev->priv;

	printk(KERN_WARNING "%s: transmit timed out\n", dev->name);
	ep->stats.tx_errors++;
	dev->trans_start = jiffies;
	if (!ep->tx_full)
		netif_wake_queue(dev);
}

/*
 * Set or clear the multicast filter for this adaptor.
 */
static void emac_set_rx_mode(struct net_device *dev)
{
	struct	dev_mc_list *dmi;
	u8      hashval, tmpval;
	u32     machash1, machash2;
	int     i, j;

	if (dev->flags & IFF_PROMISC) {
		/* Set promiscuous mode */
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN);
	} else {
		/* Reset promiscuous mode */
		emac_clearbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN);
	}

	if (dev->flags & IFF_ALLMULTI) {

		/* Set all multicast filter */
		emac_setbit_reg(EMAC_MACHASH1, 0xffffffff);
		emac_setbit_reg(EMAC_MACHASH2, 0xffffffff);

	} else {

		dmi      = dev->mc_list;
		hashval  = 0;
		machash1 = 0;
		machash2 = 0;

		for (i = 0; i < dev->mc_count; i++, dmi = dmi->next) {
			u8 *p_dmi = dmi->dmi_addr;

			/* Only support group multicast */
			if (!(*p_dmi & 1))
				continue;

			for (j = 0; j < 2; j++) {
				tmpval  = (u8) *p_dmi++;
				hashval ^= (u8) (tmpval >> 2) ^ (tmpval << 4);
				tmpval  = (u8) *p_dmi++;
				hashval ^= (u8) (tmpval >> 4) ^ (tmpval << 2);
				tmpval  = (u8) *p_dmi++;
				hashval ^= (u8) (tmpval >> 6) ^ (tmpval);
			}

			if (hashval & 0x20)
				machash2 |= (1 << (hashval & 0x1f));
			else
				machash1 |= (1 << (hashval & 0x1f));
		}

		/* Set computed multicast filter */
		emac_setbit_reg(EMAC_MACHASH1, machash1);
		emac_setbit_reg(EMAC_MACHASH2, machash2);
	}
}

/*
 * This function should be called for each device in the system on a
 * periodic basis of 100mS (10 times a second). It is used to check the
 * status of the EMAC and MDIO device.
 */
static void emac_timer_tick(unsigned long dummy)
{
	unsigned int link_status;
	unsigned int link_event = mdio_timer_tick();

	/*
	 * Signal the MDIO
	 */
	if (link_event == MDIO_EVENT_LINKUP) {
		link_status = mdio_get_status();
		if (link_status == MDIO_LINKSTATUS_FD10 ||
		    link_status == MDIO_LINKSTATUS_FD100 )
			emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_FULLDUPLEX);
		else
			emac_clearbit_reg(EMAC_MACCONTROL, EMAC_B_FULLDUPLEX);
#ifdef CONFIG_LSI_LOGIC
		mdio_lsi_linkstatus(link_status);
#endif
	}
	mod_timer(&emac_timer, jiffies + EMAC_TIMER_PERIOD);
}

/*
 * Initialize EMAC controller
 */
int __init emac_init(void)
{
	struct net_device *dev;
	struct emac_private *ep;
	unsigned long val;
	volatile u32 *preg;
	int res = 0;
	int i, err;

	/* Disable EMAC/MDIO interrupts and reset EMAC/MDIO modules */
	emac_set_reg(EMAC_EWCTL, EMAC_B_EMACRST | EMAC_B_MDIORST);

	/* Wait about 100 cycles */
	for (i = 0; i< 5; i++)
		val = emac_get_reg(EMAC_EWCTL);

	/* Remove reset of EMAC/MDIO modules */
	emac_clearbit_reg(EMAC_EWCTL, (EMAC_B_EMACRST | EMAC_B_MDIORST));

	/* Wait about 100 cycles */
	for (i = 0; i< 5; i++)
		val = emac_get_reg(EMAC_EWCTL);

	/* Set EMAC priority */
	emac_set_reg(EMAC_EWTRCTRL, 0x13);

	/* Set interrupt timer count (CPU clock / 4) */
	emac_set_reg(EMAC_EWINTTCNT, 0);

	/* MII/MDIO setup */
	mdio_init();

	/* Reset MAC Control */
	emac_set_reg(EMAC_MACCONTROL, 0);

	/* Init HDPs to NULL */
	emac_set_reg(EMAC_TX0HDP, 0);
	emac_set_reg(EMAC_TX1HDP, 0);
	emac_set_reg(EMAC_TX2HDP, 0);
	emac_set_reg(EMAC_TX3HDP, 0);
	emac_set_reg(EMAC_TX4HDP, 0);
	emac_set_reg(EMAC_TX5HDP, 0);
	emac_set_reg(EMAC_TX6HDP, 0);
	emac_set_reg(EMAC_TX7HDP, 0);
	emac_set_reg(EMAC_RX0HDP, 0);
	emac_set_reg(EMAC_RX1HDP, 0);
	emac_set_reg(EMAC_RX2HDP, 0);
	emac_set_reg(EMAC_RX3HDP, 0);
	emac_set_reg(EMAC_RX4HDP, 0);
	emac_set_reg(EMAC_RX5HDP, 0);
	emac_set_reg(EMAC_RX6HDP, 0);
	emac_set_reg(EMAC_RX7HDP, 0);

	/* Clear statistic registers */
	preg = emac_addr_reg(EMAC_RXGOODFRAMES);
	for (i = 0; i < EMAC_NUM_STATREGS; i++)
		*preg++ = 0;

	/* Allocate private information */
	dev = alloc_etherdev(sizeof(struct emac_private));
	if (!dev)
		return -ENOMEM;

	if ((err = register_netdev(dev))) {
		free_netdev(dev);
		return err;
	}

	ep = dev->priv;
	ep->mode_flags = config.flags;
	dev->base_addr = EMAC_REG_BASE;

	/* Setup MAC address */
	emac_set_reg(EMAC_MACADDRL0, config.enetaddr[5]);
	dev->dev_addr[5] = config.enetaddr[5];
	emac_set_reg(EMAC_MACADDRM, config.enetaddr[4]);
	dev->dev_addr[4] = config.enetaddr[4];

	val = 0;
	for (i = 3; i >= 0; i--) {
		val = (val << 8) | config.enetaddr[i];
		dev->dev_addr[i] = config.enetaddr[i];
	}
	emac_set_reg(EMAC_MACADDRH, val);

	/* Set buffer offset */
	emac_set_reg(EMAC_RXBUFFEROFFSET, 0);

	/* Reset RX (M)ulticast (B)roadcast (P)romiscuous Enable register */
	emac_set_reg(EMAC_RXMBPENABLE, 0);
	emac_set_reg(EMAC_MACHASH1, 0);
	emac_set_reg(EMAC_MACHASH2, 0);

	/* Clear Unicast RX on channel 0-7 */
	emac_set_reg(EMAC_RXUNICASTCLEAR, 0xff);

	if (ep->mode_flags & EMAC_CONFIG_RXCRC) {
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXPASSCRC);
		ep->packet_mtu = PKT_MTU_CRC;
	} else
		ep->packet_mtu = PKT_MTU_NOCRC;

	/* If PASSERROR is set, enable both ERROR and short frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSERROR)
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCEFEN | EMAC_B_RXCSFEN);

	/* If PASSCONTROL is set, enable control frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSCONTROL)
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCMFEN);

	/* Set the channel configuration to priority if requested */
	if (ep->mode_flags & EMAC_CONFIG_CHPRIORITY)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_TXPTYPE);

	/* Set MAC loopback if requested */
	if(ep->mode_flags & EMAC_CONFIG_MACLOOPBACK)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_LOOPBACK);

	/* Enable TX and RX channel interrupts then host interrupts */
	emac_set_reg(EMAC_RXINTMASKCLEAR, 0xff);
	emac_set_reg(EMAC_TXINTMASKCLEAR, 0xff);
	emac_set_reg(EMAC_RXINTMASKSET, 1);  /* Receive channel 0 int enable */
	emac_set_reg(EMAC_TXINTMASKSET, 1);  /* Transmit channel 0 int enable */
	emac_set_reg(EMAC_MACINTMASKSET, EMAC_B_HOSTERRINT | EMAC_B_STATINT);

	/* Set receive filter: unicast, multicast and broadcast */
	emac_set_reg(EMAC_RXUNICASTSET, 1); /* Unicast for channel 0 */
	emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_MULTEN);
	emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_BROADEN);

	/* Setup transmit/receive buffers */
	res = emac_setup_ring(ep);
	if (res)
		goto error;

	/* Set generic Ethernet operations */
	dev->open               = emac_open;
	dev->stop               = emac_close;
	dev->hard_start_xmit    = emac_start_xmit;
	dev->tx_timeout         = emac_timeout;
	dev->get_stats          = emac_get_stats;
	dev->set_multicast_list = emac_set_rx_mode;
	dev->watchdog_timeo     = TX_TIMEOUT;

	/* Set EMAC timer */
	init_timer(&emac_timer);
	emac_timer.function = emac_timer_tick;
	mod_timer(&emac_timer, jiffies + EMAC_TIMER_PERIOD);

	/* Enable RX, TX and MII */
	emac_set_reg(EMAC_TXCONTROL, 1);
	emac_set_reg(EMAC_RXCONTROL, 1);
	emac_set_reg(EMAC_MACCONTROL, EMAC_B_MIIEN);

	/* Install our interrupt handler */
	irq_map(IRQ_MACINT, INT10);
	if (request_irq(INT10, emac_interrupt, 0, "EMAC", dev) == 0)
		dev->irq = INT10;

	printk("%s: EMAC driver version 0.2 IRQ %d\n", dev->name,
	       dev->irq);
	printk("%s: Ethernet addr: ", dev->name);
	for (i = 0; i < 5; i++)
		printk("%02x:", dev->dev_addr[i]);
	printk("%02x\n", dev->dev_addr[5]);

	return 0;

 error:
	kfree(ep);
	return res;
}

module_init(emac_init);
