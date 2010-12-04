/*
 *  linux/arch/c6x/kernel/gemac.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2007, 2008, 2009, 2010 Texas Instruments Incorporated
 *  Authors: Nicolas Videau (nicolas.videau@virtuallogix.com)
 *           Aurelien Jacquiot (a-jacquiot@ti.com)
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
#include <linux/platform_device.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gmdio.h>
#include <asm/gemac.h>

#undef EMAC_DEBUG

#ifdef EMAC_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "GEMAC: [%s] " fmt, __FUNCTION__, ## args)
#define ASSERT(cond)          if (!(cond)) DPRINTK("ASSERT %s FAILURE\n", # cond)
#else
#define ASSERT(cond)
#define DPRINTK(fmt, args...) 
#endif

static int emac_open(struct net_device *dev);
static int emac_close(struct net_device *dev);
static int emac_reset(struct net_device *dev, int reset_mode);

extern int mdio_init(unsigned int txid_version);

#ifdef EMAC_TIMER_TICK_MDIO
static struct timer_list  emac_timer;
#endif

static struct emac_config config = { 0, { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }};

static int emac_shared = 0;
static int emac_idx    = 0;

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

static int __init set_emac_shared(char *str)
{
	emac_shared = 1;
	return 1;
}

__setup("emac_shared", set_emac_shared);

/*
 * Get the device statistic
 */
static struct net_device_stats *emac_get_stats(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);
	unsigned int reg;
	unsigned int dummy;
	int i;

	emac_set_stat(dev->stats.multicast,         EMAC_RXMCASTFRAMES);
	emac_set_stat(dev->stats.collisions,        EMAC_TXCOLLISION);
	emac_set_stat(dev->stats.rx_length_errors,  EMAC_RXOVERSIZED);
	emac_set_stat(dev->stats.rx_length_errors,  EMAC_RXUNDERSIZED);
	emac_set_stat(dev->stats.rx_crc_errors,     EMAC_RXCRCERRORS);
	emac_set_stat(dev->stats.rx_frame_errors,   EMAC_RXALIGNCODEERRORS);
	emac_set_stat(dev->stats.tx_carrier_errors, EMAC_TXCARRIERSLOSS);
	emac_set_stat(dev->stats.tx_fifo_errors,    EMAC_TXUNDERRUN);
	emac_set_stat(dev->stats.tx_window_errors,  EMAC_TXLATECOLL);

	/* ACK statistic by write-to-decrement */
	reg = EMAC_RXGOODFRAMES;
	for (i = 0; i < EMAC_NUM_STATREGS; i++) {
		dummy = emac_get_reg(reg);
		emac_set_reg(reg, dummy);
		reg += 4;
	}

	return &dev->stats;
}

/*
 * Receive packets
 */
static int emac_rx(struct net_device *dev, struct emac_desc *desc_ack)
{
	struct emac_private         *ep = netdev_priv(dev);
	volatile struct emac_desc *desc = ep->cur_rx;
	ushort pkt_len;
	u32 pkt_flags = desc->packet_flags_len & 0xffff0000;
	int loop = 1;
	struct sk_buff *skb, *skb_old;

	DPRINTK("%s: RX desc_ack = 0x%x, cur_rx = 0x%x, num = %d\n",
	       dev->name, desc_ack, desc, (desc_ack - desc));

	if ((desc == desc_ack) && (desc->packet_flags_len == EMAC_DESC_FLAG_OWNER))
		loop = 0;

	while(loop) {
		skb_old = ep->rx_skbuff[ep->skb_rx_dirty];

		pkt_len   = desc->packet_flags_len & EMAC_PACKET_LEN_MASK;
		pkt_flags = desc->packet_flags_len & 0xffff0000;

		skb = netdev_alloc_skb_ip_align(dev, ep->packet_mtu);
		if (skb != NULL) {
			/* Prepare old sk_buff */
			skb_old->dev = dev;
			skb_put(skb_old, pkt_len);

			/* No coherency is assumed between EDMA and L2 cache */
			L2_cache_block_invalidate((u32) desc->buff,
						  (u32) desc->buff + pkt_len);
			
			skb_old->protocol = eth_type_trans(skb_old, dev);

			/* Attach new sk_buff to RX ring */
			desc->buff = (u8 *) skb->data;
			ep->rx_skbuff[ep->skb_rx_dirty] = skb;

			DPRINTK("%s: receiving packet of %d len, proto 0x%x\n",
			       dev->name, pkt_len, skb->protocol);

			/* Give back old sk_buff */
			netif_rx(skb_old);
			dev->last_rx = jiffies;

			/* Fill statistic */
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += pkt_len;
		} else {
			printk(KERN_WARNING "%s: Memory squeeze, dropping packet.\n", dev->name);
			dev->stats.rx_dropped++;
		}

		/* Update RX dirty */
		if (ep->skb_rx_dirty >= RX_RING_MOD_MAX)
			ep->skb_rx_dirty = 0;
		else
			ep->skb_rx_dirty = ep->skb_rx_dirty + 1;

		/* Descriptor is now available */
		desc->buff_offset_len = ep->packet_mtu;
		desc->packet_flags_len = EMAC_DESC_FLAG_OWNER;
		
		/* Check if it is the last descriptor which has been received */
		if (desc == desc_ack)
			loop = 0;

		/* Loop in the ring */
		desc = HW_TO_PTR(desc->next);
	}
	ep->cur_rx = (struct emac_desc *) desc;

	/* Check if the receiver stopped */
	if (pkt_flags & EMAC_DESC_FLAG_EOQ)
		emac_set_reg(EMAC_RX0HDP + (IDX_TO_CHAN(emac_idx) << 2),
			     PTR_TO_HW((u32) ep->cur_rx));

	return 0;
}

/*
 * Transmit packets ACK
 */
static int emac_tx(struct net_device *dev, struct emac_desc *desc_ack)
{
	struct emac_private         *ep = netdev_priv(dev);
	volatile struct emac_desc *desc = ep->dirty_tx;
	int loop = 1;

	while(loop) {
		DPRINTK("%s: TX desc_ack = 0x%x, desc = 0x%x, num = %d\n",
			dev->name, desc_ack, desc, (desc_ack - desc));

		if (ep->tx_skbuff[ep->skb_tx_dirty] == NULL) {
			printk(KERN_ERR "%s: SKB NULL desc =%p desc_ack =%p, skb_tx_dirty=%ld count=%ld\n",
			       dev->name, desc, desc_ack, ep->skb_tx_dirty, ep->count_tx);
			break;
		}

		/* Free the skbuff associated to this packet */
		dev_kfree_skb_irq(ep->tx_skbuff[ep->skb_tx_dirty]);

		ep->tx_skbuff[ep->skb_tx_dirty] = NULL;
		ep->skb_tx_dirty = (ep->skb_tx_dirty + 1) & TX_RING_MOD_MASK;
		ep->count_tx--;

		/* Check if it is the last acknowledged descriptor */
		if (desc++ == desc_ack)
			loop = 0;

		/* Check end of the ring */
		if (desc > ep->tx_desc_base + (TX_RING_SIZE - 1))
			desc = ep->tx_desc_base;
	}

	if (netif_running(dev))
		netif_wake_queue(dev);

	/* Since we have freed up a buffer, the ring is no longer full */
	if (ep->tx_full) {
		ep->tx_full = 0;

		DPRINTK("%s: wake up queue count_tx = %d cur_tx = 0x%x head_tx\n",
			dev->name, ep->count_tx, ep->cur_tx, ep->head_tx);

		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
	}

	/* If this is the end of the queue */
	if (desc_ack->packet_flags_len & EMAC_DESC_FLAG_EOQ) {

		/* If there are some waiting packets, start the new queue */
		if (ep->count_tx > 0)
			emac_set_reg(EMAC_TX0HDP + (IDX_TO_CHAN(emac_idx) << 2),
				     PTR_TO_HW(ep->head_tx));

		/* Set the new head */
		ep->head_tx = ep->cur_tx;
	}

	/* Update dirty tx */
	ep->dirty_tx = (struct emac_desc *) desc;

	return NETDEV_TX_OK;
}

/*
 * Transmit the content of a skbuff
 */
static int emac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
 	struct emac_private       *ep =  netdev_priv(dev);
	volatile struct emac_desc *desc;
	volatile struct emac_desc *prev_desc = NULL;
	ushort pkt_len = skb->len;
	unsigned long flags;

	if (ep->tx_full) {
		printk(KERN_WARNING "%s: tx queue full\n", dev->name);
		dev->stats.tx_dropped++;
		return NETDEV_TX_BUSY;
	}

	/* Pad short frame */
	if (unlikely(pkt_len < ETH_ZLEN)) {
		if (skb_padto(skb, ETH_ZLEN)) {
			netif_wake_queue(dev);
			return NETDEV_TX_OK;
		}
		pkt_len = ETH_ZLEN;
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

	DPRINTK("%s: sending packet of %d len, skb_cur = %x, buffer = 0x%x\n",
		dev->name, pkt_len, ep->skb_cur, skb->data);

	/* Save skb */
	ep->tx_skbuff[ep->skb_cur] = skb;
	ep->skb_cur = (ep->skb_cur + 1) & TX_RING_MOD_MASK;
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += pkt_len;
	dev->trans_start = jiffies;

	desc->packet_flags_len |= EMAC_DESC_FLAG_SOP
		| EMAC_DESC_FLAG_EOP
		| EMAC_DESC_FLAG_OWNER;
	
	/* Get the previous element of the ring if we are not the head */
	if (desc != ep->head_tx) {
		if (desc == ep->tx_desc_base)
			prev_desc = ep->tx_desc_base + (TX_RING_SIZE - 1);
		else
			prev_desc = desc - 1;

		/* Link the buffer to the previous one in the list */
		prev_desc->next = (struct emac_desc *) PTR_TO_HW(desc);
	}

	ep->count_tx++;

	/* Update current pointer */
	ep->cur_tx = (struct emac_desc *) desc + 1;

	/* Check end of ring */
	if (ep->cur_tx > ep->tx_desc_base + (TX_RING_SIZE - 1))
		ep->cur_tx = ep->tx_desc_base;

	/*
	 * If we are the new head and there is no descriptor to acknowledge, start
         * the new head.
	 */
	if ((desc == ep->head_tx) && (ep->count_tx == 1)) {
		emac_set_reg(EMAC_TX0HDP + (IDX_TO_CHAN(emac_idx) << 2),
			     PTR_TO_HW(ep->head_tx));
		ep->head_tx = ep->cur_tx; /* set the new head */
	}

	/* Check ring oll over: do not reach the not yet acknowledged packets */
	if (ep->count_tx == TX_RING_SIZE) {
		ep->tx_full = 1;

		DPRINTK("%s: tx queue full count_tx = %d cur_tx = 0x%x head_tx\n",
			dev->name, ep->count_tx, ep->cur_tx, ep->head_tx);

		netif_stop_queue(dev);
	}
	spin_unlock_irqrestore(&ep->lock, flags);

	return NETDEV_TX_OK;
}

static void emac_handle_host_interrupt(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);
	unsigned long        status;

	/* Handle host error */
	status = emac_get_reg(EMAC_MACSTATUS);

	/* Check if the problem occurs on our channel when we are slave */
	if ((ep->slave) 
	    && (((status & EMAC_M_RXERRCH) >> EMAC_S_RXERRCH) != IDX_TO_CHAN(emac_idx))
	    && (((status & EMAC_M_TXERRCH) >> EMAC_S_TXERRCH) != IDX_TO_CHAN(emac_idx)))
		return;

	if ((status & EMAC_M_RXERRCODE) == (EMAC_V_OWNERSHIP <<  EMAC_S_RXERRCODE)) {
		printk(KERN_ERR "%s: EMAC rx ring full\n", dev->name);
		dev->stats.rx_fifo_errors++;
	} else
		printk(KERN_ERR "%s: EMAC fatal host error 0x%lx\n",
		       dev->name, status);
	
	DPRINTK(KERN_ERR "%s: Error head=%p desc=%p dirty=%p skb_tx_dirty=%ld count=%ld\n",
		dev->name, ep->head_tx, ep->cur_tx,
		ep->dirty_tx, ep->skb_tx_dirty, ep->count_tx);
	
	if (!ep->slave) {
		/* Reconfigure the device */
		ep->fatal_error = 1;
		emac_reconfigure_device(dev);
	}
}

/*
 * Receive EMAC interrupt management
 */

#ifdef EMAC_HAS_SEPARATE_RXTX_IRQS

static irqreturn_t emac_rx_interrupt(int irq, void * netdev_id)
{
	struct net_device *dev = netdev_id;
	struct emac_private *ep = netdev_priv(dev);
	struct emac_desc *desc;
	unsigned long irq_flags;
	unsigned long status;

	ectl_arch_rx_irq_enter(IDX_TO_CHAN(emac_idx));

	irq_flags = emac_arch_get_int_vector();

	if (irq_flags & EMAC_B_STATPEND)
		(void) emac_get_stats(dev);
	
	if (irq_flags & EMAC_B_HOSTPEND)
		emac_handle_host_interrupt(dev);

	if (emac_arch_get_pending_irq(irq_flags,
				      EMAC_B_RXPEND0,
				      IDX_TO_CHAN(emac_idx))) {

		/* ACK the interrupt */
		desc = (struct emac_desc *) emac_get_reg(EMAC_RX0INTACK
							 + (IDX_TO_CHAN(emac_idx) << 2));
		emac_set_reg(EMAC_RX0INTACK + (IDX_TO_CHAN(emac_idx) << 2), desc);

		/* Handle receive event */
		emac_rx(netdev_id, HW_TO_PTR(desc));
	}

	ectl_arch_rx_irq_leave(IDX_TO_CHAN(emac_idx));

	return IRQ_HANDLED;
}

/*
 * Transmit EMAC interrupt management
 */
static irqreturn_t emac_tx_interrupt(int irq, void * netdev_id)
{
	struct net_device   *dev = netdev_id;
	struct emac_private *ep  = netdev_priv(dev);
	struct emac_desc    *desc;
	unsigned long irq_flags;
	unsigned long status;

	ectl_arch_tx_irq_enter(IDX_TO_CHAN(emac_idx));

	irq_flags = emac_arch_get_int_vector();

	if (irq_flags & EMAC_B_STATPEND)
		(void) emac_get_stats(dev);
	
	if (irq_flags & EMAC_B_HOSTPEND)
		emac_handle_host_interrupt(dev);
	
	if (emac_arch_get_pending_irq(irq_flags,
				      EMAC_B_TXPEND0,
				      IDX_TO_CHAN(emac_idx))) {

		/* ACK the interrupt */
		desc = (struct emac_desc *) emac_get_reg(EMAC_TX0INTACK
							 + (IDX_TO_CHAN(emac_idx) << 2));
		emac_set_reg(EMAC_TX0INTACK + (IDX_TO_CHAN(emac_idx) << 2), desc);

		/* Handle transmit ACK event */
		emac_tx(netdev_id, HW_TO_PTR(desc));
      	}

	ectl_arch_tx_irq_leave(IDX_TO_CHAN(emac_idx));

	return IRQ_HANDLED;
}

#else

/*
 * Main EMAC interrupt management
 */
static irqreturn_t emac_interrupt(int irq, void * netdev_id)
{
	struct net_device   *dev = netdev_id;
	struct emac_private *ep  = netdev_priv(dev);
	struct emac_desc    *desc;
	unsigned long irq_flags;
	unsigned long status;

	/* Disable EMAC/MDIO interrupts  */
	emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN);

	irq_flags = emac_get_reg(EMAC_MACINVECTOR);

	if (irq_flags & EMAC_B_STATPEND)
		(void) emac_get_stats(dev);
	
	if (irq_flags & EMAC_B_HOSTPEND) {
		emac_handle_host_interrupt(dev);

		/* EMAC/MDIO interrupts already enabled by emac_open() */
		return IRQ_HANDLED;
	}

	if (irq_flags & EMAC_B_TXPEND0) {
		/* ACK the interrupt */
		desc = (struct emac_desc *) emac_get_reg(EMAC_TX0INTACK);
		emac_set_reg(EMAC_TX0INTACK, desc);

		/* Handle transmit ACK event */
		emac_tx(netdev_id, desc);
	}

	if (irq_flags & EMAC_B_RXPEND0) {
		/* ACK the interrupt */
		desc = (struct emac_desc *) emac_get_reg(EMAC_RX0INTACK);
		emac_set_reg(EMAC_RX0INTACK, desc);

		/* Handle receive event */
		emac_rx(netdev_id, desc);
	}

	/* Enable EMAC/MDIO interrupts */
	emac_setbit_reg(EMAC_EWCTL, EMAC_B_INTEN);

	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * EMAC poll
 */
static void emac_poll(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);

	ectl_arch_disable_irq(IDX_TO_CHAN(emac_idx));

#ifdef EMAC_HAS_SEPARATE_RXTX_IRQS
	emac_tx_interrupt(dev->irq + 1, dev);
	emac_rx_interrupt(dev->irq, dev);
#else
	emac_interrupt(dev->irq, dev);
#endif

	ectl_arch_enable_irq(IDX_TO_CHAN(emac_idx), !ep->slave);
}
#endif

/*
 * Register the MAC address
 */

static void emac_register_mac_addr(struct net_device *dev, int idx)
{
	struct emac_private *ep = netdev_priv(dev);
	u32 val;
	int i;

	/* Copy the config MAC address to the dev_addr */
	for (i = 4; i >= 0; i--)
		dev->dev_addr[i] = config.enetaddr[i];
	/* 
	 * Compute MAC addr low for all for the given EMAC instances if several.
	 * Each device consume n MAC addresses where n is the number cores.
	 * This value is for GEM0, this value + 2 is for GEM1, etc
	 * The number programmed in this field will be incremented by n * 2 from 
	 * device to device
	 */
	dev->dev_addr[5] = (config.enetaddr[5] +
			    (IDX_TO_MAC(idx)) + DEV_TO_MAC(dev->dev_id)) & 0xff;

	/* Choose MAC address index between 0 and 31 */
	emac_arch_set_mac_addr_index(idx);

	/* Compute and store MAC addr high */
	val = 0;
	for (i = 3; i >= 0; i--)
		val = (val << 8) | dev->dev_addr[i];
	emac_arch_set_mac_addr_high(val);

	val  = (dev->dev_addr[5] << 8) | dev->dev_addr[4];
	val |= (1 << EMAC_S_VALID) 
		| (1 << EMAC_S_MATCHFILTER)
		| (IDX_TO_CHAN(idx) << EMAC_S_CHANNEL);
	
	emac_arch_set_mac_addr_low(val);
}

/*
 * Reset the EMAC software ring.
 */
static void emac_reset_ring(struct emac_private *ep)
{
	struct emac_desc *desc;
	int i;

	desc = (struct emac_desc *) ep->emac_dsc_base;

	/* Reset rx ring */
	ep->cur_rx       = desc;
	ep->skb_rx_dirty = 0;

	for (i = 0; i < RX_RING_SIZE; desc++, i++) 
		desc->packet_flags_len = EMAC_DESC_FLAG_OWNER;

	/* Reset tx ring */
	desc = (struct emac_desc *) (ep->emac_dsc_base + (RX_RING_SIZE << 4));
	ep->cur_tx       = desc;
	ep->dirty_tx     = desc;
	ep->head_tx      = desc;
	ep->skb_cur      = 0;
	ep->skb_tx_dirty = 0;
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
 * Reconfigure the EMAC device
 */
static int emac_reconfigure_device(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);

	if (!ep->slave) {
		unsigned int  hash1;
		unsigned int  hash2;
		unsigned int  promisc;
		unsigned int  control;
		unsigned int  rx_intmask;
		unsigned int  tx_intmask;
		unsigned int  unicast;
		unsigned int *mac_ctx;
		int           i;

		/* Get state */
		hash1   = emac_arch_get_hash1();
		hash2   = emac_arch_get_hash2();
		promisc = emac_arch_get_promisc();
		control = emac_arch_get_control();

		unicast    = emac_get_reg(EMAC_RXUNICASTSET);
		rx_intmask = emac_get_reg(EMAC_RXINTMASKSET);
		tx_intmask = emac_get_reg(EMAC_TXINTMASKSET);

		/* Save MAC addresses */
		mac_ctx = (unsigned int *) kmalloc((sizeof(u32) << 1) * MAC_ADDR_NUM, GFP_KERNEL);
		for (i = 0; i < MAC_ADDR_NUM; i++) {
			emac_arch_set_mac_addr_index(i);
			mac_ctx[i << 1]       = emac_arch_get_mac_addr_high();
			mac_ctx[(i << 1) + 1] = emac_arch_get_mac_addr_low();
		}

		/* Reset the EMAC */
		if (netif_running(dev)) {
			emac_close(dev);
			emac_reset(dev, GEMAC_RESET_WARM);
			emac_open(dev);
		} else {
			emac_reset(dev, GEMAC_RESET_WARM);
		}

		/* Restore MAC addresses */
		for (i = 0; i < MAC_ADDR_NUM; i++) {
			emac_arch_set_mac_addr_index(i);
			emac_arch_set_mac_addr_high(mac_ctx[i << 1]);
			emac_arch_set_mac_addr_low(mac_ctx[(i << 1) + 1]);
		}
		kfree(mac_ctx);

		/* Restore state */
		emac_arch_set_hash1(hash1);
		emac_arch_set_hash2(hash2);
		emac_arch_set_promisc(hash2);
		emac_arch_set_control(hash2);

		emac_set_reg(EMAC_RXINTMASKSET, rx_intmask);
		emac_set_reg(EMAC_TXINTMASKSET, tx_intmask);
		emac_set_reg(EMAC_RXUNICASTSET, unicast);
	} else
		/* Only master core can reset the device */
		return -EBUSY;

	return 0;
}

/*
 * Reset the EMAC hardware.
 */
static int emac_reset(struct net_device *dev, int reset_mode)
{
	struct emac_private *ep = netdev_priv(dev);
	volatile u32        *preg;
	int                  i;

	if (!ep->slave) {

		if (reset_mode == GEMAC_RESET_COLD) {
			/* Reset integrated switch if any */
			emac_arch_reset_switch();

			/* Reset EMAC control Module */ 
			emac_arch_reset_ectl();
		}

		/* Reset EMAC */
		emac_setbit_reg(EMAC_SOFTRESET, EMAC_B_SOFTRST);

		/* Wait until reset has occured or timeout */
		for (i = 0; i <= HZ / 2; i++) {
			if (emac_get_reg(EMAC_SOFTRESET) == 0x0)
				break;
			if (i == HZ / 2)
				return 1;
			msleep_interruptible(1);
		}
	}

	/* Set interrupt pacing */
	emac_arch_set_pacing();

	if (!ep->slave) {
		/* Reset MAC, TX, RX Control */
		emac_set_reg(EMAC_MACCONTROL, 0);
		emac_set_reg(EMAC_TXCONTROL, 0);
		emac_set_reg(EMAC_RXCONTROL, 0);
		
		if (reset_mode == GEMAC_RESET_COLD) {
#ifdef EMAC_DO_INIT_MDIO
			/* MII/MDIO setup */
			mdio_init(emac_get_reg(EMAC_TXIDVER));
#endif

			/* Init HDPs to NULL for all channels */
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
			if (reset_mode == GEMAC_RESET_COLD) {
				preg = emac_addr_reg(EMAC_RXGOODFRAMES);
				for (i = 0; i < EMAC_NUM_STATREGS; i++)
					*preg++ = 0;
			}

			/* Clear all MAC addresses */
			emac_arch_clear_mac_addr();
		}
	}

	/* Register my MAC address */
	emac_register_mac_addr(dev, emac_idx);

	if (!ep->slave) { 

		/* Used for pause frame (IEEE 802.3x) but not working for MC */
		emac_set_reg(EMAC_MACSRCADDRLO,
			     (dev->dev_addr[0] << 8) | dev->dev_addr[1]);
		emac_set_reg(EMAC_MACSRCADDRHI,
			     (dev->dev_addr[2] << 24) | (dev->dev_addr[3] << 16) |
			     (dev->dev_addr[4] << 8)  | (dev->dev_addr[5]));

		if (reset_mode == GEMAC_RESET_COLD) {

			/* Init completion pointers to NULL */
			emac_set_reg(EMAC_RX0INTACK, 0x0);
			emac_set_reg(EMAC_RX1INTACK, 0x0);
			emac_set_reg(EMAC_RX2INTACK, 0x0);
			emac_set_reg(EMAC_RX3INTACK, 0x0);
			emac_set_reg(EMAC_RX4INTACK, 0x0);
			emac_set_reg(EMAC_RX5INTACK, 0x0);
			emac_set_reg(EMAC_RX6INTACK, 0x0);
			emac_set_reg(EMAC_RX7INTACK, 0x0);
			emac_set_reg(EMAC_TX0INTACK, 0x0);
			emac_set_reg(EMAC_TX1INTACK, 0x0);
			emac_set_reg(EMAC_TX2INTACK, 0x0);
			emac_set_reg(EMAC_TX3INTACK, 0x0);
			emac_set_reg(EMAC_TX4INTACK, 0x0);
			emac_set_reg(EMAC_TX5INTACK, 0x0);
			emac_set_reg(EMAC_TX6INTACK, 0x0);
			emac_set_reg(EMAC_TX7INTACK, 0x0);
		}

		/* Reset unicast, multicast, broadcast Rx */
		emac_reset_filter(dev, IDX_TO_CHAN(emac_idx), reset_mode);

		/* Set device modes */
		emac_set_modes(dev);

		/* Disable all channel interrupts */
		if (reset_mode == GEMAC_RESET_COLD) {
			emac_set_reg(EMAC_RXINTMASKCLEAR, 0xff);
			emac_set_reg(EMAC_TXINTMASKCLEAR, 0xff);
		}

		/* Enable host and stat interrupts */
		emac_set_reg(EMAC_MACINTMASKSET, EMAC_B_HOSTINT | EMAC_B_STATINT);
	} else {
		/* Fixup mode according to hw configuration */
		emac_fixup_modes(dev);
	}

	/* Enable TX and RX interrupts for a given channel */
	emac_arch_enable_irq(EMAC_TXINTMASKSET, IDX_TO_CHAN(emac_idx));
	emac_arch_enable_irq(EMAC_RXINTMASKSET, IDX_TO_CHAN(emac_idx));

	/* Reset transmit/receive buffers */
	if (reset_mode == GEMAC_RESET_WARM)
		(void) emac_reset_ring(ep);

	/* Set default receive filter: unicast, multicast and broadcast */
	emac_set_filter(dev, IDX_TO_CHAN(emac_idx));

	return 0;
}

/*
 * Open the device
 */
static int emac_open(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);

	if (!ep->slave) {
		/* Set MAC mode (Gigabyte, xxMII, pacing, etc) */
		emac_arch_init_control(ep->mode_flags & EMAC_CONFIG_PACING);

		/* Enable RX and TX */
		emac_set_reg(EMAC_TXCONTROL, 1);
		emac_set_reg(EMAC_RXCONTROL, 1);
	}

	/* Ready for RX */
	emac_set_reg(EMAC_RX0HDP + (IDX_TO_CHAN(emac_idx) << 2),
		     PTR_TO_HW(ep->cur_rx));

	/* Enable interrupts at the ECTL level (misc int only for master) */
	ectl_arch_enable_irq(IDX_TO_CHAN(emac_idx), !ep->slave);

	netif_start_queue(dev);

	return 0;
}

/*
 * Close the device
 */
static int emac_close(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);

	netif_stop_queue(dev);

	/* Disable interrupt at the ECTL level */
	ectl_arch_disable_irq(IDX_TO_CHAN(emac_idx));

	/* Teardown RX and TX */
	emac_set_reg(EMAC_RXTEARDOWN, IDX_TO_CHAN(emac_idx));
	emac_set_reg(EMAC_TXTEARDOWN, IDX_TO_CHAN(emac_idx));

	if (!ep->fatal_error) {
		unsigned int tmp;

		/* Wait for the teardown to complete */
		for(tmp = 0; tmp != 0xfffffffc;
		    tmp = emac_get_reg(EMAC_RX0INTACK + (IDX_TO_CHAN(emac_idx) << 2)))
			emac_set_reg(EMAC_RX0INTACK + (IDX_TO_CHAN(emac_idx) << 2),
				     tmp);
		
		for(tmp = 0; tmp != 0xfffffffc;
		    tmp = emac_get_reg(EMAC_TX0INTACK + (IDX_TO_CHAN(emac_idx) << 2)))
			    emac_set_reg(EMAC_TX0INTACK + (IDX_TO_CHAN(emac_idx) << 2),
					 tmp);
	}

	if (!ep->slave) {
		/* Disable Rx, Tx and MII */
		emac_set_reg(EMAC_TXCONTROL, 0);
		emac_set_reg(EMAC_RXCONTROL, 0);
		emac_set_reg(EMAC_MACCONTROL, 0);
	}

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
int emac_setup_ring(struct emac_private *ep)
{
	struct emac_desc *desc;
	int               i;
	struct sk_buff   *skb;

	/* Setup rx ring*/
	desc = (struct emac_desc *) ep->emac_dsc_base;
	ep->rx_desc_base = desc;
	ep->cur_rx       = desc;
	ep->skb_rx_dirty = 0;

	/* Allocate ring buffers via sk buffers */
	for (i = 0; i < RX_RING_SIZE; i++) {
		skb = netdev_alloc_skb_ip_align(ep->dev, ep->packet_mtu);
		if (!skb)
			return 1;

		ep->rx_skbuff[i]       = skb;
		desc->buff             = (u8 *) skb->data;
		desc->buff_offset_len  = ep->packet_mtu;
		desc->packet_flags_len = EMAC_DESC_FLAG_OWNER;
		desc->next             = PTR_TO_HW(desc + 1);
		desc++;
	}

	/* Loop the ring */
	(desc - 1)->next = PTR_TO_HW(ep->rx_desc_base);

	/* Setup tx ring */
	desc = (struct emac_desc *) (ep->emac_dsc_base + (RX_RING_SIZE << 4));
	ep->tx_desc_base = desc;
	ep->cur_tx       = desc;
	ep->dirty_tx     = desc;
	ep->head_tx      = desc;
	ep->skb_cur      = 0;
	ep->skb_tx_dirty = 0;
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
	struct emac_private *ep = netdev_priv(dev);

	printk(KERN_WARNING "%s: transmit timed out\n", dev->name);
	dev->stats.tx_errors++;
	dev->trans_start = jiffies;
	if (!ep->tx_full)
		netif_wake_queue(dev);
}

/*
 * Set the MAC address
 */
static int emac_set_mac_address(struct net_device *dev, void *p)
{
	struct emac_private *ep   = netdev_priv(dev);
	struct sockaddr     *addr = (struct sockaddr *) p;
	int i;
	
	/* Get new MAC address */
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	for (i = 0; i <= 5; i++)
		config.enetaddr[i] = dev->dev_addr[i] & 0xff;
	
	printk("%s: changing MAC address to ", dev->name);
	for (i = 0; i < 5; i++)
		printk("%02x:", dev->dev_addr[i]);
	printk("%02x\n", dev->dev_addr[5]);

	return emac_reconfigure_device(dev);
}

/* 
 * Set or clear the multicast filter for this adaptor.
 */
#ifdef EMAC_HAS_ALE_SUPPORT
static void emac_reset_filter(struct net_device *dev, 
			      int channel,
			      int reset_mode)
{
	struct emac_private *ep = netdev_priv(dev);
	int i;

	if (reset_mode == GEMAC_RESET_COLD) {
		/* Alloc mcast list */
		ep->mcast_infos = kmalloc(3 * EMAC_V_ALE_MAX_MCAST, GFP_KERNEL);
		/* Init mcast table */
		if (ep->mcast_infos) {
			memset(ep->mcast_infos, 0x0, 3 * EMAC_V_ALE_MAX_MCAST);
			/* Clear device multicast list */
			for (i = 0; i < EMAC_V_ALE_MAX_MCAST; i++) {
				emac_set_reg(EMAC_ALETBLW0, 0);
				emac_set_reg(EMAC_ALETBLW1, 0);
				emac_set_reg(EMAC_ALETBLW2, 0);
				emac_set_reg(EMAC_ALETBLCTL,
					     i | EMAC_B_ALE_WRITE_RDZ);
			}
		}
	}
	if ((reset_mode == GEMAC_RESET_WARM) && (ep->mcast_infos)) {
		/* Re-set device multicast list */
		for (i = 0; i < ep->mcast_valid_len; i++) {
			emac_set_reg(EMAC_ALETBLW0, ep->mcast_infos[3*i]);
			emac_set_reg(EMAC_ALETBLW1, ep->mcast_infos[3*i+1]);
			emac_set_reg(EMAC_ALETBLW2, ep->mcast_infos[3*i+2]);
			emac_set_reg(EMAC_ALETBLCTL,
				     i | EMAC_B_ALE_WRITE_RDZ);
		}
		/* Set mcast mode if needed */
		if (ep->mcast_valid_len)
			emac_setbit_reg(EMAC_ALEUNKNOWNVLAN,
					EMAC_B_REG_MCAST_FLOOD_ON);
	}
}

static void emac_set_modes(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);
	
	if (ep->mode_flags & EMAC_CONFIG_RXCRC) {
		ep->packet_mtu = PKT_MTU_CRC;
	} else
		ep->packet_mtu = PKT_MTU_NOCRC;
	
	/* If PASSERROR is set, enable both ERROR and short frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSERROR)
		emac_setbit_reg(EMAC_MACCONTROL,
				EMAC_B_RXCEFEN | EMAC_B_RXCSFEN);
	
	/* If PASSCONTROL is set, enable control frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSCONTROL)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_RXCMFEN);
	
	/* Set MAC loopback if requested */
	if(ep->mode_flags & EMAC_CONFIG_MACLOOPBACK)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_LOOPBACK);
}

static void emac_set_filter(struct net_device *dev, int channel)
{
	struct emac_private *ep = netdev_priv(dev);

	emac_set_reg(CPSW_SW_CTL, 0x000000F4);    /* Enable switch flow control */
	emac_set_reg(CPSW_RX_CH_MAP, 0x11110000); /* RX switch priority */
	
	emac_set_reg(EMAC_ALECONTROL, 0x80000004); /* ALE enable and bypass */
	emac_set_reg(EMAC_ALEPRESCALE, 0x0); /* no mcat/broadcast limit */
	emac_set_reg(EMAC_ALEUNKNOWNVLAN, 0x07030307); /* need it!!! */
	emac_set_reg(EMAC_ALEPORTCTL0, 0x3); /* Port 0 in forward state */
	emac_set_reg(EMAC_ALEPORTCTL1, 0x3); /* Port 1 in forward state */
	emac_set_reg(EMAC_ALEPORTCTL2, 0x3); /* Port 2 in forward state */
}

static void emac_set_rx_mode(struct net_device *dev)
{
	struct emac_private *ep  = netdev_priv(dev);
	struct dev_mc_list  *dmi = dev->mc_list;
	u8     hashval, tmpval;
	u32    info0, info1, info2;
	int    i, j;

	/* Only master can use promiscuous or multicast */
	if (ep->slave)
		return;

	if (dev->flags & IFF_PROMISC)
		/* Set promiscuous mode */
		emac_setbit_reg(EMAC_ALECONTROL, EMAC_B_ALEBYPASS);
	else
		/* Reset promiscuous mode */
		emac_clearbit_reg(EMAC_ALECONTROL, EMAC_B_ALEBYPASS);
	
	/* Clear multicast config */
	emac_clearbit_reg(EMAC_ALEUNKNOWNVLAN, EMAC_B_MCAST_FLOOD_ON |
			  EMAC_B_REG_MCAST_FLOOD_ON);
	
	/* Clear old multicast list that will not be reused */
	for (i = dev->mc_count; i < ep->mcast_valid_len; i++) {
		emac_setbit_reg(EMAC_ALETBLW0, 0);
		emac_setbit_reg(EMAC_ALETBLW1, 0);
		emac_setbit_reg(EMAC_ALETBLW2, 0);
		emac_setbit_reg(EMAC_ALETBLCTL,
				i | EMAC_B_ALE_WRITE_RDZ);
	}
	
 	if ((dev->flags & IFF_ALLMULTI) ||
	    (dev->mc_count > EMAC_V_ALE_MAX_MCAST)) {
		/* Set all multicast */
		emac_setbit_reg(EMAC_ALEUNKNOWNVLAN, EMAC_B_MCAST_FLOOD_ON |
				EMAC_B_REG_MCAST_FLOOD_ON);
	} else {
		/* Set mcast from a list */
		for (i = 0; i < dev->mc_count; i++, dmi = dmi->next) {
			u8 *p_dmi = dmi->dmi_addr;
						
			info0=(((p_dmi[2]&0x0ff)<<24) | ((p_dmi[3]&0x0ff)<<16) |
			       ((p_dmi[4]&0x0ff)<<8) | ((p_dmi[5]&0x0ff)<<0));
			info1=((3<<30) | (0<<29) | (1<<28) | ((0&0x0fff)<<16) |
			       ((p_dmi[0]&0x0ff)<<8) | ((p_dmi[1]&0x0ff)<<0));
			info2= (0<<3) | (7&0x7);
			
			/* Only support group multicast */
			if (!(*p_dmi & 1)) {
				/* don't use this entry */
				info0 = 0;
				info1 = 1;
				info2 = 2;
			}
			
			emac_set_reg(EMAC_ALETBLW0, info0);
			emac_set_reg(EMAC_ALETBLW1, info1);
			emac_set_reg(EMAC_ALETBLW2, info2);
			emac_set_reg(EMAC_ALETBLCTL,
					i | EMAC_B_ALE_WRITE_RDZ);
			
			/* Copy mcast list */
			if (ep->mcast_infos) {
				ep->mcast_infos[3*i]   = info0;
				ep->mcast_infos[3*i+1] = info1;
				ep->mcast_infos[3*i+2] = info2;
			}
		}
		
		/* set multicast mode if needed */
		if (dev->mc_count)
			emac_setbit_reg(EMAC_ALEUNKNOWNVLAN,
					EMAC_B_REG_MCAST_FLOOD_ON);
		
		/* Remember mcast valid addrs count */
		ep->mcast_valid_len = dev->mc_count;
	}
}

#else /* !EMAC_HAS_ALE_SUPPORT */

static void emac_reset_filter(struct net_device *dev, 
			      int channel,
			      int reset_mode)
{
	struct emac_private *ep = netdev_priv(dev);
	
	/* Reset multicast hash table */
	emac_set_reg(EMAC_MACHASH1, 0);
	emac_set_reg(EMAC_MACHASH2, 0);
		
	/* Set buffer offset */
	emac_set_reg(EMAC_RXBUFFEROFFSET, 0);

	/* Reset RX (M)ulticast (B)roadcast (P)romiscuous Enable register */
	emac_set_reg(EMAC_RXMBPENABLE, 0);

	if (reset_mode == GEMAC_RESET_COLD) {
		/* Clear unicast RX on channel 0-7 */
		emac_set_reg(EMAC_RXUNICASTCLEAR, 0xff);
	} else
		emac_set_reg(EMAC_RXUNICASTCLEAR, (1 << channel));
}

static void emac_fixup_modes(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);

	if (emac_get_reg(EMAC_RXMBPENABLE) & EMAC_B_RXPASSCRC)
		ep->packet_mtu = PKT_MTU_CRC;
	else
		ep->packet_mtu = PKT_MTU_NOCRC;
}

static void emac_set_modes(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);

	if (ep->slave)
		return;
	
	if (ep->mode_flags & EMAC_CONFIG_RXCRC) {
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXPASSCRC);
		ep->packet_mtu = PKT_MTU_CRC;
	} else
		ep->packet_mtu = PKT_MTU_NOCRC;
		
	/* If PASSERROR is set, enable both ERROR and short frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSERROR)
		emac_setbit_reg(EMAC_RXMBPENABLE,
				EMAC_B_RXCEFEN | EMAC_B_RXCSFEN);
	
	/* If PASSCONTROL is set, enable control frames */
	if (ep->mode_flags & EMAC_CONFIG_PASSCONTROL)
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCMFEN);
	
	/* Set the channel configuration to priority if requested */
	if (ep->mode_flags & EMAC_CONFIG_CHPRIORITY)
			emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_TXPTYPE);

	/* Set MAC loopback if requested */
	if(ep->mode_flags & EMAC_CONFIG_MACLOOPBACK)
		emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_LOOPBACK);
}

static void emac_set_filter(struct net_device *dev, int channel)
{
	struct emac_private *ep = netdev_priv(dev);
	
	/* Set unicast for the given channel */
	emac_set_reg(EMAC_RXUNICASTSET, 1 << channel);

	if (!ep->slave) {
		emac_set_reg(EMAC_RXMBPENABLE, 0);
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_MULTEN);
		emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_BROADEN);
		emac_setbit_reg(EMAC_RXMBPENABLE, (channel | (channel << 8)));
	}
}

static void emac_set_rx_mode(struct net_device *dev)
{
	struct emac_private *ep = netdev_priv(dev);
	struct dev_mc_list  *dmi;
	u8     hashval, tmpval;
	u32    machash1, machash2;
	int    i, j;

	/* Only master can use promiscuous or multicast */
	if (ep->slave)
		return;

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
				tmpval   = (u8) *p_dmi++;
				hashval ^= (u8) (tmpval >> 2) ^ (tmpval << 4);
				tmpval   = (u8) *p_dmi++;
				hashval ^= (u8) (tmpval >> 4) ^ (tmpval << 2);
				tmpval   = (u8) *p_dmi++;
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
#endif /* !EMAC_HAS_ALE_SUPPORT */

#ifdef EMAC_TIMER_TICK_MDIO
/*
 * This function should be called for each device in the system on a
 * periodic basis of 100mS (10 times a second). It is used to check the
 * status of the EMAC and MDIO device.
 */
static void emac_timer_tick(unsigned long data)
{
	struct net_device   *dev = (struct net_device *)data;
	struct emac_private *ep  = netdev_priv(dev);
	unsigned int link_status;
	unsigned int link_event = mdio_timer_tick();
	unsigned int intfmacsel = mdio_get_macsel();
	
	/*
	 * Signal the MDIO
	 */
	if (link_event == MDIO_EVENT_LINKUP) {
		link_status = mdio_get_status();
		if (link_status == MDIO_LINKSTATUS_FD10 ||
		    link_status == MDIO_LINKSTATUS_FD100 ||
		    link_status == MDIO_LINKSTATUS_FD1000 ) {
			emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_FULLDUPLEX);
			if (intfmacsel == DEVSTAT_MACSEL_RMII)
				emac_setbit_reg(EMAC_MACCONTROL,
						EMAC_B_RMIIDUPLEXMODE);
		} else {
			emac_clearbit_reg(EMAC_MACCONTROL, EMAC_B_FULLDUPLEX);
			if (intfmacsel == DEVSTAT_MACSEL_RMII)
				emac_clearbit_reg(EMAC_MACCONTROL,
						  EMAC_B_RMIIDUPLEXMODE);
		}

		if (link_status == MDIO_LINKSTATUS_FD1000)
			emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_GIG);

		if ((link_status == MDIO_LINKSTATUS_FD10 ||
		     link_status == MDIO_LINKSTATUS_HD10 ) &&
		    (intfmacsel == DEVSTAT_MACSEL_RMII) )
			/* Clear bit to set clock to 2.5 MHz */
			emac_clearbit_reg(EMAC_MACCONTROL, EMAC_B_RMIISPEED);

		if ((link_status == MDIO_LINKSTATUS_FD100 ||
		     link_status == MDIO_LINKSTATUS_HD100 ) &&
		    (intfmacsel == DEVSTAT_MACSEL_RMII) )
			/* Set bit to set clock to 25 MHz */
			emac_setbit_reg(EMAC_MACCONTROL, EMAC_B_RMIISPEED);

		/* eventually call platform hook for RMII out of reset */
		rmii_arch_fix();

		/* Put RGMII in forced link mode */
		if (intfmacsel == DEVSTAT_MACSEL_RGMII)
			emac_clearbit_reg(EMAC_MACCONTROL, EMAC_B_RGMIIEN);

#ifdef CONFIG_LSI_LOGIC
		mdio_lsi_linkstatus(link_status);
#endif
	}
	mod_timer(&emac_timer, jiffies + EMAC_TIMER_PERIOD);
}
#endif /* EMAC_TIMER_TICK_MDIO */

const char emac_driver_name[] = "EMAC";

static const struct net_device_ops gemac_netdev_ops = {
	.ndo_open		= emac_open,
	.ndo_stop		= emac_close,
	.ndo_start_xmit		= emac_start_xmit,
	.ndo_tx_timeout		= emac_timeout,
	.ndo_set_multicast_list	= emac_set_rx_mode,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_get_stats		= emac_get_stats,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= emac_poll,
#endif
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static const struct ethtool_ops gemac_ethtool_ops = {
};

static int __init emac_probe(struct platform_device *pdev)
{
	struct net_device      *netdev;
	struct emac_private    *ep;
	int                     res = -EINVAL;
	int                     i;
	int                     err;
#ifdef EMAC_ARCH_HAS_MAC_ADDR
	char                    hw_emac_addr[6];
#endif
	struct resource        *cur_res;
	static int              ndevs = 0;
	int                     irq   = 0;

	emac_idx = get_emac_idx();

	if (ndevs >= EMAC_MAX_INSTANCE) {
		printk("gemac: Invalid instance number: %d (max=%d)\n",
		       ndevs, EMAC_MAX_INSTANCE);
		return -EINVAL;
	}

	/* Allocate private information */
	netdev = alloc_etherdev(sizeof(struct emac_private));
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	ep      = netdev_priv(netdev);
	ep->dev = netdev;

	spin_lock_init(&ep->lock);

	/* Get irqs numbers */
	cur_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,"IRQ_SRC");
	if (cur_res)
		irq = cur_res->start;

	/* Get EMAC I/O addresses */
	ep->emac_reg_base = 0;
	ep->ectl_reg_base = 0;
	ep->emac_dsc_base = 0;
	ep->mdio_reg_base = 0;

	cur_res = platform_get_resource_byname(pdev, IORESOURCE_IO, "EMAC_REG_BASE");
	if (cur_res)
		ep->emac_reg_base = cur_res->start;

	cur_res = platform_get_resource_byname(pdev, IORESOURCE_IO, "ECTL_REG_BASE");
	if (cur_res)
		ep->ectl_reg_base = cur_res->start;

	cur_res = platform_get_resource_byname(pdev, IORESOURCE_IO, "EMAC_DSC_BASE");
	if (cur_res)
		ep->emac_dsc_base = cur_res->start 
			+ ((IDX_TO_CHAN(emac_idx) * QUEUE_DESC_NUM) << 4);

#ifdef EMAC_DO_INIT_MDIO
	cur_res = platform_get_resource_byname(pdev, IORESOURCE_IO, "MDIO_REG_BASE");
	if (cur_res)
		ep->mdio_reg_base = cur_res->start;
#endif

	if (!irq | !ep->emac_reg_base | !ep->ectl_reg_base | !ep->emac_dsc_base) {
		printk("%s: Enable to get hardware resources\n", netdev->name);
		goto error;
	}

	ep->mode_flags = config.flags;

	if (emac_check_shared_capability())
		ep->slave = emac_shared;

	netdev->base_addr = ep->emac_reg_base;
	netdev->dev_id    = pdev->id;
	
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
	/* Setup the device */
	res = emac_reset(netdev, GEMAC_RESET_COLD);
	if (res) {
		free_netdev(netdev);
		goto error;
	}

	/* driver system function */
	ether_setup(netdev);

	/* Set generic Ethernet operations */
	netdev->netdev_ops      = &gemac_netdev_ops;
	netdev->watchdog_timeo  = TX_TIMEOUT;
	netdev->ethtool_ops	= &gemac_ethtool_ops;

	platform_set_drvdata(pdev, netdev);

	/* Register Ethernet device */
	res = register_netdev(netdev);
	if (res) {
		printk("%s: Unable to register netdev\n", netdev->name);
		goto error;
	}

	/* Setup transmit/receive buffers */
	res = emac_setup_ring(ep);
	if (res) {
		printk("%s: Enable to allocate memory\n", netdev->name);
		goto error;
	}
	
#ifdef EMAC_TIMER_TICK_MDIO
	/* Set EMAC timer */
	init_timer(&emac_timer);
	emac_timer.function = emac_timer_tick;
	emac_timer.data = (unsigned long)netdev;
	mod_timer(&emac_timer, jiffies + EMAC_TIMER_PERIOD);
#endif
	
	/* Install our interrupt handler(s) */
#ifdef EMAC_HAS_SEPARATE_RXTX_IRQS
	if ((request_irq(irq, emac_rx_interrupt, _INTFLAG, "GEMAC RX", netdev) == 0) &&
	    (request_irq(irq + 1, emac_tx_interrupt, _INTFLAG, "GEMAC TX", netdev) == 0)) 
		netdev->irq = irq;
#else
	if (request_irq(irq, emac_interrupt, 0, "GEMAC", netdev) == 0)
		netdev->irq = irq;
#endif
	
	printk("%s: EMAC(%d) driver version 2.1 IRQ=%d queue=%d %s",
	       netdev->name, pdev->id, netdev->irq, IDX_TO_CHAN(emac_idx),
	       emac_shared ? "shared\n" : "\n");
	
	printk("%s: MAC address=", netdev->name);
	for (i = 0; i < 5; i++)
		printk("%02x:", netdev->dev_addr[i]);
#ifdef CONFIG_SGMII
	printk("%02x PHY=SGMII\n", netdev->dev_addr[i]);
#else
	printk("%02x PHY=%s%sMII\n", netdev->dev_addr[i],
	       ((mdio_get_macsel() & DEVSTAT_MACSEL_RMII) ? "R" : ""),
	       ((mdio_get_macsel() & DEVSTAT_MACSEL_GMII) ? "G" : ""));
#endif
	++ndevs;
	return 0;

 error:
	free_netdev(netdev);
	return res;
}

static struct platform_driver emac_driver = {
	.driver = {
		.name	= (char *) emac_driver_name,
		.owner	= THIS_MODULE,
	},
	.probe	= emac_probe,
};

static void __exit emac_cleanup(void)
{
	platform_driver_unregister(&emac_driver);
}
module_exit(emac_cleanup);

/*
 * Initialize GEMAC controller
 */
int emac_init(void)
{
	return platform_driver_register(&emac_driver);
}

module_init(emac_init);

