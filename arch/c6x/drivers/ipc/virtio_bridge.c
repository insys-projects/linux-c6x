/*
 *  linux/arch/c6x/drivers/ipc/virtio_bridge.c - bridge driver for virtio net devices
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>

#include <asm/ipc-core.h>

#include "virtio_ipc.h"

/*
 * Maximal number of interfaces in a SoC
 * (must be norammly equal to max number of cores)
 */
#define VIRTIO_BRIDGE_MAX_IF CORE_NUM

struct bridge_if_desc {
	__u16        last_avail;
	__u16        last_added;
	__u16        core_id;
	__u16        __pad;
	struct vring vring;
};

DEFINE_MUTEX(bridge_lock);

#define get_local_if_id() (get_coreid())

static struct bridge_if_desc bridge_if[VIRTIO_BRIDGE_MAX_IF];

/*
 * Each buffer in the virtqueues is actually a chain of descriptors.  This
 * function returns the next descriptor in the chain, or vq->vring.num if we're
 * at the end.
 */
static unsigned int next_desc(struct vring_desc *desc,
			      unsigned int       i, 
			      unsigned int       max)
{
	unsigned int next;

	/* If this descriptor says it doesn't chain, we're done. */
	if (!(desc[i].flags & VRING_DESC_F_NEXT))
		return max;

	/* Check they're not leading us off end of descriptors. */
	next = desc[i].next;

	if (next >= max) {
		EPRINTK("desc next is %u\n", next);
		return -1;
	}

	return next;
}

/*
 * Called when desc have been transfered
 */
static void add_used(struct vring *vring, unsigned int head, int len)
{
	struct vring_used_elem *used;	

	mutex_lock(&bridge_lock);

	/*
	 * The virtqueue contains a ring of used buffers. Get a pointer to the
	 * next entry in that used ring.
	 */
	used      = &vring->used->ring[vring->used->idx % vring->num];	
	used->id  = head;
	used->len = len;

	/*
	 * We must flush twice to respect the semantic:
	 * 1 - update the used desc
	 * 2 - then update the used index to make point to the used desc
	 * This allow to be sure that desc has been correctly updated
	 */
	if (len)
		virtio_ipc_flush_ring(vring, CACHE_SYNC_RING_USED);

	vring->used->idx++;

	/* 
	 * Flush the caches for remote to update the used index now
	 */
	if (len)
		virtio_ipc_flush_ring(vring, CACHE_SYNC_RING_USED_IDX);

	mutex_unlock(&bridge_lock);
}

/*
 * This looks in the virtqueue for the first in or out available head desc
 */
static int get_next_head_desc(struct bridge_if_desc *bdesc,
			      unsigned int           out)
{
	struct vring_desc *desc;
	u16               *last_avail = &bdesc->last_avail;
	unsigned int       head;
	unsigned int       max = bdesc->vring.num;

	/* 
	 * Check if there are available buffers
	 */
	if (*last_avail == bdesc->vring.avail->idx) {
		DPRINTK("no available buffer %u %u\n",
			*last_avail, bdesc->vring.avail->idx);
		return max;
	}

	/* 
	 * Check it isn't doing very strange things with descriptor numbers.
	 */
	if ((u16)(bdesc->vring.avail->idx - *last_avail) > max) {
		EPRINTK("bridge moved used index from %u to %u\n",
			*last_avail, bdesc->vring.avail->idx);
		return -1;
	}

	mutex_lock(&bridge_lock);

	/*
	 * Grab the next descriptor number they're advertising
	 */
	head = bdesc->vring.avail->ring[*last_avail % max];

	/*
	 * Go to the next available head
	 */
	(*last_avail)++;
	
	/* If their number is silly, that's a fatal mistake. */
	if (head >= max) {
		EPRINTK("bridge says index %u is available\n", head);
		mutex_unlock(&bridge_lock);
		return -1;
	}

	desc = bdesc->vring.desc;

	DPRINTK("new head for 0x%x is now %d\n", (unsigned int) bdesc, head);

	/*
	 * Find the head of the wanted type of desc
	 */
	do {
		/* If this is an input descriptor */
		if (desc[head].flags & VRING_DESC_F_WRITE) {
			if (!out)
				goto found;
		} else {
			/*
			 * If it's an output descriptor, they're all supposed
			 * to come before any input descriptors.
			 */
			if (out) 
				goto found;
		}  
	} while ((head = next_desc(desc, head, bdesc->vring.num)) != bdesc->vring.num);

	/* We did no found any corresponding descriptor */
	mutex_unlock(&bridge_lock);
	return -1;

found:
	mutex_unlock(&bridge_lock);
	return (int) head;
}

/*
 * Determine the destination interface number for a given Ethernet packet
 */
static int get_dst_if_id(unsigned int addr)
{
	struct ethhdr *hdr = (struct ethhdr *) addr;
	unsigned int   if_id;

	DPRINTK("source " MAC_FMT "\n",
		hdr->h_source[0], hdr->h_source[1], hdr->h_source[2],
		hdr->h_source[3], hdr->h_source[4], hdr->h_source[5]);

	DPRINTK("destination " MAC_FMT "\n",
		hdr->h_dest[0],	hdr->h_dest[1], hdr->h_dest[2],
		hdr->h_dest[3],	hdr->h_dest[4],	hdr->h_dest[5]);

	/*
	 * Normally we should check the full destination MAC address
	 * to verify if it matches the VIRTIO_IPC_NET_MAC OUI,
	 * but we are lazy and we want to go fast...
	 */
	if_id = hdr->h_dest[5];

	/*
	 * We suppose it is broadcast
	 */
	if (if_id == 0xff)
		return VIRTIO_BRIDGE_MAX_IF;

	/*
	 * Interface numbers are 0, 2, ..., VIRTIO_BRIDGE_MAX_IF * 2
	 */
	if (if_id < (VIRTIO_BRIDGE_MAX_IF << 1))
		return (if_id >> 1);
	else
		return -1;
}

/*
 * Copy a head of buffers to the remote head(s)
 */
static int virtio_bridge_transfer(struct vring_virtqueue *vq,
				  struct bridge_if_desc  *src_bdesc)
{
	struct bridge_if_desc *dst_bdesc;
	struct vring_desc *src_desc = src_bdesc->vring.desc;
	struct vring_desc *dst_desc;
	unsigned int free = 0;
	unsigned int addr = 0;
	unsigned int tlen;
	unsigned int len;
	unsigned int i;
	unsigned int j;

	int src_head;
	int dst_head;
	int eth_head;
	int index = 0;
	int ret;
	int id, min_id, max_id;
	int k;
	u16 max = src_bdesc->vring.num;

	/* Get the next outcomming buffer from source ring */
	src_head = get_next_head_desc(src_bdesc, 1);

	/* Did we finish? */
	if (src_head == max)
		return 0;

	/* Error case */
	if (src_head == -1)
		return -1;

	/* Retrieve destination interface from destination MAC addr */
	if (src_desc[src_head].flags & VRING_DESC_F_NEXT) {
		eth_head = src_desc[src_head].next;
	} else {
		EPRINTK("Missing Ethernet frame\n");
		return -1;
	}
		
	id = get_dst_if_id(get_vaddr(src_desc[eth_head].addr));
	if (id == -1) {
		EPRINTK("Invalid Ethernet frame\n");
		return -1;
	}
	if (id == VIRTIO_BRIDGE_MAX_IF) {
		/* Broadcast case */
		min_id = 0;
		max_id = VIRTIO_BRIDGE_MAX_IF;
	} else {
		/* Unicast case */
		min_id = id;
		max_id = id + 1;
	}
		
	/* 
	 * Iterate for all interfaces
	 */
	for (k = min_id; k < max_id; k++) {
		
		tlen = 0;
		ret  = 0;

		if (k == get_local_if_id())
			continue;

		dst_bdesc = &bridge_if[k];
		if (dst_bdesc == NULL) {
			DPRINTK("no remote (%d) interface found\n", k);
			continue;
		}

		dst_desc = dst_bdesc->vring.desc;
		if (dst_desc == NULL) {
			DPRINTK("interface found %d not configured\n", k);
			continue;
		}

		/* Invalidate the destination input remote ring content from our caches */
		virtio_ipc_invalidate_ring(&dst_bdesc->vring, CACHE_SYNC_RING_DESC);

#if 1 /* Workaround to allow multiple interfaces without multi-queue support */
		
		/*
		 * Check if the last_avail index has moved behind our back due to
		 * the activity of other core interfaces.
		 * If so we are trying to update this index based on the used index 
		 * but this does not protect us against concurrent accesses.
		 */
		virtio_ipc_invalidate_ring(&dst_bdesc->vring, CACHE_SYNC_RING_USED_IDX);
		if (dst_bdesc->vring.used->idx > dst_bdesc->last_avail) {
			DPRINTK("moving last_avail from %d to %d for if %d\n",
				dst_bdesc->last_avail, dst_bdesc->vring.used->idx, k);
			dst_bdesc->last_avail = dst_bdesc->vring.used->idx;
		}
#endif

		/* Retrieve an available destination incomming buffer */
		dst_head = get_next_head_desc(dst_bdesc, 0);
	
		/* No more space in destination ring? */
		if (dst_head == max) {
			DPRINTK("destination ring full for %d\n", k);
			/* Skip this interface */
			continue;
		} 

		/* Error case */ 
		if (dst_head == -1) {
			return -1;
		}

		/* Number of total treated buffers */
		index = 0; 

		/* Source and destination desc indexes */
		i = src_head;
		j = dst_head;

		/* 
		 * Copy and aggregate incoming buffers in the destination .
		 * Will use EDMA or CPPI/QM later.
		 */
		do {
			if (index++ <= 1) {
				/* 
				 * The two first buffers are set in distinc buffers
				 */
				if (j == max) {
					/* No more desc */
					ret = -1;
					goto error;
				}

				free = dst_desc[j].len;
				addr = get_vaddr(dst_desc[j].addr);
			
				j =  next_desc(dst_desc, j, max);
			}
			
			/* Length of the descriptor to transfer*/
			len = src_desc[i].len;
		
			if ((index > 0) && (free >= len) && (len)) {
				/* 
				 * Copy the payload data and aggregate them in the second buf.
				 * Do not need to flush copied data as destination address
				 * space is not cached
				 */
				memcpy((void*) addr, (void*) get_vaddr(src_desc[i].addr), len);
			} else {
				EPRINTK("not enough space ! i = %d, j = %d, free = %d, len = %d\n",
					i, j, free, len);
				ret = -1;
				goto error;
			}
			
			addr += len;
			tlen += len;
			free -= len;

		} while ((i = next_desc(src_desc, i, max)) != max);

       		DPRINTK("%d bytes transfered\n", tlen);

		/* Finished with the destination */
		add_used(&dst_bdesc->vring, dst_head, tlen);

		/* Signal the destination core for this packet */
		ipc_core->ipc_send(dst_bdesc->core_id, IPC_INT_NET);
	}

	/* Finished with the source */
	add_used(&src_bdesc->vring, src_head, 0); /* 0 length to avoid syncing data */

	return 0;
error:
	return ret;
}

/*
 * Invalidate new added buffers after being kicked (so ready to receive from remote)
 */
static int virtio_sync_buffers(struct vring_virtqueue *vq,
			       u16                    *last_added)
{
	struct vring_desc *desc;
	unsigned int       max        = vq->vring.num;
	unsigned int       first      = 1;
	unsigned int       index;

	/* 
	 * Check if there are available buffers
	 */
	if (*last_added == vq->vring.avail->idx) {
		DPRINTK("no available buffer %u %u\n",
			*last_added, vq->vring.avail->idx);
		return 0;
	}

	DPRINTK("new buffer to sync, last_added = %d, idx = %d\n",
		*last_added, vq->vring.avail->idx);

	mutex_lock(&bridge_lock);
	
	/* Get the next incomming buffer from ring */
	index = vq->vring.avail->ring[*last_added % max];

	/* Go to the next available desc */
	(*last_added)++;

	/* If their number is silly, that's a fatal mistake */
	if (index >= max) {
		EPRINTK("bridge says index %u is available\n", index);
		mutex_unlock(&bridge_lock);
		return -1;
	}

	desc = vq->vring.desc;
	
	/* Look into new set of descriptors */
	do {
		/* Find incomming buffers */
		if ((desc[index].flags & VRING_DESC_F_WRITE)
		    && (desc[index].len != 0)
		    && (!first)) {
			/*
			 * Invalidate the buffer content
			 */ 
			L2_cache_block_invalidate((u32) desc[index].addr,
						  (u32) desc[index].addr
						  + desc[index].len);
		}
		/*
		 * Do not flush the first buffer as it contains GSO info which 
		 * are not used and are not aligned on cache lines.
		 */
		first = 0;
		
	} while ((index = next_desc(desc, index, max)) != max);

	mutex_unlock(&bridge_lock);
	return 1;
}

/*
 * When the output vq is notified: meaning probably a transfer to perform
 */
static void virtio_output_vq_notify(struct vring_virtqueue *vq)
{
	struct bridge_if_desc  *src_bdesc;
	int                     ret;

	src_bdesc = &bridge_if[get_local_if_id()];

	/* Perform the buffer transfer */
	while(1) {
		/* Do the transfer for a set of available buffers  */
		ret = virtio_bridge_transfer(vq, src_bdesc);
		if (ret <= 0)
			break;
	}

	if (ret == 0) {
		if (!(vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT))
			/* Local transmit ACK */
			vring_interrupt(0, &vq->vq);
	}
}

/*
 * When the input vq is notified: need to sync the new buffers
 */
static void virtio_input_vq_notify(struct vring_virtqueue *vq)
{
	struct bridge_if_desc  *bdesc;
	int                     ret;

	bdesc = &bridge_if[get_local_if_id()];

	/* Perform the buffer synchronization */
	while(1) {
		ret = virtio_sync_buffers(vq, &bdesc->last_added);
		if (ret <= 0)
			break;
	}

	/* Flush the ready-to-receive input ring content from our caches */
	virtio_ipc_flush_ring(&vq->vring, CACHE_SYNC_RING_DESC);
}

/*
 * Bridge interrupt handler for the input vq
 */
void virtio_bridge_interrupt(u32 ipc_num, void *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	
	if (ipc_num == IPC_INT_NET) {
		/* Sync our rx ring */
		virtio_ipc_invalidate_ring(&vq->vring, CACHE_SYNC_RING_USED);
		
		/* Call virtio vring interrupt handler */		
		if (!(vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT))
			vring_interrupt(ipc_num, _vq);
	} 
}
EXPORT_SYMBOL(virtio_bridge_interrupt);

/*
 * When the virtio_ring code wants to notify the bridge, it calls us here
 * We hand the address of the virtqueue to know which virtqueue we're talking about.
 */
void virtio_bridge_notify(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq  = to_vvq(_vq);

	DPRINTK("called with source vq %s (0x%x)\n",
		vq->vq.name, (unsigned int) &vq->vq);

	/* Case of an output vq */
	if (strcmp(vq->vq.name, "output") == 0)
		virtio_output_vq_notify(vq);
	else
		virtio_input_vq_notify(vq);
}

EXPORT_SYMBOL(virtio_bridge_notify);

/*
 * Register a network interface: it can be a remote or local
 * We use this to set the bridge routing table.
 */
int virtio_bridge_register_if(struct ipc_device_desc *desc)
{
	struct bridge_if_desc *bdesc = &bridge_if[desc->id];

	if (desc->id == get_local_if_id()) {
		/* The output vq is the second one */
		struct ipc_vqconfig *vqc = get_vqconfig(&desc->config, 1);

		/* Initialize the local vring structure with output ring */
		vring_init(&bdesc->vring,
			   vqc->num,
			   (void*) get_vaddr(vqc->address),
			   VIRTIO_IPC_RING_ALIGN);
	} else {
		/* The input vq is the first one */
		struct ipc_vqconfig *vqc = get_vqconfig(&desc->config, 0);

		/* Initialize the remote vring structure with input ring */
		vring_init(&bdesc->vring,
			   vqc->num,
			   (void*) get_vaddr(vqc->address),
			   VIRTIO_IPC_RING_ALIGN);
	}

	bdesc->core_id    = desc->owner;
	bdesc->last_avail = 0;
	bdesc->last_added = 0;

	return 0;
}
EXPORT_SYMBOL(virtio_bridge_register_if);
