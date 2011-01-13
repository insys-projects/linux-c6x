/*
 *  linux/arch/c6x/drivers/ipc/virtio_bridge.c - bridge driver for virtio net devices
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
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

#include <asm/ipc-core.h>

#include "virtio_ipc.h"

/*
 * Maximal number of interfaces in a SoC
 * (must be mostly equal to max number of cores)
 */
#define VIRTIO_BRIDGE_MAX_IF  32

struct bridge_if_desc {
	__u16        last_avail;
	__u16        core_id;
	__u16        __pad;
	struct vring vring;
};

DEFINE_MUTEX(bridge_lock);

/* AJ: this must be fixed */
#define get_local_if_id() (get_coreid())
#define get_dst_if_id()   ((get_coreid() + 1) % 2)

static struct bridge_if_desc bridge_if[VIRTIO_BRIDGE_MAX_IF];

/*
 * Each buffer in the virtqueues is actually a chain of descriptors.  This
 * function returns the next descriptor in the chain, or vq->vring.num if we're
 * at the end.
 */
static unsigned next_desc(struct vring_desc *desc,
			  unsigned int i, unsigned int max)
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

	/*
	 * The virtqueue contains a ring of used buffers. Get a pointer to the
	 * next entry in that used ring.
	 */
	used      = &vring->used->ring[vring->used->idx % vring->num];
	used->id  = head;
	used->len = len;

	vring->used->idx++;
}

/*
 * This looks in the virtqueue for the first in and out available buffers
 * This function changes the out_desc and in_desc pointers to the first out 
 * and in descriptors and returns 1 if at least a descriptor is found.
 */
static int get_vq_desc(struct bridge_if_desc  *bdesc,
		       struct vring_desc     **out_desc,
		       struct vring_desc     **in_desc,
		       unsigned int           *out_num,
		       unsigned int           *in_num,
		       unsigned int           *head)
{
	unsigned int       i, max;
	struct vring_desc *desc;
	u16               *last_avail = &bdesc->last_avail;

	/* 
	 * Check if there are available buffers
	 */
	if (*last_avail == bdesc->vring.avail->idx) {
		DPRINTK("no available buffer %u %u\n",
			*last_avail, bdesc->vring.avail->idx);
		return 0;
	}

	/* Check it isn't doing very strange things with descriptor numbers. */
	if ((u16)(bdesc->vring.avail->idx - *last_avail) > bdesc->vring.num) {
		EPRINTK("bridge moved used index from %u to %u\n",
			*last_avail, bdesc->vring.avail->idx);
		return -1;
	}

	/*
	 * Grab the next descriptor number they're advertising
	 */
	*head = bdesc->vring.avail->ring[*last_avail % bdesc->vring.num];
	(*last_avail)++;
	
	/* If their number is silly, that's a fatal mistake. */
	if (*head >= bdesc->vring.num) {
		EPRINTK("bridge says index %u is available\n", *head);
		return -1;
	}
	
	/* When we start there are none of either input nor output. */
	*out_num = *in_num = 0;
	
	max  = bdesc->vring.num;
	desc = bdesc->vring.desc;
	i    = *head;
	
	DPRINTK("new head is now %d\n", *head);

	/*
	 * If this is an indirect entry, then this buffer contains a descriptor
	 * table which we handle as if it's any normal descriptor chain.
	 */
	if (desc[i].flags & VRING_DESC_F_INDIRECT) {
		if (desc[i].len % sizeof(struct vring_desc)) {
			EPRINTK("invalid size for indirect buffer table\n");
			return -1;
		}
		max  = desc[i].len / sizeof(struct vring_desc);
		desc = (struct vring_desc *) get_vaddr(desc[i].addr);
		i    = 0;
	}
	
	do {
                /* If this is an input descriptor, get it and increment that count. */
		if (desc[i].flags & VRING_DESC_F_WRITE) {
			(*in_num)++;
			if (in_desc)
				(*in_desc++) = &desc[i];
		} else {
			/*
			 * If it's an output descriptor, they're all supposed
			 * to come before any input descriptors.
			 */
			if (*in_num) {
				EPRINTK("descriptor has out after in\n");
				return -1;
			}
			(*out_num)++;
			if (out_desc)
				(*out_desc++) = &desc[i];
		}
		
		/* If we've got too many, that implies a descriptor loop. */
		if (*out_num + *in_num > max) {
			EPRINTK("looped descriptor\n");
			return -1;
		}
	} while ((i = next_desc(desc, i, max)) != max);

	/* We found a descriptor */
	return 1;
}

/*
 * Bridge interrupt handler
 */
void virtio_bridge_interrupt(u32 ipc_num, void *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	
	if ((ipc_num == IPC_INT_NET) 
	    && (!(vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT))) {
		
		/* Sync our rx ring */
		virtio_ipc_sync_ring(&vq->vring, CACHE_SYNC_RING_DESC);
		
		/* Call virtio vring interrupt handler */
		vring_interrupt(ipc_num, _vq);
	}
}
EXPORT_SYMBOL(virtio_bridge_interrupt);

static struct vring_desc *desc_src[VIRTQUEUE_NUM];
static struct vring_desc *desc_dst[VIRTQUEUE_NUM];

/*
 * Bridge transfer
 */
static int virtio_bridge_transfer(struct vring_virtqueue *vq,
				  struct bridge_if_desc  *src_bdesc,
				  struct bridge_if_desc  *dst_bdesc)
{
	unsigned int       head_src, head_dst, out_src, out, in;
	unsigned int       free = 0, len, tlen = 0;
	unsigned  int      addr = 0;
	int                ret, i;

	/* Retrieve a desc to send */
	ret = get_vq_desc(src_bdesc, desc_src, NULL, &out_src, &in, &head_src);
	if (in) {
		EPRINTK("input buffers in net output queue?\n");
		return -1;
	}

	/* No more desc */
	if (ret <= 0)
		return ret; 

	/* Invalidate the destination input remote ring content in our caches */
	virtio_ipc_sync_ring(&dst_bdesc->vring, CACHE_SYNC_RING_DESC);

	/* Retrieve a destination available desc */
	ret = get_vq_desc(dst_bdesc, NULL, desc_dst, &out, &in, &head_dst);
	if (out) {
		EPRINTK("output buffers in net input queue?\n");
		return -1;
	}
	if (ret == 0) {
		EPRINTK("destination ring full\n");
		return -1;
	} 
	if (ret < 0)
		return ret;

	DPRINTK("get %d desc\n", out_src);

	/* 
	 * Copy and aggregate incoming buffers.
	 * Will use EDMA or CPPI later.
	 */
	for (i = 0; i < out_src; i++) {
		if (i <= 1) {
			free = desc_dst[i]->len;
			addr = get_vaddr(desc_dst[i]->addr);
		}

		/* Length of the descriptor to transfer*/
		len = desc_src[i]->len;

		if ((free >= len) && (len)) {
			/* 
			 * Copy the payload data.
			 * Do not need to flush copied data as destination address 
			 * space is not cached
			 */
			memcpy((void*) addr, (void*) get_vaddr(desc_src[i]->addr), len);
		} else
			EPRINTK("not enough space ! i = %d, free = %d, len = %d\n", i, free, len);
	       
		addr += len;
		tlen += len;
		free -= len;
	}
	DPRINTK("%d bytes transfered\n", tlen);

	/*
	 * Done with these ones
	 */
	add_used(&dst_bdesc->vring, head_dst, tlen);
	add_used(&src_bdesc->vring, head_src, 0);     /* to avoid syncing data */

	/* Flush the modified destination input remote ring content from our caches */
	virtio_ipc_flush_ring(&dst_bdesc->vring, CACHE_SYNC_RING_USED);

	return 0;
}

/*
 * When the output vq is notified: meaning probably a transfer to perform
 */
void virtio_output_vq_notify(struct vring_virtqueue *vq)
{
	struct bridge_if_desc  *src_bdesc;
	struct bridge_if_desc  *dst_bdesc;
	int                     ret;

	/*
	 * Find the source and destination bdesc
	 */
	src_bdesc = &bridge_if[get_local_if_id()];
	dst_bdesc = &bridge_if[get_dst_if_id()];

	if (dst_bdesc->vring.desc == NULL) {
		EPRINTK("no remote (%d) interface found\n", get_dst_if_id());
		return;
	}

	/* Perform the buffer transfer */
	while(1) {

		mutex_lock(&bridge_lock);
		ret = virtio_bridge_transfer(vq, src_bdesc, dst_bdesc);
		mutex_unlock(&bridge_lock);

		if (ret <= 0)
			break;
	}

	if (ret == 0) {
		/* Signal the destination core */
		ipc_core->ipc_send(dst_bdesc->core_id, IPC_INT_NET);

		if (!(vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT))
			/* Local transmit ACK */
			vring_interrupt(0, &vq->vq);
	}
}

/*
 * When the virtio_ring code wants to notify the bridge, it calls us here
 * We hand the address of the virtqueue to know which virtqueue we're talking about.
 */
void virtio_bridge_notify(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq  = to_vvq(_vq);

	DPRINTK("called with source vq %s (0x%x)\n", vq->vq.name, &vq->vq);

	/* Case of an output vq */
	if (strcmp(vq->vq.name, "output") == 0)
		virtio_output_vq_notify(vq);
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

	return 0;
}
EXPORT_SYMBOL(virtio_bridge_register_if);
