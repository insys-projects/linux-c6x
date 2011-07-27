/*
 *  linux/arch/c6x/drivers/ipc/virtio_ipc.c - IPC driver for virtio devices
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

#ifndef _C6X_VIRTIO_IPC_H
#define _C6X_VIRTIO_IPC_H

#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_net.h>

#include <asm/setup.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/cache.h>

#include <mach/board.h>

#define EPRINTK(fmt, args...) printk(KERN_ERR "[%s] " fmt, __FUNCTION__ , ## args)
#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "[%s] " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define FEATURE_SET(f) (1 << (f))

#define VIRTIO_IPC_NET_FEATURES (FEATURE_SET(VIRTIO_F_NOTIFY_ON_EMPTY)	\
				 | FEATURE_SET(VIRTIO_NET_F_MAC))

#define VIRTIO_IPC_NET_MAC0 0x00;
#define VIRTIO_IPC_NET_MAC1 0x01;
#define VIRTIO_IPC_NET_MAC2 0x01; /* This is a private OUI */
#define VIRTIO_IPC_NET_MAC3 0x00;
#define VIRTIO_IPC_NET_MAC4 0x00;

#define VIRTQUEUE_DESC_NUM      256

#define get_vaddr(a)            ((__u32)((a) & 0xffffffff))
#define set_vaddr(a)            ((__u64)(a))
#define get_master_vaddr(a)	((int) (a) - PAGE_OFFSET_RAW + RAM_DDR2_CE0)

/* 
 * The alignment to use between consumer and producer parts of vring.
 */
#define VIRTIO_IPC_RING_ALIGN	L2_CACHE_BYTES

/*
 * These structures define the different virtio_ipc devices followed by their
 * corresponding virtqueues. Mostly inspired from lguest/kvm but simplified.
 */
struct ipc_device_desc {
	/* unique Id */
	__u8 id;
	/* The device type: console, network, disk etc.  Type 0 terminates. */
	__u8 type;
	/* The core which owns the device */
	__u8 owner;
	/* The number of virtqueues (first in config array) */
	__u8 num_vq;
	/* The number of bytes of the config info after virtqueues. */
	__u8 config_len;
	/* A status byte . */
	__u8 status;
	/* Padding */
	__u8 __pad[2];
	/* pointers to the vqconfig array */
	__u8 config[0];
};

/*
 * This is how we expect the device configuration field for a virtqueue
 * to be laid out in config space.
 */
struct ipc_vqconfig {
	/* The address of the virtio ring */
	__u64 address;
	/* The number of entries in the virtio_ring */
	__u16 num;
	/* The IPC interrupt associated to this queue */
	__u16 irq;
};

#define get_vqconfig(a, i) (struct ipc_vqconfig *)((u32) (a) + ((i) * sizeof(struct ipc_vqconfig)))

/*
 * Return the location of the config info structure
 */
static inline char *virtio_ipc_device_get_config_info(const struct ipc_device_desc *desc)
{
	return ((char *)desc  + sizeof(*desc) + desc->num_vq * sizeof(struct ipc_vqconfig));
}

/*
 * The total size of the device config used by this device (incl. desc) aligned on cache lines
 */
static inline unsigned virtio_ipc_device_config_size(const struct ipc_device_desc *desc)
{
	return L2_CACHE_ALIGN_UP(sizeof(*desc)
				 + desc->num_vq * sizeof(struct ipc_vqconfig)
				 + desc->config_len);
}

/*
 * This must be accordance with the beginning of the vring_virtqueue structure
 * defined in the transport layer (virtio_ring.c). It allows to quickly retrieve 
 * the vring structure from the virtqueue.
 */
struct vring_virtqueue
{
	struct virtqueue          vq;
	struct vring              vring;
};

#define to_vvq(_vq) container_of(_vq, struct vring_virtqueue, vq)

/*
 * Our ipc_device structure
 */
struct virtio_ipc_device {
	struct virtio_device    vdev;
	struct ipc_device_desc *idev;
	u8                      status;
};

#define to_virtio_ipc_dev(vd) container_of(vd, struct virtio_ipc_device, vdev)

extern int master_core;

/*
 * Caches synchronization primitives (if needed)
 */
#define VIRTIO_USE_CACHED_RING

#ifdef VIRTIO_USE_CACHED_RING
/*
 * Cache coherency synchronization primitives
 */
#define CACHE_SYNC_RING_DESC     0
#define CACHE_SYNC_RING_USED     1
#define CACHE_SYNC_RING_USED_IDX 2
#define CACHE_SYNC_RING_ALL      3

#define __VIRTIO_RING_SET_START_END()					\
	switch (how) {							\
	case CACHE_SYNC_RING_DESC:					\
		start = (u32) vring->desc;				\
		end   = (u32) vring->used;				\
		break;							\
	case CACHE_SYNC_RING_USED:					\
		start = (u32) vring->used;				\
		end   = (u32) start					\
			+ vring_size(vring->num, VIRTIO_IPC_RING_ALIGN) \
			- ((u32)vring->used - (u32)vring->desc);	\
		break;							\
	case CACHE_SYNC_RING_USED_IDX:					\
		start = (u32) vring->used;				\
		end   = (u32) &vring->used->ring;			\
		break;							\
	case CACHE_SYNC_RING_ALL:					\
		start = (u32) vring->desc;				\
		end   = (u32) start + vring_size(vring->num, VIRTIO_IPC_RING_ALIGN); \
		break;							\
	}								\
	DPRINTK("%s start = 0x%x, end = 0x%x\n", (how == CACHE_SYNC_RING_DESC) ? "desc" : \
		((how == CACHE_SYNC_RING_USED) ? "used" : "all"),	\
		start, end)
	
static inline void virtio_ipc_flush_ring(struct vring *vring, int how) 
{
	if (master_core) {
		u32 start, end;
		__VIRTIO_RING_SET_START_END();
		L2_cache_block_writeback(start, end);
	}
}

static inline void virtio_ipc_invalidate_ring(struct vring *vring, int how) 
{
	if (master_core) {
		u32 start, end;
		__VIRTIO_RING_SET_START_END();
		L2_cache_block_invalidate(start, end);
	}
}

static inline void virtio_ipc_sync_ring(struct vring *vring, int how) 
{
	if (master_core) {
		u32 start, end;
		__VIRTIO_RING_SET_START_END();
		L2_cache_block_writeback_invalidate(start, end);
	}
}

#define virtio_ipc_flush_config(base)       L2_cache_block_writeback((base), (base) + PAGE_SIZE);
#define virtio_ipc_invalidate_config(base)  L2_cache_block_invalidate((base), (base) + PAGE_SIZE);
#define virtio_ipc_sync_config(base)        L2_cache_block_writeback_invalidate((base), (base) + PAGE_SIZE);
#else
#define virtio_ipc_flush_ring(vring, h) 
#define virtio_ipc_invalidate_ring(vring, h) 
#define virtio_ipc_sync_ring(vring, h) 
#define virtio_ipc_flush_config(base)
#define virtio_ipc_invalidate_config(base)
#define virtio_ipc_sync_config(base)
#endif

/*
 * IPC interrupt mapping (static)
 */
#define IPC_INT_MAINT      0
#define IPC_INT_NET        1

/*
 * Bridge functions
 */
extern void virtio_bridge_notify(struct virtqueue *_vq);
extern int  virtio_bridge_register_if(struct ipc_device_desc *desc);
extern void virtio_bridge_interrupt(u32 ipc_num, void *_vq);

/*
 * IPC functions
 */
int tci648x_ipc_request(void (*handler)(u32, void*),
			u32 iflags,
			u32 ipc_num,
			void *data);
int tci648x_ipc_free(u32 ipc_num);
int tci648x_ipc_send(int core_id, int ipc_num);
int tci648x_ipc_wait(int ipc_num);

#endif
