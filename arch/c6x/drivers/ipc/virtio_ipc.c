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

#include <linux/init.h>
#include <linux/types.h>
#include <linux/bootmem.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <asm/ipc-core.h>

#include "virtio_ipc.h"

/* 
 * 1 if we are the master core
 * This is detected if we are the first Linux kernel in DDR, not necessarily the core 0
 */
static int master_core = 0;

/*
 * Pointer to the SoC virtio device repository
 */
static u32 __virtio_ipc_devices_ptr __section(.virtio_ipc_dev);
static u32 *virtio_ipc_devices_ptr;

/*
 * Used to synchronize data when calling get_buf() from the virtio transport layer
 */
void arch_virtio_sync_data(u32 addr, u32 len)
{
	if (len == 0)
		return;

	DPRINTK("syncing data: start = 0x%x, len = %d, end = 0x%x\n", addr, len, addr + len);
	L2_cache_block_invalidate(addr, addr + len);
}

void arch_virtio_flush_data(u32 addr, u32 len)
{
	if (len == 0)
		return;

	DPRINTK("syncing data: start = 0x%x, len = %d, end = 0x%x\n", addr, len, addr + len);
	L2_cache_block_writeback(addr, addr + len);
}

#ifdef VIRTIO_USE_CACHED_RING
void arch_virtio_sync_ring(struct vring *vring)
{
	virtio_ipc_sync_ring(vring, CACHE_SYNC_RING_USED);
}

void arch_virtio_flush_ring(struct vring *vring)
{
	virtio_ipc_flush_ring(vring, CACHE_SYNC_RING_DESC);
}
#endif

static int add_virtqueue(u32 addr, u32 index, u32 num, u32 irq)
{
	struct ipc_vqconfig *vqc = get_vqconfig(addr, index);

	/* The address of the vring must be shared accross cores */		
#ifdef VIRTIO_USE_CACHED_RING
	u32 vq_addr = (u32) kzalloc(vring_size(num, VIRTIO_IPC_RING_ALIGN),
				    GFP_KERNEL);
#else
	u32 size    = vring_size(num, VIRTIO_IPC_RING_ALIGN);
	u32 vq_addr = (u32) dma_alloc_coherent(NULL, size, NULL,
					       GFP_KERNEL | GFP_DMA);
	if (!vq_addr) {
		EPRINTK("no coherent (DMA) memory available!!!\n");
		return -1;
	}

	memset((void*) vq_addr, 0, size);
#endif
	DPRINTK("vq_addr = 0x%x\n", vq_addr);
	vqc->address = set_vaddr(vq_addr);
	vqc->num     = num;
	vqc->irq     = irq;

	return 0;
}

/*
 * Setup the shared config space.
 * Do it once per SoC and only the master is allowed to do that
 */
static u32 virtio_ipc_set_config_space(void)
{
	u32 base, addr, dev;

	/* One page should be enough */
#ifdef VIRTIO_USE_CACHED_RING 
	base = __get_free_page(GFP_KERNEL);
#else
	base = (u32) dma_alloc_coherent(NULL, PAGE_SIZE, NULL, GFP_KERNEL | GFP_DMA);
	if (!base) {
		EPRINTK("no coherent (DMA) memory available!!!\n");
		return 0;
	}

#endif
	memset((void*) base, 0, PAGE_SIZE);

	addr = base;

	/* Register network bridges in the devices configuration space */
	for (dev = 0; dev < CORE_NUM; dev++) {
		struct ipc_device_desc   *desc = (struct ipc_device_desc*) addr;
		struct virtio_net_config *conf;

		desc->id         = dev;                             /* id */
		desc->type       = VIRTIO_ID_NET;
		desc->owner      = (get_coreid() + dev) % CORE_NUM; /* core owner*/
		desc->num_vq     = 2;
		desc->config_len = sizeof(struct virtio_net_config);
		desc->status     = 0;

		/* Network devices need a recv and a send queue */
		add_virtqueue((u32) &desc->config,
			      0, VIRTQUEUE_DESC_NUM, IPC_INT_NET);  /* input */
		add_virtqueue((u32) &desc->config,
			      1, VIRTQUEUE_DESC_NUM, IPC_INT_NET);  /* output */

		conf = (struct virtio_net_config *)
			virtio_ipc_device_get_config_info(desc); /* config info */

		conf->status = 0;

		conf->mac[0] = VIRTIO_IPC_NET_MAC0;
		conf->mac[1] = VIRTIO_IPC_NET_MAC1;
		conf->mac[2] = VIRTIO_IPC_NET_MAC2;
		conf->mac[3] = VIRTIO_IPC_NET_MAC3;
		conf->mac[4] = VIRTIO_IPC_NET_MAC4;
		conf->mac[5] = (dev << 1) & 0xff;

		addr += virtio_ipc_device_config_size(desc);

		if (addr >= base + PAGE_SIZE) {
			EPRINTK("the device config space is not big enough\n");
			return 0;
		}
	}

	/* Sync the config space to slave cores */
	DPRINTK("sync config space at 0x%x\n", base);
	virtio_ipc_sync_config(base);

	return base;
}

/*
 * Scan the shared config space.
 */
static int virtio_ipc_scan_config_space(u32 cf_base)
{
	u32 addr, dev;

	/* Sync the config space */
	DPRINTK("invalidate config space at 0x%x\n", cf_base);
	virtio_ipc_invalidate_config(cf_base);

	addr = cf_base;

	/* Register bridges in the devices configuration space */
	for (dev = 0; dev < CORE_NUM; dev++) {
		struct ipc_device_desc *desc = (struct ipc_device_desc*) addr;
		
		if (desc->type == VIRTIO_ID_NET)
			virtio_bridge_register_if(desc);

		addr += virtio_ipc_device_config_size(desc);
	}
	return 0;
}

/* 
 * This gets the device's feature bits.
 */
static u32 virtio_ipc_get_features(struct virtio_device *vdev)
{
	return VIRTIO_IPC_NET_FEATURES;
}

static void virtio_ipc_finalize_features(struct virtio_device *vdev)
{
	/* Give virtio_ring a chance to accept features. */
	vring_transport_features(vdev);
}

/*
 * Reading and writing elements in config info
 */
static void virtio_ipc_get(struct virtio_device *vdev, unsigned int offset,
			   void *buf, unsigned len)
{	
	struct virtio_ipc_device *dev  = to_virtio_ipc_dev(vdev);
	struct ipc_device_desc   *desc = dev->idev;

	BUG_ON(offset + len > desc->config_len);
	memcpy(buf, virtio_ipc_device_get_config_info(desc) + offset, len);
}

static void virtio_ipc_set(struct virtio_device *vdev, unsigned int offset,
			   const void *buf, unsigned len)
{
	struct virtio_ipc_device *dev  = to_virtio_ipc_dev(vdev);
	struct ipc_device_desc   *desc = dev->idev;

	BUG_ON(offset + len > desc->config_len);
	memcpy(virtio_ipc_device_get_config_info(desc) + offset, buf, len);
}

/*
 * The operations to get and set the status word just access
 * the status field of the device descriptor.
 */
static u8 virtio_ipc_get_status(struct virtio_device *vdev)
{
	struct virtio_ipc_device *dev = to_virtio_ipc_dev(vdev);
	
	DPRINTK("status = %d\n", dev->status);

	return dev->status;
}

static void virtio_ipc_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_ipc_device *dev = to_virtio_ipc_dev(vdev);

	dev->status = status;

	DPRINTK("status = %d\n", dev->status);
}

static void virtio_ipc_del_vq(struct virtqueue *vq)
{
	vring_del_virtqueue(vq);
}

static void virtio_ipc_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list)
		virtio_ipc_del_vq(vq);
}

/*
 * To reset the device. It will zero the status and all the
 * features.
 */
static void virtio_ipc_reset(struct virtio_device *vdev)
{
	struct virtio_ipc_device *dev = to_virtio_ipc_dev(vdev);

	dev->status = 0;

	DPRINTK("called\n");
}

/*
 * This routine finds the first virtqueue described in the configuration of
 * this device and sets it up.
 */
static struct virtqueue *virtio_ipc_find_vq(struct virtio_device *vdev,
					    unsigned index,
					    void (*callback)(struct virtqueue *vq),
					    const char *name)
{
	struct virtio_ipc_device *dev = to_virtio_ipc_dev(vdev);
	struct ipc_vqconfig      *vqc = get_vqconfig(&dev->idev->config, index);
	struct virtqueue         *_vq;
	struct vring_virtqueue   *vq;
	int err;
     
	/*
	 * OK, tell virtio_ring.c to set up a virtqueue now we know its size
	 * and we've got a pointer to its pages.
	 * This will alocate the struct virtqueue + vring + private 
	 * and create the vring content but it is not synced to other cores
	 */
	_vq = vring_new_virtqueue(vqc->num,
				  VIRTIO_IPC_RING_ALIGN,
				  vdev,
				  (void*) get_vaddr(vqc->address),
				  virtio_bridge_notify,
				  callback,
				  name);
	if (!_vq) {
		err = -ENOMEM;
		goto error;
	}

	/* 
	 * Sync the ring content for other cores
	 */
	vq = to_vvq(_vq);
	virtio_ipc_sync_ring(&vq->vring, CACHE_SYNC_RING_ALL);

	/*
	 * Set our net bridge interrupt (rx) handler
	 */
	if ((strcmp(name, "input") == 0) && (vdev->id.device == VIRTIO_ID_NET)) {
		ipc_core->ipc_request(virtio_bridge_interrupt, 0, IPC_INT_NET, _vq);
	}

	return _vq;

error:
	return ERR_PTR(err);
}

/*
 * Retrieve virtqueues
 */
static int virtio_ipc_find_vqs(struct virtio_device *vdev,
			       unsigned              nvqs,
			       struct virtqueue     *vqs[],
			       vq_callback_t        *callbacks[],
			       const char           *names[])
{
	struct virtio_ipc_device *dev = to_virtio_ipc_dev(vdev);
	int i = 0;

	DPRINTK("nvqs = %d\n", nvqs);

	/* We must have this many virtqueues */
	if (nvqs > dev->idev->num_vq)
		goto error;

	/* Retrieve the corresponding vq for this device */
	for (i = 0; i < nvqs; ++i) {
		vqs[i] = virtio_ipc_find_vq(vdev, i, callbacks[i], names[i]);
		if (IS_ERR(vqs[i]))
			goto error;
	}	

	return 0;

error:
	virtio_ipc_del_vqs(vdev);
	return PTR_ERR(vqs[i]);
}

/*
 * The virtio config ops for a device
 */
static struct virtio_config_ops virtio_ipc_vq_configspace_ops = {
	.get_features      = virtio_ipc_get_features,
	.finalize_features = virtio_ipc_finalize_features,
	.get               = virtio_ipc_get,
	.set               = virtio_ipc_set,
	.get_status        = virtio_ipc_get_status,
	.set_status        = virtio_ipc_set_status,
	.reset             = virtio_ipc_reset,
	.find_vqs          = virtio_ipc_find_vqs,
	.del_vqs           = virtio_ipc_del_vqs,
};

/*
 * The root device for the virtio_ipc devices.
 * This makes them appear as /sys/devices/virtio_ipc/0,1,2 not /sys/devices/0,1,2.
 */
static struct device *virtio_ipc_root;

/*
 * Adds a new device and register it with virtio.
 * Appropriate drivers are loaded by the device model.
 */
static void add_virtio_ipc_device(struct ipc_device_desc *d, unsigned int offset)
{
	/* virtio-net device */
	struct virtio_ipc_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		EPRINTK("cannot allocate virtio_ipc_device\n");
		return;
	}

	dev->vdev.dev.parent = virtio_ipc_root;
	dev->vdev.id.device  = d->type;
	dev->vdev.config     = &virtio_ipc_vq_configspace_ops;
	dev->idev            = d;

	/* Register the virtio device */
	if (register_virtio_device(&dev->vdev) != 0) {
		EPRINTK("failed to register virtio_device %u type %u\n",
			offset, d->type);
		kfree(dev);
	}
}

/*
 * virtio_ipc_scan_devices() simply iterates through the device page.
 * The type 0 is reserved to mean "end of devices".
 */
static void virtio_ipc_scan_devices(u32 devices_repo_ptr)
{
	unsigned int i;
	struct ipc_device_desc *d;

	for (i = 0; i < PAGE_SIZE; i += virtio_ipc_device_config_size(d)) {
		d =  (struct ipc_device_desc*) (devices_repo_ptr + i);
		
		if (d->type == 0)
			break;
		
		/* If we are owner of the device, add it to our repo */
		if (d->owner == get_coreid())
			add_virtio_ipc_device(d, i);
	}
}

/*
 * Init function for virtio
 */
static int __init virtio_ipc_devices_init(void)
{
	int rc;

	/* 
	 * Compute the virtio device repository
	 */
	virtio_ipc_devices_ptr = (u32*) get_master_vaddr(&__virtio_ipc_devices_ptr);
	DPRINTK("virtio_ipc_devices = 0x%x\n", virtio_ipc_devices_ptr);

	/*
	 * If we our kernel text is a the beginning of DDR2, we are the master core
	 */
	if ((u32) &__virtio_ipc_devices_ptr == (u32) virtio_ipc_devices_ptr)
		master_core = 1;

	/* 
	 * If we are the master core, allocate and initialize the shared 
	 * config space. It includes the vring data too.
	 */
	if (master_core) {
		*virtio_ipc_devices_ptr = virtio_ipc_set_config_space();
		if (*virtio_ipc_devices_ptr == 0)
			return -1;

		/* The shared config space pointer must be flushed too */
		L2_cache_block_writeback(*virtio_ipc_devices_ptr,
					 *virtio_ipc_devices_ptr + 4);
	}
		
	/* Scan the config space and eventually configure the client driver (i.e. bridges) */
	virtio_ipc_scan_config_space(*virtio_ipc_devices_ptr);

	virtio_ipc_root = root_device_register("virtio_ipc");
	if (IS_ERR(virtio_ipc_root)) {
		rc = PTR_ERR(virtio_ipc_root);
		EPRINTK("could not register virtio_ipc root device");
		return rc;
	}

	/* Register the different virtio_ipc devices reading the shared config space */
	virtio_ipc_scan_devices(*virtio_ipc_devices_ptr);

	printk("IPC: virtio backend registered");
	if (master_core)
		printk(" (master)");
	printk("\n");

	return 0;
}

/*
 * We do this after core stuff, but before the drivers.
 */
postcore_initcall(virtio_ipc_devices_init);
