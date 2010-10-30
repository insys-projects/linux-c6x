/*
 * RapidIO userspace interface using /dev 
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_ids.h>
#include <linux/rio_regs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <asm/virtconvert.h>
#include <asm/uaccess.h>

#include "rio.h"

/*
 * This supports acccess to RapidIO devices using normal userspace I/O calls.
 *
 * RapidIO has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask. You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/rapidio/0/1 device
 * nodes, since there is no fixed association of minor numbers with any
 * particular RapidIO site or device.
 */
#define RIO_DEV_MAJOR 	154	/* assigned */
#define RIO_DEV_NAME    "rio"
#define N_RIO_MINORS	32	/* number of minors per instance (up to 256) */

static unsigned long minors[N_RIO_MINORS / BITS_PER_LONG];

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...) 
#endif

#ifdef CONFIG_MMU
#define USE_COPY 1
#else
#define USE_COPY 0
#endif

/*
 * RapidIO File ops
 */
static loff_t rio_dev_llseek(struct file* filp, loff_t off, int whence)
{
	loff_t new;

	switch (whence) {
	case 0:	 new = off; break;
	case 1:	 new = filp->f_pos + off; break;
	case 2:	 new = 1 + off; break;
	default: return -EINVAL;
	}

	return (filp->f_pos = new);
}

static ssize_t rio_dev_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct rio_dev	        *rdev = filp->private_data;
	int                      res; 
	size_t                   count = size;
	size_t                   write_sz;
	char                    *src_buf;
	char                    *p;
	int                      copy;
	int                      write_pos = 0;

	if (!size)
		return 0;

	if (!rdev->net->hport->ops->transfer)
		return -EPROTONOSUPPORT;

	DPRINTK("write buffer, p = 0x%x, size = %d\n", buf, size);

	/* try to do zero-copy is buffer has the right property */
	if ((virt_to_phys(buf) != L1_CACHE_ALIGN(virt_to_phys(buf))) ||
	    (size < L1_CACHE_BYTES) || (USE_COPY)) {
		int alloc_size = (size > RIO_MAX_DIO_CHUNK_SIZE) ?
			RIO_MAX_DIO_CHUNK_SIZE : size;

		p = src_buf = kmalloc(L1_CACHE_ALIGN(alloc_size), GFP_KERNEL);

		if (!p) {
			res = -ENOMEM;
			goto out;
		}

		copy = 1;
		
		/* if allocated buffer is still non-aligned on cache */
		if ((unsigned int) p != L1_CACHE_ALIGN((unsigned int) p)) {
			res = -ENOMEM;
			goto out;
		}	
		DPRINTK("Allocating write buffer, p = 0x%x, size = %d\n", p, L1_CACHE_ALIGN(size));

	} else {
		p    = src_buf = buf; /* zero-copy case */
		copy = 0;
	}

	while(count) {
		if (copy) {
			write_sz = (count <= RIO_MAX_DIO_CHUNK_SIZE) ? count : RIO_MAX_DIO_CHUNK_SIZE;
			copy_from_user(p, buf + write_pos, write_sz);
		} else
			write_sz = count;

	      	count -= write_sz;

		DPRINTK("starting, size = %d, ppos = 0x%x, buf = 0x%x, mode = 0x%x\n",
			write_sz, (u32) *ppos + rdev->dio.base_offset, p, rdev->dio.write_mode);

		/* Start the DIO transfer */
		res = rdev->net->hport->ops->transfer(rdev->net->hport,
						      rdev->net->hport->id,
						      rdev->destid,
						      virt_to_phys(p),
						      (u32) (*ppos + rdev->dio.base_offset),
						      write_sz,
						      rdev->dio.write_mode);
		
		if (res)
			goto out;

		if (!copy)
			p += write_sz;

		*ppos     += (u64) write_sz;
		write_pos += write_sz;
	}

	DPRINTK("finished, size = %d, ppos = 0x%llx\n", 
		size, 
		*ppos + rdev->dio.base_offset);
	
	res = size;
out:
	if (copy)
		kfree(src_buf);

	return res;
}

static ssize_t rio_dev_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct rio_dev	        *rdev = filp->private_data;
	int                      res;
	size_t                   count = size;
	size_t                   read_sz;
	char                    *dest_buf;
	char                    *p;
	int                      copy;
	int                      read_pos = 0;

	if (!rdev->net->hport->ops->transfer)
		return -EPROTONOSUPPORT;

	DPRINTK("read buffer, buf = 0x%x, size = %d\n", buf, size);

	/* try to do zero-copy is buffer has the right property */
	if ((virt_to_phys(buf) != L1_CACHE_ALIGN(virt_to_phys(buf))) ||
	    (size < L1_CACHE_ALIGN(size)) || (USE_COPY)) {
		int alloc_size = (size > RIO_MAX_DIO_CHUNK_SIZE) ?
			RIO_MAX_DIO_CHUNK_SIZE : size;

		p = dest_buf = kmalloc(L1_CACHE_ALIGN(alloc_size), GFP_KERNEL);
		if (!p) {
			res = -ENOMEM;
			goto out;
		}

		copy = 1;

		/* if allocated buffer is still non-aligned on cache */
		if ((unsigned int) p != L1_CACHE_ALIGN((unsigned int) p)) {
			res = -ENOMEM;
			goto out;
		}

		DPRINTK("Allocating read buffer, p = 0x%x, size = %d\n", 
			p, L1_CACHE_ALIGN(size));

	} else {
		p    = dest_buf = buf; /* zero-copy case */
		copy = 0;
	}

	while(count) {
		if (copy)
			read_sz = (count <= RIO_MAX_DIO_CHUNK_SIZE) ? count : RIO_MAX_DIO_CHUNK_SIZE;
		else
			read_sz = count;

		count  -= read_sz;

		DPRINTK("starting, size = %d, ppos = 0x%x\n", read_sz, 
			(u32) *ppos + rdev->dio.base_offset);

		/* Start the DIO transfer */
		res = rdev->net->hport->ops->transfer(rdev->net->hport,
						      rdev->net->hport->id,
						      rdev->destid,
						      virt_to_phys(p),
						      (u32) (*ppos + rdev->dio.base_offset),
						      read_sz,
						      RIO_DIO_MODE_READ);
		if (res)
			goto out;

		DPRINTK("Incoming data, size = %d, user buf = 0x%x, buf = 0x%x\n", 
			size, buf, dest_buf);


		if (copy)
			copy_to_user(buf + read_pos, p, read_sz);
		else
			p += read_sz;

		*ppos    += (u64) read_sz;
		read_pos += read_sz;

	}

	DPRINTK("finished, size = %d, ppos = 0x%llx\n", 
		size, 
		*ppos + rdev->dio.base_offset);
	
	res = size;
 out:
	if (copy)
		kfree(dest_buf);

	return res;	
}

static int rio_dev_ioctl(struct inode *inode, struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	struct rio_dev	        *rdev   = filp->private_data;
	int                      status = 0;

	switch (cmd) {
	case RIO_DIO_BASE_SET:
		u32 base;
		if (get_user(base, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		rdev->dio.base_offset = base;
		break;


	case RIO_DIO_BASE_GET:
		u32 base = rdev->dio.base_offset;
		if (put_user(base, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		break;

	case RIO_DIO_MODE_SET:
		int mode;
		if (get_user(mode, (int *) arg)) {
                        status = -EFAULT;
			break;
		}
		switch(mode & 0xf) {
		case RIO_DIO_MODE_WRITER:
			rdev->dio.write_mode = RIO_DIO_MODE_WRITER;
			break;
		case RIO_DIO_MODE_WRITE:
			rdev->dio.write_mode = RIO_DIO_MODE_WRITE;
			break;
		case RIO_DIO_MODE_SWRITE:
			rdev->dio.write_mode = RIO_DIO_MODE_SWRITE;
			break;
		default:
			status = -EINVAL;
			break;
		}
		break;

	case RIO_DIO_MODE_GET:
		int mode = rdev->dio.write_mode;
		if (put_user(mode, (u32 *) arg)) {
                        status = -EFAULT;
			break;
		}
		break;

	case RIO_DBELL_TX:
		int dbnum;
		if (get_user(dbnum, (int *) arg)) {
                        status = -EFAULT;
			break;
		}
		/* Send a doorbell */
		if (rdev->net->hport->ops->dsend)
			status = rdev->net->hport->ops->dsend(rdev->net->hport,
							      rdev->net->hport->id,
							      rdev->destid,  
							      (u16) dbnum);
		else
			status = -EPROTONOSUPPORT;
		break;

	case RIO_DBELL_RX:
		int doorbell_info;
		if (get_user(doorbell_info, (int *) arg)) {
                        status = -EFAULT;
			break;
		}
		/* Wait a doorbell */
		if (rdev->net->hport->ops->dsend)
			status = rdev->net->hport->ops->dwait(rdev->net->hport,
							      (u16) doorbell_info);
		else
			status = -EPROTONOSUPPORT;
		break;

	default:
		status = -EINVAL;
        }
	return status;
}

static int rio_dev_open(struct inode *inode, struct file *filp)
{
	struct list_head *n;
	struct rio_dev   *rdev;
	int	          status = -ENXIO;

	WARN_ON(in_interrupt());
	spin_lock(&rio_global_list_lock);
	n = rio_devices.next;

	while (n && (n != &rio_devices)) {
		rdev = rio_dev_g(n);
		if (rdev->dev.devt == inode->i_rdev)
			goto exit;
		n = n->next;
	}
	rdev = NULL;
      exit:
	rdev = rio_dev_get(rdev);
	spin_unlock(&rio_global_list_lock);

	if (rdev != NULL) {
		filp->private_data = rdev;
		status = 0;
	}
	return status;
}

static int rio_dev_release(struct inode *inode, struct file *filp)
{
	struct rio_dev *rdev;
	int	        status = 0;

	mutex_lock(&device_list_lock);

	rdev = filp->private_data;
	filp->private_data = NULL;

	mutex_unlock(&device_list_lock);

	return status;
}

static struct file_operations rio_dev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.llseek =       rio_dev_llseek,
	.write =	rio_dev_write,
	.read =		rio_dev_read,
	.ioctl =	rio_dev_ioctl,
	.open =		rio_dev_open,
	.release =	rio_dev_release,

};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/rapidio/0/1 character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static void rio_dev_classdev_release(struct device *dev)
{
}

static struct class rio_dev_class = {
	.name		= "rio_dev",
	.owner		= THIS_MODULE,
	.dev_release	= rio_dev_classdev_release,
};

/*
 * Called when adding a site, this will create the corresponding char device
 * with udev/mdev
 */
int rio_dev_add(struct rio_dev *rdev)
{
	int	          status;
	unsigned long     minor;
	struct rio_mport *port = rdev->net->hport;

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_RIO_MINORS);

	if (minor < N_RIO_MINORS) {
		struct device *dev;

		rdev->dio.base_offset = 0;
		rdev->dev.devt        = MKDEV(RIO_DEV_MAJOR, minor);

		dev = device_create(&rio_dev_class,
				    &rdev->dev, 
				    rdev->dev.devt,
				    rdev,
				    "%s%d.%d",
				    RIO_DEV_NAME,
				    port->id,
				    rdev->destid);

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&rdev->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0)
		set_bit(minor, minors);

	mutex_unlock(&device_list_lock);

	return status;
}

int rio_dev_remove(struct rio_dev *rdev, struct rio_mport *port)
{
	mutex_lock(&device_list_lock);

	device_destroy(&rio_dev_class, rdev->dev.devt);
	clear_bit(MINOR(rdev->dev.devt), minors);
	mutex_unlock(&device_list_lock);

	return 0;
}

int rio_dev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes. Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_RIO_MINORS > 256);
	status = register_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME, &rio_dev_fops);
	if (status < 0)
		return status;

	status = class_register(&rio_dev_class);
	if (status < 0) {
		unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
		return status;
	}
	return 0;
}

void rio_dev_exit(void)
{
	class_unregister(&rio_dev_class);
	unregister_chrdev(RIO_DEV_MAJOR, RIO_DEV_NAME);
}

EXPORT_SYMBOL_GPL(rio_dev_init);
EXPORT_SYMBOL_GPL(rio_dev_exit);
EXPORT_SYMBOL_GPL(rio_dev_add);
EXPORT_SYMBOL_GPL(rio_dev_remove);
