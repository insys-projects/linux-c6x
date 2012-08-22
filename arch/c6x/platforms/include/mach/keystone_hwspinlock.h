/*
 * Keystone hardware spinlock driver
 * 
 * Copyright (C) 2011, 2012 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Cyril Chemparathy <cyril@ti.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * Cloned from original OMAP driver, original information follows:
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Simon Que <sque@ti.com>
 *          Hari Kanigeri <h-kanigeri2@ti.com>
 *          Ohad Ben-Cohen <ohad@wizery.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#ifndef __MACH_C6X_KEYSTONE_HWSPINLOCK_H
#define __MACH_C6X_KEYSTONE_HWSPINLOCK_H

#include <linux/delay.h>

#define DEVICE_HWSPINLOCK_BASE  0x02640000
#define DEVICE_HWSPINLOCK_SIZE  0x800

#define LOCK_RESET_OFFSET	0x8
#define LOCK_BASE_OFFSET	0x0100
#define SPINLOCK_NOTTAKEN	1
#define NUM_SPINLOCKS		32

/**
 * struct hwspinlock - this struct represents a single hwspinlock instance
 * @bank: the hwspinlock_device structure which owns this lock
 * @lock: initialized and used by hwspinlock core
 * @priv: private data, owned by the underlying platform-specific hwspinlock drv
 */
struct hwspinlock {
	spinlock_t    lock;
	void __iomem *io_base;
	void         *priv;
};

static inline void keystone_hwspinlock_reset(struct hwspinlock *lock)
{
	__raw_writel(1, lock->io_base + LOCK_RESET_OFFSET);

	while (!(__raw_readl(lock->io_base + LOCK_RESET_OFFSET) & 1)); /* spin */
}

static inline int keystone_hwspinlock_trylock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;

	/* attempt to acquire the lock by reading its value */
	return (SPINLOCK_NOTTAKEN == __raw_readl(lock_addr));
}

static inline void keystone_hwspinlock_unlock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;

	/* release the lock by writing 1 to it */
	__raw_writel(SPINLOCK_NOTTAKEN, lock_addr);
}

static inline void keystone_hwspinlock_lock(struct hwspinlock *lock)
{
	while (!keystone_hwspinlock_trylock(lock)); /* spin */
}

static inline void keystone_hwspinlock_relax(struct hwspinlock *lock)
{
	ndelay(50);
}

static inline struct hwspinlock* keystone_hwspinlock_init(int id)
{
	struct hwspinlock *hwlock;
	void __iomem      *io_base;

	io_base = ioremap(DEVICE_HWSPINLOCK_BASE, DEVICE_HWSPINLOCK_SIZE);
	if (!io_base)
		return NULL;

	hwlock = kzalloc(sizeof(*hwlock), GFP_KERNEL);
	if (!hwlock)
		goto iounmap_base;

	hwlock->priv    = io_base + LOCK_BASE_OFFSET + sizeof(u32) * id;
	hwlock->io_base = io_base;

	return hwlock;

iounmap_base:
	iounmap(io_base);
	return NULL;
}
#endif /* __MACH_C6X_KEYSTONE_HWSPINLOCK_H */

