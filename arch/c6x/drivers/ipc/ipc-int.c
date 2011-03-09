/*
 *  linux/arch/c6x/drivers/ipc/ipc-int.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/ipc-core.h>

#include <mach/board.h>

#define IPCG           1
#define IPC_INT_MAX    27
#define IPC_INT_SHIFT  4

struct ipc_info {
	void               (*handler)(u32, void*);
	wait_queue_head_t  wq;
	u32                flags;
	void              *data;
} ipc_info;

static struct ipc_info ipc_list[IPC_INT_MAX + 1];
static spinlock_t      ipc_lock;

/*
 * Request an inter-core interrupt rx handler
 */
static int ipc_int_request(void (*handler)(u32, void*),
			       u32 iflags,
			       u32 ipc_num,
			       void *data)
{
	unsigned long flags;

	if (ipc_num > IPC_INT_MAX)
		return -EINVAL;

	if (ipc_list[ipc_num].handler != NULL)
		return -EBUSY;

	spin_lock_irqsave(&ipc_lock, flags);

	ipc_list[ipc_num].handler = handler;
	ipc_list[ipc_num].flags   = iflags;
	ipc_list[ipc_num].data    = data;
	
	spin_unlock_irqrestore(&ipc_lock, flags);

	return 0;
}

/*
 * Free an inter-core interrupt rx handler
 */
static int ipc_int_free(u32 ipc_num)
{
	unsigned long flags;

	if (ipc_num > IPC_INT_MAX)
		return -EINVAL;
	
	spin_lock_irqsave(&ipc_lock, flags);

	ipc_list[ipc_num].handler = NULL;
	ipc_list[ipc_num].data    = NULL;
	ipc_list[ipc_num].flags   = 0;
	
	spin_unlock_irqrestore(&ipc_lock, flags);

	return 0;
}

/*
 * Trigger an inter-core interrupt to a given core
 */
static int ipc_int_send(int core_id, int ipc_num)
{
	volatile u32 *reg_ipcgr;

	if ((ipc_num < 0) || (ipc_num > IPC_INT_MAX))
		return -EINVAL;

	if ((core_id < 0) || (core_id > CORE_NUM))
		return -EINVAL;

	reg_ipcgr = ((u32 *) IPCGR_BASE) + core_id;
	
	*reg_ipcgr = (1 << (ipc_num + IPC_INT_SHIFT)) | IPCG;

	return 0;
} 

/*
 * Wait a given inter-core interrupt
 */
static int ipc_int_wait(int ipc_num)
{
	unsigned long flags;

	DECLARE_WAITQUEUE(wait, current);

	if ((ipc_num < 0) || (ipc_num > IPC_INT_MAX))
		return -EINVAL;

	spin_lock_irqsave(&ipc_lock, flags);
	add_wait_queue(&ipc_list[ipc_num].wq, &wait);
	current->state = TASK_INTERRUPTIBLE;
	spin_unlock_irqrestore(&ipc_lock, flags);

	schedule();

	spin_lock_irqsave(&ipc_lock, flags);
	remove_wait_queue(&ipc_list[ipc_num].wq, &wait);
	current->state = TASK_RUNNING;
	spin_unlock_irqrestore(&ipc_lock, flags);

	if (signal_pending(current))
		return -ERESTARTSYS;
	else 
		return ipc_num;
} 

/*
 * Handle the receive of an inter-core interrupt
 */
static irqreturn_t ipc_int_rx_interrupt(int irq, void * dummy)
{
	volatile u32 *reg_ipcgr = ((u32 *) IPCGR_BASE) + get_coreid();
	volatile u32 *reg_ipcar = ((u32 *) IPCAR_BASE) + get_coreid();
	u32 mask;
	u32 ipc_src;
	u32 ipc_num;
	
	while((mask = (*reg_ipcgr) & ~0xf) != 0) {
		ipc_src    = __ffs(mask) ;
		*reg_ipcar = (1 << ipc_src);      /* un-assert handled IPC source */

		ipc_num = ipc_src - IPC_INT_SHIFT;
	
		/* Call handler if any */
		if (ipc_list[ipc_num].handler != NULL) {
			
			if (!(ipc_list[ipc_num].flags & IRQF_DISABLED))
				local_irq_enable();
			
			ipc_list[ipc_num].handler(ipc_num, ipc_list[ipc_num].data);
			
			local_irq_disable();
		}
		
		/* Wake up waiting processes */
		wake_up_all(&ipc_list[ipc_num].wq);
	}
	
	return IRQ_HANDLED;
}

static int __init ipc_int_init(void)
{
	int res;
	int i;

	spin_lock_init(&ipc_lock);

	for (i = 0 ; i < IPC_INT_MAX; i++) {
		init_waitqueue_head(&ipc_list[i].wq);
		ipc_list[i].flags   = 0;
		ipc_list[i].handler = NULL;
	}

	res = request_irq(IRQ_IPCLOCAL,
			  ipc_int_rx_interrupt,
			  IRQF_DISABLED,
			  "IPC",
			  NULL);
	if (res != 0)
		return res;

	/* Set our IPC interrupt methods */
	ipc_core->ipc_request = ipc_int_request;
	ipc_core->ipc_free    = ipc_int_free;
	ipc_core->ipc_send    = ipc_int_send;
	ipc_core->ipc_wait    = ipc_int_wait;

	return 0;
}

module_init(ipc_int_init);
