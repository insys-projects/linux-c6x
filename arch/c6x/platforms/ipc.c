/*
 *  linux/arch/c6x/kernel/ipc.c
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

#include <mach/board.h>

#define IPCG       1
#define IPC_MAX    27
#define IPC_SHIFT  4

struct ipc_info {
	void               (*handler)(u32);
	wait_queue_head_t  wq;
	u32                flags;
} ipc_info;

/*static*/ struct ipc_info ipc_list[IPC_MAX + 1];
static spinlock_t      ipc_lock;

/*
 * Request an Inter-DSP Interrupt Rx handler
 */
int tci648x_ipc_request(void (*handler)(u32), u32 flags, u32 ipc_num)
{
	u32 flags;

	if (ipc_num > IPC_MAX)
		return -EINVAL;

	if (ipc_list[ipc_num].handler != NULL)
		return -EBUSY;

	spin_lock_irqsave(&ipc_lock, flags);

	ipc_list[ipc_num].handler = handler;
	ipc_list[ipc_num].flags   = flags;
	
	spin_unlock_irqrestore(&ipc_lock, flags);

	return 0;
}

/*
 * Free an Inter-DSP Interrupt Rx handler
 */
int tci648x_ipc_free(u32 ipc_num)
{
	u32 flags;

	if (ipc_num > IPC_MAX)
		return -EINVAL;
	
	spin_lock_irqsave(&ipc_lock, flags);

	ipc_list[ipc_num].handler = NULL;
	ipc_list[ipc_num].flags   = 0;
	
	spin_unlock_irqrestore(&ipc_lock, flags);

	return 0;
}

/*
 * Trigger an Inter-DSP Interrupt to a given core
 */
int tci648x_ipc_send(int core_id, int ipc_num)
{
	volatile u32 *reg_ipcgr;

	if ((ipc_num < 0) || (ipc_num > IPC_MAX))
		return -EINVAL;

	if ((core_id < 0) || (core_id > CORE_NUM))
		return -EINVAL;

	reg_ipcgr = ((u32 *) IPCGR_BASE) + core_id;
	
	*reg_ipcgr = (1 << (ipc_num + IPC_SHIFT)) | IPCG;

	return 0;
} 

/*
 * Wait a given Inter-DSP Interrupt
 */
int tci648x_ipc_wait(int ipc_num)
{
	u32 flags;

	DECLARE_WAITQUEUE(wait, current);

	if ((ipc_num < 0) || (ipc_num > IPC_MAX))
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
 * Handle the receive of an Inter-DSP Interrupt
 */
static irqreturn_t tci648x_ipc_rx_interrupt(int irq, void * dummy)
{
	volatile u32 *reg_ipcgr = ((u32 *) IPCGR_BASE) + get_coreid();
	volatile u32 *reg_ipcar = ((u32 *) IPCAR_BASE) + get_coreid();
	u32 mask;
	u32 ipc_src;
	u32 ipc_num;
	
	mask = (*reg_ipcgr) & ~0xf;
	if (mask == 0)
		return IRQ_HANDLED; /* Spurious interrupt case */
	
	ipc_src = __ffs(mask);
	*reg_ipcar = (1 << ipc_src); /* Un-assert handled IPC source */
	
	ipc_num = ipc_src - IPC_SHIFT;
	
	/* Call handler if any */
	if (ipc_list[ipc_num].handler != NULL) {
		
		if (!(ipc_list[ipc_num].flags & SA_INTERRUPT))
			local_irq_enable();
		
		ipc_list[ipc_num].handler(ipc_num);

		local_irq_disable();
	}

	/* Wake up waiting processes */
	wake_up_all(&ipc_list[ipc_num].wq);

	return IRQ_HANDLED;
}

static int __init tci648x_ipc_init(void)
{
	int res;
	int i;

	spin_lock_init(&ipc_lock);

	for (i = 0 ; i < IPC_MAX; i++) {
		init_waitqueue_head(&ipc_list[i].wq);
		ipc_list[i].flags   = 0;
		ipc_list[i].handler = NULL;
	}
		
	irq_map(IRQ_IPCLOCAL, INT5);
	res = request_irq(IRQ_IPC,
			  tci648x_ipc_rx_interrupt,
			  IRQF_DISABLED,
			  "IPC",
			  NULL);
	if (res != 0)
		return res;

	printk(KERN_INFO "IPC: Inter-DSP Interrupt service\n");

	return 0;
}

module_init(tci648x_ipc_init);
