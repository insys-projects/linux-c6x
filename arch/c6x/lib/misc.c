/*
 *  linux/arch/c6x/lib/misc.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com) and others
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <asm/system.h>

void abort(void)
{
	while (1) {
		BUG();
		asm(" IDLE");
	}
}

void exit(int status)
{
	abort();
}

static u8 *__alloca_buffer = NULL;

void *alloca (size_t size)
{
	unsigned long flags;
        void *free_buf = NULL;

#ifdef ALLOCA_DEBUG
	printk(KERN_INFO "alloca() called (size = %d)\n", size);
#endif

	local_irq_save(flags);

	/* Allocate buffer at first alloca() call */
	if (__alloca_buffer == NULL) {
		__alloca_buffer = (u8 *) __get_free_page(GFP_KERNEL);
		if (__alloca_buffer == NULL) {
			local_irq_restore(flags);
			printk("alloca() failed, unable to get free page\n");
			return free_buf;
		}
	}
	
	/*
	 * Check for buffer wrap-around, if it is the case,
	 * restart at the begining of the buffer.
	 */
	if ((((unsigned) __alloca_buffer + size + 7) & PAGE_MASK) !=
	    (((unsigned) __alloca_buffer) & PAGE_MASK))
	    __alloca_buffer = (u8*)((unsigned) __alloca_buffer & PAGE_MASK);
	
	free_buf = (void *) __alloca_buffer;
	__alloca_buffer =
		(u8 *) (((unsigned) __alloca_buffer + size + 7) & ~7);

	local_irq_restore(flags);

	return free_buf;
}
