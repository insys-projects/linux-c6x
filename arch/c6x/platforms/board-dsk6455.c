/*
 *  linux/arch/c6x/platforms/board-dsk6455.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2008, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/sc16is7xx.h>
#include <linux/kernel_stat.h>
#include <linux/platform_device.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/timer.h>
#include <asm/percpu.h>
#include <asm/clock.h>

#include <mach/gemac.h>
#include <mach/gmdio.h>
#include <mach/board.h>

/*
 * Resources present on the DSK6455 board
 */
static struct resource _flash_res = {
	.name  = "Flash",
	.start = 0xb0000000,
	.end   = 0xbfffffff,
	.flags = IORESOURCE_MEM,
};
static struct resource _cpld_async_res = {
	.name  = "CPLD async",
	.start = 0xa0000000,
	.end   = 0xa0000008,
	.flags = IORESOURCE_MEM,
}; 

#define NR_RESOURCES 2
static struct resource *dsk_resources[NR_RESOURCES] = 
	{ &_flash_res, &_cpld_async_res };


static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	int i, ret;

	printk("Designed for the DSK6455 board, Spectrum Digital Inc.\n");

	/* Initialize DSK6455 resources */
	iomem_resource.name = "Memory";
	for (i = 0; i < NR_RESOURCES; i++)
		request_resource(&iomem_resource, dsk_resources[i]);

	/* Initialize led register */
	cpld_set_reg(DSK6455_CPLD_USER, 0x0);

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	mach_progress(1, "End of EVM6486 specific initialization");
}

__init void evm_init(void)
{
}

arch_initcall(evm_init);
