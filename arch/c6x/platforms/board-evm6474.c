/*
 * Copyright (C) 2010 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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

#include <mach/board.h>

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

#ifdef CONFIG_RAPIDIO_TCI648X
#include <linux/rio.h>
#include <asm/rio.h>

struct tci648x_rio_board_controller_info evm6474_rio_controller = {
	0x3,                         /* bitfield of port(s) to probe on this controller */
	TCI648X_RIO_MODE_0,          /* SERDES configuration (BOOTMODE8) */
	0,                           /* host id */
	RIO_DO_ENUMERATION,          /* initialisation method */
	1                            /* large size (16bit)*/
};

static struct platform_device evm6474_rio_device = {
	.name           = "tci648x-rapidio",
	.id             = 1,
	.dev		= { .platform_data = &evm6474_rio_controller },
};

static void __init evm_init_rio(void)
{
	platform_device_register(&evm6474_rio_device);
}

core_initcall(evm_init_rio);
#endif

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	int i, ret;

	printk("Designed for the EVM6474 board, Texas Instruments.\n");

	gpio_direction(0xFFFF);  /* all input */

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

//	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6474 specific initialization");
}
