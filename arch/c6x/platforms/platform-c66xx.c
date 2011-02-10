/*
 *  linux/arch/c6x/platforms/platform-c66xx.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/genhd.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mtd/physmap.h>

#include <asm/pll.h>
#include <asm/setup.h>
#include <asm/irq.h>

#include <mach/board.h>

/*
 * Resources present on the SoC
 */
static struct resource c6x_soc_res = {
	.name = "C66X SOC peripherals",
	.start = 0x01000000, .end = 0x02ffffff
};

#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
extern unsigned int c6x_platram_start;
extern unsigned int c6x_platram_size;

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probe_types[] = { "cmdlinepart", NULL };
#endif

static struct platdata_mtd_ram c6x_plat_data = {
	.mapname   = "DRAM",
#ifdef CONFIG_MTD_PARTITIONS
	.probes    = (const char **) part_probe_types,
#endif
	.bankwidth = 4,
};

static struct resource c6x_platram_resource = {
	.start = 0,
	.end   = 0,
	.flags = IORESOURCE_MEM,
};

static struct platform_device c6x_platram_device = {
	.name          = "mtd-ram",
	.id            = 8,
	.dev = {
		.platform_data = &c6x_plat_data,
	},
	.num_resources = 1,
	.resource      = &c6x_platram_resource,
};
#endif

static void init_pll(void)
{
}

static void init_power(void)
{
}

void c6x_soc_setup_arch(void)
{
 	/* Initialize C66x IRQs */          	
	clear_all_irq(); /* acknowledge all pending irqs */

	init_pll();

	init_power();

	/* Initialize SoC resources */
	iomem_resource.name = "Memory";
	request_resource(&iomem_resource, &c6x_soc_res);
}


static int __init platform_arch_init(void)
{
	int status = 0;

#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
	if (c6x_platram_size) {

		c6x_platram_resource.start = c6x_platram_start;
		c6x_platram_resource.end   = c6x_platram_start + c6x_platram_size - 1;

		status  = platform_device_register(&c6x_platram_device);
			printk(KERN_ERR "Could not register platram device: %d\n", status);
	}
#endif
	return status;
}

arch_initcall(platform_arch_init);
