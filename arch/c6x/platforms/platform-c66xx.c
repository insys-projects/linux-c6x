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
#include <asm/dscr.h>

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
	unsigned int val;

	/* Unlock DSCR boot config */
	dscr_set_reg(DSCR_KICK0, DSCR_KICK0_KEY);
	dscr_set_reg(DSCR_KICK1, DSCR_KICK1_KEY);

	/* Set ENSAT */
	val = dscr_get_reg(DSCR_MAINPLLCTL1) | 0x40;
	dscr_set_reg(DSCR_MAINPLLCTL1, val);

	/* Set OUTPUT_DIVIDE and Main PLL Bypass enabled */
	pll1_set_reg(SECCTL, 0x810000 | ((PLL_OUTDIV - 1) << 19));

	udelay(10);
	
	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLENSRC);
	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLEN);

	udelay(10);

	/* Reset PLL */
	pll1_setbit_reg(PLLCTL, PLLCTL_PLLRST);
	if (pll1_get_reg(PLLCTL & 0x2) != 0) {
		val = pll1_get_reg(PLLCTL) & 0xfffffffd;
		pll1_setbit_reg(PLLCTL, val);

		/* Wait PLL stabilization time */
		udelay(150);
	}
	
	/* Set PLL multiplier * 2 in PLLM */
	pll1_set_reg(PLLM, PLLM_VAL(PLL_MUL*2));

	/* Program main PLL BWADJ field */
	val = dscr_get_reg(DSCR_MAINPLLCTL0);
	val = (((PLL_MUL + 1)/2  - 1) << 24) & 0xff000000;
	dscr_set_reg(DSCR_MAINPLLCTL0, val);

	/*
	 * This can't be right. There is no predivider here...
	 * pll1_set_reg(PLLPREDIV, PLLPREDIV_VAL(10) | PLLPREDIV_EN);
	 */
	pll1_wait_gostat();

	/*  Set divider */
	pll1_set_reg(PLLDIV2, PLLDIV_RATIO(PLL_DIV2) | PLLDIV_EN);
	pll1_set_reg(PLLDIV5, PLLDIV_RATIO(PLL_DIV5) | PLLDIV_EN);
	pll1_set_reg(PLLDIV8, PLLDIV_RATIO(PLL_DIV8) | PLLDIV_EN);
	
	/* Adjust modified related sysclk align */
	pll1_set_reg(PLLALNCTL, pll1_get_reg(PLLDCHANGE));

	pll1_setbit_reg(PLLCMD, PLLCMD_GOSTAT);

	pll1_wait_gostat();

	/* Wait for PLL to lock */
	udelay(5);

	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLRST);

	udelay(40);

	/* Main PLL Bypass disabled, set OUTPUT_DIVIDE */
	pll1_set_reg(SECCTL, 0x10000 | ((PLL_OUTDIV - 1) << 19)); 
	pll1_setbit_reg(PLLCTL, PLLCTL_PLLEN);

	/* Lock DSCR */
	dscr_set_reg(DSCR_KICK0, 0);
	dscr_set_reg(DSCR_KICK1, 0);
}

static void init_power(void)
{
}

#ifdef CONFIG_TI_KEYSTONE_PKTDMA
#include <mach/netcp.h>
#include <mach/keystone_qmss.h>

struct keystone_platform_data c6x_pktdma_data = {
	.irq            = IRQ_QMH + DEVICE_QM_ETH_ACC_CHANNEL,

};

static struct platform_device pktdma_dev0 = {
        .name           = "keystone_pktdma",
        .id             = 0,
	.dev = {
		.platform_data = &c6x_pktdma_data,
	},
};

static void setup_pa(void)
{
        platform_device_register(&pktdma_dev0);
}
#else
static void setup_pa(void) { }
#endif

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

	setup_pa();

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
