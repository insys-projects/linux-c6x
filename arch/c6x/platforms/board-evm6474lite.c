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

#include <mach/gemac.h>
#include <mach/board.h>

#ifdef CONFIG_TMS320C64X_GEMAC
static struct resource emac_resources0 [] = {
	{
		.name           = "EMAC_REG_BASE",
		.start          =  EMAC_REG_BASE,
		.end            =  EMAC_REG_BASE + 0xFFF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "ECTL_REG_BASE",
		.start          =  ECTL_REG_BASE,
		.end            =  ECTL_REG_BASE + 0x7FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "EMAC_DSC_BASE",
		.start          =  EMAC_DSC_BASE,
		.end            =  EMAC_DSC_BASE + 0x17FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "IRQ_SRC",
		.start          =  IRQ_EMAC_RX_0,
		.flags          =  IORESOURCE_IRQ,
	},
};

static struct platform_device emac_dev0 = {
        .name           = "EMAC",
        .id             = 0,
	.resource       = emac_resources0,
        .num_resources  = ARRAY_SIZE(emac_resources0),
};

static void setup_emac(void)
{
        int status;

        status  = platform_device_register(&emac_dev0);
        if (status != 0)
                pr_debug("setup_emac0 --> %d\n", status);
        //else
        //  /* Power domain need to be activated here */
}
#else
static void setup_emac(void) {}
#endif

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	int i, ret;

	printk("Designed for the EVM6474 Lite EVM\n");

	/* Configure the interupt selector MUX registers */
	irq_map(IRQ_TINT1, IRQ_CLOCKEVENTS);

	gpio_direction(0xFFFF);  /* all input */

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

//	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6474 Lite specific initialization");
}

__init void evm_init(void)
{
	setup_emac();
}

arch_initcall(evm_init);
