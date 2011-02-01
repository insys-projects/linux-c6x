/*
 *  linux/arch/c6x/platforms/board-evm6457.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2008, 2009, 2010, 2011 Texas Instruments Incorporated
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
#include <linux/i2c/sc16is7xx.h>
#include <linux/kernel_stat.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-gpio-c6x.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/timer.h>
#include <asm/percpu.h>
#include <asm/clock.h>
#include <asm/edma.h>

#include <mach/i2c.h>
#include <mach/board.h>

#ifdef CONFIG_I2C
static struct at24_platform_data at24_eeprom_data = {
	.byte_len	= 0x100000 / 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};

#ifdef CONFIG_SERIAL_SC16IS7XX
static struct sc16is7xx_platform_data uart_data = {
	.baud_base = 921600,
};
#endif

static struct i2c_board_info evm_i2c_info[] = {
#ifdef CONFIG_SERIAL_SC16IS7XX
	/* this must come first */
	{ I2C_BOARD_INFO("sc16is750", 0x4d),
	  .platform_data = &uart_data,
	},
#endif
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c1024", 0x50),
	  .platform_data = &at24_eeprom_data,
	},
#endif
};

#define I2C_UART_GPIO 15

static void __init evm_setup_i2c(void)
{
#if defined(CONFIG_SERIAL_SC16IS7XX)
	int status, irq;
	/* setup gpio for interrupt from i2c UART */
	status = gpio_request(I2C_UART_GPIO, "I2C-UART IRQ");
	if (status < 0)
		printk(KERN_ERR "Cannot get GPIO for I2C UART: %d\n", status);
	else {
		gpio_direction_input(I2C_UART_GPIO);
		irq = gpio_to_irq(I2C_UART_GPIO);
		if (irq < 0)
			printk(KERN_ERR "GPIO for I2C UART has no IRQ: %d\n", irq);
		else
			set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
		evm_i2c_info[0].irq = irq;
	}
#endif
	i2c_register_board_info(1, evm_i2c_info,
				ARRAY_SIZE(evm_i2c_info));
}
#else
#define evm_setup_i2c()
#endif /* CONFIG_I2C */

#if defined(CONFIG_MTD_NAND_GPIO_C6X) || defined(CONFIG_MTD_NAND_GPIO_C6X_MODULE)
static struct mtd_partition evm_nand_parts[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 0x00200000,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x01000000,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct gpio_nand_platdata evm_nand_platdata = {
	.parts = evm_nand_parts,
	.num_parts = ARRAY_SIZE(evm_nand_parts),
	.chip_delay = 25,
};

static struct platform_device evm_nand = {
	.name		= "gpio-nand-c6x",
	.id		= -1,
	.dev		= {
		.platform_data = &evm_nand_platdata,
	}
};

static void __init evm_setup_nand(void)
{
	platform_device_register(&evm_nand);
}
#else
static inline void evm_setup_nand(void) {}
#endif

#ifdef CONFIG_EDMA3

/* Four Transfer Controllers on TMS320C6455 */
static const s8
queue_tc_mapping[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{-1, -1},
};

static const s8
queue_priority_mapping[][2] = {
	/* {event queue no, Priority} */
	{0, 4},	/* FIXME: what should these priorities be? */
	{1, 0},
	{2, 5},
	{3, 1},
	{-1, -1},
};


static struct edma_soc_info edma_cc0_info = {
	.n_channel		= EDMA_NUM_DMACH,
	.n_region		= EDMA_NUM_REGIONS,
	.n_slot			= EDMA_NUM_PARAMENTRY,
	.n_tc			= EDMA_NUM_EVQUE,
	.n_cc			= 1,
	.queue_tc_mapping	= queue_tc_mapping,
	.queue_priority_mapping	= queue_priority_mapping,
};

static struct edma_soc_info *edma_info[] = {
	&edma_cc0_info,
};

static struct resource edma_resources[] = {
	{
		.name	= "edma_cc0",
		.start	= EDMA_REGISTER_BASE,
		.end	= EDMA_REGISTER_BASE + 0xFFFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc0",
		.start	= EDMA_TC0_BASE,
		.end	= EDMA_TC0_BASE + 0x03FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc1",
		.start	= EDMA_TC1_BASE,
		.end	= EDMA_TC1_BASE + 0x03FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc2",
		.start	= EDMA_TC2_BASE,
		.end	= EDMA_TC2_BASE + 0x03FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc3",
		.start	= EDMA_TC3_BASE,
		.end	= EDMA_TC3_BASE + 0x03FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma0",
		.start	= EDMA_IRQ_CCINT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma0_err",
		.start	= EDMA_IRQ_CCERRINT,
		.flags	= IORESOURCE_IRQ,
	},
	/* not using TC*_ERR */
};

static struct platform_device edma_device = {
	.name			= "edma",
	.id			= 0,
	.dev.platform_data	= edma_info,
	.num_resources		= ARRAY_SIZE(edma_resources),
	.resource		= edma_resources,
};


static void __init evm_setup_edma(void)
{
	int status;

	status = platform_device_register(&edma_device);
	if (status != 0)
		pr_debug("setup_edma --> %d\n", status);
}
#else
#define evm_setup_edma()
#endif /* CONFIG_EDMA3 */

static struct pll_data pll1_data = {
	.num       = 1,
	.phys_base = ARCH_PLL1_BASE,
};

static struct clk clkin1 = {
	.name = "clkin1",
	.rate = 60000000,
	.node = LIST_HEAD_INIT(clkin1.node),
	.children = LIST_HEAD_INIT(clkin1.children),
	.childnode = LIST_HEAD_INIT(clkin1.childnode),
};

static struct clk pll1_clk = {
	.name = "pll1",
	.parent = &clkin1,
	.pll_data = &pll1_data,
	.flags = CLK_PLL,
};

static struct clk pll1_sysclk1 = {
	.name = "pll1_sysclk1",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 1,
};

static struct clk pll1_sysclk2 = {
	.name = "pll1_sysclk2",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 3,
};

static struct clk pll1_sysclk3 = {
	.name = "pll1_sysclk3",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 6,
};

static struct clk pll1_sysclk4 = {
	.name = "pll1_sysclk4",
	.parent = &pll1_clk,
	.flags = CLK_PLL,
	.div = PLLDIV4,
};

static struct clk pll1_sysclk5 = {
	.name = "pll1_sysclk5",
	.parent = &pll1_clk,
	.flags = CLK_PLL,
	.div = PLLDIV5,
};

static struct clk i2c_clk = {
	.name = "i2c",
	.parent = &pll1_sysclk3,
};

static struct clk core_clk = {
	.name = "core",
	.parent = &pll1_sysclk1,
};

static struct clk watchdog_clk = {
	.name = "watchdog",
	.parent = &pll1_sysclk5,
};

static struct clk_lookup evm_clks[] = {
	CLK(NULL, "pll1", &pll1_clk),
	CLK(NULL, "pll1_sysclk1", &pll1_sysclk1),
	CLK(NULL, "pll1_sysclk2", &pll1_sysclk2),
	CLK(NULL, "pll1_sysclk3", &pll1_sysclk3),
	CLK(NULL, "pll1_sysclk4", &pll1_sysclk4),
	CLK(NULL, "pll1_sysclk5", &pll1_sysclk5),
	CLK(NULL, "core", &core_clk),
	CLK("i2c_davinci.1", NULL, &i2c_clk),
	CLK("watchdog", NULL, &watchdog_clk),
	CLK("", NULL, NULL)
};

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{
	printk(KERN_INFO "Designed for the EVM6457 board, Texas Instruments.\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6457 specific initialization");
}

static __init int evm_init(void)
{
	evm_setup_edma();
	evm_setup_i2c();
	evm_setup_nand();
	return 0;
}

arch_initcall(evm_init);
