/*
 * Copyright (C) 2010, 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
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

#include <mach/board.h>

#ifdef CONFIG_RAPIDIO_TCI648X
#include <linux/rio.h>
#include <asm/rio.h>

struct tci648x_rio_board_controller_info evm6474l_rio_controller = {
	0x3,                         /* bitfield of port(s) to probe on this controller */
	TCI648X_RIO_MODE_0,          /* SERDES configuration (BOOTMODE8) */
	0,                           /* host id */
	RIO_DO_ENUMERATION,          /* initialisation method */
	1                            /* large size (16bit)*/
};

static struct resource evm6474l_rio_resources[] = {
	{
		.name	= "LSU",
		.start	= TCI648X_LSU_CHANNEL_EVENT,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "ICCR",
		.start	= TCI648X_ICCR_CHANNEL_EVENT,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "RATE",
		.start	= TCI648X_RATE_CHANNEL_EVENT,
		.flags	= IORESOURCE_DMA,
	},
};

static struct platform_device evm6474l_rio_device = {
	.name           = "tci648x-rapidio",
	.id             = 1,
	.dev		= { 
		.platform_data = &evm6474l_rio_controller,
	},
	.num_resources	= ARRAY_SIZE(evm6474l_rio_resources),
	.resource	= evm6474l_rio_resources,
};

static int __init evm_init_rio(void)
{
	return platform_device_register(&evm6474l_rio_device);
}

core_initcall(evm_init_rio);
#endif

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

#define I2C_UART_GPIO 12

static void __init board_setup_i2c(void)
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
#define board_setup_i2c()
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

/* Six Transfer Controllers on TMS320C6474 */
static const s8
queue_tc_mapping[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{4, 4},
	{5, 5},
	{-1, -1},
};

static const s8
queue_priority_mapping[][2] = {
	/* {event queue no, Priority} */
	{0, 4},	/* FIXME: what should these priorities be? */
	{1, 0},
	{2, 5},
	{3, 1},
	{4, 2},
	{5, 3},
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
	/*
	 * NB: Keep "edma0" IRQ first so it can be easily found at
	 *     runtime. It is based on core id.
	 */
	{
		.name	= "edma0",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma0_err",
		.start	= EDMA_IRQ_CCERRINT,
		.flags	= IORESOURCE_IRQ,
	},
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
		.name	= "edma_tc4",
		.start	= EDMA_TC4_BASE,
		.end	= EDMA_TC4_BASE + 0x03FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc5",
		.start	= EDMA_TC5_BASE,
		.end	= EDMA_TC5_BASE + 0x03FF,
		.flags	= IORESOURCE_MEM,
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


static void __init board_setup_edma(void)
{
	int status;

	/* this is based on coreid, so must be calculated at runtime */
	edma_resources[0].start = EDMA_IRQ_CCINT;

	status = platform_device_register(&edma_device);
	if (status != 0)
		pr_debug("setup_edma --> %d\n", status);
}
#else
#define board_setup_edma()
#endif /* CONFIG_EDMA3 */

static struct pll_data pll1_data = {
	.num       = 1,
	.phys_base = ARCH_PLL1_BASE,
};

static struct clk clkin1 = {
	.name = "clkin1",
	.rate = 50000000,
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

static struct clk pll1_sysclk7 = {
	.name = "pll1_sysclk7",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 1,
};

static struct clk pll1_sysclk9 = {
	.name = "pll1_sysclk9",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 3,
};

static struct clk pll1_sysclk10 = {
	.name = "pll1_sysclk10",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 6,
};

static struct clk pll1_sysclk11 = {
	.name = "pll1_sysclk11",
	.parent = &pll1_clk,
	.flags = CLK_PLL,
	.div = PLLDIV11,
};

static struct clk pll1_sysclk12 = {
	.name = "pll1_sysclk12",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 2,
};

static struct clk pll1_sysclk13 = {
	.name = "pll1_sysclk13",
	.parent = &pll1_clk,
	.flags = CLK_PLL,
	.div = PLLDIV13,
};

static struct clk i2c_clk = {
	.name = "i2c",
	.parent = &pll1_sysclk10,
};

static struct clk core_clk = {
	.name = "core",
	.parent = &pll1_sysclk7,
};

static struct clk watchdog_clk = {
	.name = "watchdog",
	.parent = &pll1_sysclk10,
};

static struct clk_lookup evm_clks[] = {
	CLK(NULL, "pll1", &pll1_clk),
	CLK(NULL, "pll1_sysclk7", &pll1_sysclk7),
	CLK(NULL, "pll1_sysclk9", &pll1_sysclk9),
	CLK(NULL, "pll1_sysclk10", &pll1_sysclk10),
	CLK(NULL, "pll1_sysclk11", &pll1_sysclk11),
	CLK(NULL, "pll1_sysclk12", &pll1_sysclk12),
	CLK(NULL, "pll1_sysclk13", &pll1_sysclk13),
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
	printk("Designed for the EVM6474 Lite EVM\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6474 Lite specific initialization");
}

__init int evm_init(void)
{
	board_setup_edma();
	board_setup_i2c();
	evm_setup_nand();

	return 0;
}

arch_initcall(evm_init);
