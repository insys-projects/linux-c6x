/*
 *  linux/arch/c6x/platforms/board-evm6472.c
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
#include <linux/i2c/sc16is7xx.h>
#include <linux/kernel_stat.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-evm6488.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/timer.h>
#include <asm/percpu.h>
#include <asm/clock.h>

#include <mach/i2c.h>
#include <mach/board.h>

#ifdef CONFIG_RAPIDIO_TCI648X
#include <linux/rio.h>
#include <asm/rio.h>

struct tci648x_rio_board_controller_info evm6472_rio_controller = {
	0x3,                         /* bitfield of port(s) to probe on this controller */
	TCI648X_RIO_MODE_0,          /* SERDES configuration (BOOTMODE8) */
	0,                           /* host id */
	RIO_DO_ENUMERATION,          /* initialisation method */
	1                            /* large size (16bit)*/
};

static struct platform_device evm6472_rio_device = {
	.name           = "tci648x-rapidio",
	.id             = 1,
	.dev		= { .platform_data = &evm6472_rio_controller },
};

static void __init evm_init_rio(void)
{
	platform_device_register(&evm6472_rio_device);
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

#if defined(CONFIG_MTD_NAND_EVM6488) || defined(CONFIG_MTD_NAND_EVM6488_MODULE)
static struct mtd_partition evm_nand_parts[] = {
	[0] = {
		.name	= "evm-nand",
		.size	= MTDPART_SIZ_FULL,
		.offset	= 0,
	},
};

static struct gpio_nand_platdata evm_nand_platdata = {
	.parts = evm_nand_parts,
	.num_parts = ARRAY_SIZE(evm_nand_parts),
	.chip_delay = 25,
};

static struct platform_device evm_nand = {
	.name		= "nand-evm6488",
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

static struct pll_data pll1_data = {
	.num       = 1,
	.phys_base = ARCH_PLL1_BASE,
};

static struct pll_data pll2_data = {
	.num       = 2,
	.phys_base = ARCH_PLL2_BASE,
};

static struct pll_data pll3_data = {
	.num       = 3,
	.phys_base = ARCH_PLL3_BASE,
};

static struct clk clkin1 = {
	.name = "clkin1",
	.rate = 25000000,
};

static struct clk clkin2 = {
	.name = "clkin2",
	.rate = 25000000,
};

static struct clk clkin3 = {
	.name = "clkin3",
	.rate = 26660000,
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
	.div = 1,
};

static struct clk pll1_sysclk3 = {
	.name = "pll1_sysclk3",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 1,
};

static struct clk pll1_sysclk4 = {
	.name = "pll1_sysclk4",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 1,
};

static struct clk pll1_sysclk5 = {
	.name = "pll1_sysclk5",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 1,
};

static struct clk pll1_sysclk6 = {
	.name = "pll1_sysclk6",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 1,
};

static struct clk pll1_sysclk7 = {
	.name = "pll1_sysclk7",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 3,
};

static struct clk pll1_sysclk8 = {
	.name = "pll1_sysclk8",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 4,
};

static struct clk pll1_sysclk9 = {
	.name = "pll1_sysclk9",
	.parent = &pll1_clk,
	.flags = CLK_PLL | FIXED_DIV_PLL,
	.div = 2,
};

static struct clk pll1_sysclk10 = {
	.name = "pll1_sysclk10",
	.parent = &pll1_clk,
	.flags = CLK_PLL,
	.div = PLLDIV10,
};

static struct clk i2c_clk = {
	.name = "i2c",
	.parent = &pll1_sysclk8,
};

static struct clk core_clk = {
	.name = "core",
	.parent = &pll1_sysclk1,
};

static struct clk_lookup evm_clks[] = {
	CLK(NULL, "pll1", &pll1_clk),
	CLK(NULL, "pll1_sysclk1", &pll1_sysclk1),
	CLK(NULL, "pll1_sysclk2", &pll1_sysclk2),
	CLK(NULL, "pll1_sysclk3", &pll1_sysclk3),
	CLK(NULL, "pll1_sysclk4", &pll1_sysclk4),
	CLK(NULL, "pll1_sysclk5", &pll1_sysclk5),
	CLK(NULL, "pll1_sysclk6", &pll1_sysclk6),
	CLK(NULL, "pll1_sysclk7", &pll1_sysclk7),
	CLK(NULL, "pll1_sysclk8", &pll1_sysclk8),
	CLK(NULL, "pll1_sysclk9", &pll1_sysclk9),
	CLK(NULL, "pll1_sysclk10", &pll1_sysclk10),
	CLK("i2c_davinci.1", NULL, &i2c_clk),
	CLK(NULL, "core", &core_clk),
	CLK(NULL, NULL, NULL)
};

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{
	int i, ret;

	printk(KERN_INFO "Designed for the EVM6472 board, Texas Instruments.\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6472 specific initialization");
}

__init int evm_init(void)
{
	evm_setup_i2c();
	evm_setup_nand();
	return 0;
}

arch_initcall(evm_init);
