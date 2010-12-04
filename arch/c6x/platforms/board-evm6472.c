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
#include <asm/gemac.h>

#include <mach/i2c.h>
#include <mach/board.h>

#ifdef CONFIG_TMS320C64X_GEMAC
#ifdef CONFIG_TMS320C64X_GEMAC_0
static struct resource emac_resources0 [] = {
	{
		.name           = "EMAC_REG_BASE",
		.start          =  EMAC0_REG_BASE,
		.end            =  EMAC0_REG_BASE + 0xFFF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "ECTL_REG_BASE",
		.start          =  ECTL0_REG_BASE,
		.end            =  ECTL0_REG_BASE + 0x7FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "EMAC_DSC_BASE",
		.start          =  EMAC0_DSC_BASE,
		.end            =  EMAC0_DSC_BASE + 0x17FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "IRQ_SRC",
		.start          =  IRQ_EMACRXINT0,
		.flags          =  IORESOURCE_IRQ,
	},
};

static struct platform_device emac_dev0 = {
        .name           = "EMAC",
        .id             = 0,
	.resource       = emac_resources0,
        .num_resources  = ARRAY_SIZE(emac_resources0),
};
#endif

#ifdef CONFIG_TMS320C64X_GEMAC_1
static struct resource emac_resources1 [] = {
	{
		.name           = "EMAC_REG_BASE",
		.start          =  EMAC1_REG_BASE,
		.end            =  EMAC1_REG_BASE + 0xFFF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "ECTL_REG_BASE",
		.start          =  ECTL1_REG_BASE,
		.end            =  ECTL1_REG_BASE + 0x7FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "EMAC_DSC_BASE",
		.start          =  EMAC1_DSC_BASE,
		.end            =  EMAC1_DSC_BASE + 0x17FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "IRQ_SRC",
		.start          =  IRQ_EMACRXINT1,
		.flags          =  IORESOURCE_IRQ,
	},
};

static struct platform_device emac_dev1 = {
        .name           = "EMAC",
        .id             = 1,
	.resource       = emac_resources1,
        .num_resources  = ARRAY_SIZE(emac_resources1),
};
#endif

static void setup_emac(void)
{
        int status;

#ifdef CONFIG_TMS320C64X_GEMAC_0
        status  = platform_device_register(&emac_dev0);
        if (status != 0)
                pr_debug("setup_emac0 --> %d\n", status);
        //else
        //  /* Power domain need to be activated here */
#endif
#ifdef CONFIG_TMS320C64X_GEMAC_1
        status  = platform_device_register(&emac_dev1);
        if (status != 0)
                pr_debug("setup_emac1 --> %d\n", status);
	else {
		val = dscr_get_reg(DSCR_DEVCTL);
		dscr_set_device(val | DSCR_B_DEVCTL_EMAC1, DSCR_DEVCTL);
	}
#endif
}
#else
static void setup_emac(void) {}
#endif

#ifdef CONFIG_I2C
static struct at24_platform_data at24_eeprom_data = {
	.byte_len	= 0x100000 / 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};

static struct pca953x_platform_data pca9536_data = {
	.gpio_base = 16,
};

#ifdef CONFIG_SERIAL_SC16IS7XX
static struct sc16is7xx_platform_data uart_data = {
	.baud_base = 921600,
};
#endif

static struct i2c_board_info evm_i2c_info[] = {
#ifdef CONFIG_SERIAL_SC16IS7XX
	{ I2C_BOARD_INFO("sc16is750", 0x4d),
	  .irq = IRQ_GPIO15,
	  .platform_data = &uart_data,
	},
#endif
	{ I2C_BOARD_INFO("pca9536", 0x41),
	  .platform_data = &pca9536_data
	},
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c1024", 0x50),
	  .platform_data = &at24_eeprom_data,
	},
#endif
};

static void __init evm_setup_i2c(void)
{
	i2c_register_board_info(1, evm_i2c_info,
				ARRAY_SIZE(evm_i2c_info));
}
#else
#define evm_setup_i2c()
#endif /* CONFIG_I2C */

static struct pll_data pll1_data = {
	.num       = 1,
	.phys_base = ARCH_PLL1_BASE,
};

static struct clk clkin1 = {
	.name = "clkin1",
	.rate = 25000000,
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
	.div = 6,
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
	CLK(NULL, NULL, NULL)
};

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

static void __init board_init_IRQ(void)
{
	/* Map our IRQs */
	irq_map(IRQ_TINT, INT11);

#ifdef CONFIG_TMS320C64X_GEMAC_0
	irq_map(IRQ_EMACRXINT0, INT7);
	irq_map(IRQ_EMACTXINT0, INT8);
#endif
#ifdef CONFIG_TMS320C64X_GEMAC_1
	irq_map(IRQ_EMACRXINT1, INT9);
	irq_map(IRQ_EMACTXINT1, INT10);
#endif
}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{
	int i, ret;

	printk(KERN_INFO "Designed for the EVM6472 board, Texas Instruments.\n");

	gpio_direction(0xFFFF);  /* all input */
	
	/* setup LED pins as outputs */
	gpio_pin_direction(LED0_GPIO_PIN, GPIO_OUTPUT);
	gpio_pin_direction(LED1_GPIO_PIN, GPIO_OUTPUT);

#if defined(CONFIG_SERIAL_SC16IS7XX)
	/* setup GP15 for interrupt from i2c UART */
	gpio_int_edge_detection_set(15, GPIO_FALLING_EDGE);
	gpio_bank_int_enable();
#endif

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;
	mach_init_IRQ      = board_init_IRQ;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6486 specific initialization");
}

__init int evm_init(void)
{
	evm_setup_i2c();
        setup_emac();
	return 0;
}

arch_initcall(evm_init);
