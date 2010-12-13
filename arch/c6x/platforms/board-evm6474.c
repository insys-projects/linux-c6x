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

#ifdef CONFIG_MCBSP
#include <asm/mcbsp.h>
#include <asm/edma.h>

static const struct mcbsp_info evm6474_mcbsp_info[] = {
	[0] = {.phys_base   = IO_ADDRESS(MCBSP0_BASE_ADDR),
	       .dma_rx_sync = DMA_MCBSP0_RX,
	       .dma_tx_sync = DMA_MCBSP0_TX,
	       .dma_rx_data = MCBSP0_EDMA_RX_DATA,
	       .dma_tx_data = MCBSP0_EDMA_TX_DATA,
	       .rx_irq      = IRQ_REVT0,
	       .tx_irq      = IRQ_XEVT0},

	[1] = {.phys_base   = IO_ADDRESS(MCBSP1_BASE_ADDR),
	       .dma_rx_sync = DMA_MCBSP1_RX,
	       .dma_tx_sync = DMA_MCBSP1_TX,
	       .dma_rx_data = MCBSP1_EDMA_RX_DATA,
	       .dma_tx_data = MCBSP1_EDMA_TX_DATA,
	       .rx_irq      = IRQ_REVT1,
	       .tx_irq      = IRQ_XEVT1},
};

static struct platform_device evm6474_mcbsp_device0 = {
	.name           = "mcbsp",
	.id             = 1,
	.dev		= { .platform_data = &evm6474_mcbsp_info[0] },
};

static struct platform_device evm6474_mcbsp_device1 = {
	.name           = "mcbsp",
	.id             = 2,
	.dev		= { .platform_data = &evm6474_mcbsp_info[1] },
};

static void __init evm_init_mcbsp(void)
{
	platform_device_register(&evm6474_mcbsp_device0); /* McBSP0 */
	platform_device_register(&evm6474_mcbsp_device1); /* McBSP1 */
}

core_initcall(evm_init_mcbsp);
#endif

#ifdef CONFIG_MCBSP_UART
#include <asm/mcbsp-uart.h>

static const struct mcbsp_uart_info evm6474_mcbsp_uart_info = {
    .mcbsp_id   = 0, /* first McBSP (McBSP0) used for UART */
    .mcbsp_num  = 2, /* use two McBSP (McBSP0, McBSP1) */
};

static struct platform_device evm6474_mcbsp_uart_device = {
	.name           = "mcbsp_serial",
	.id             = 1,
	.dev		= { .platform_data = &evm6474_mcbsp_uart_info },
};

static void __init evm_init_mcbsp_uart(void)
{
	platform_device_register(&evm6474_mcbsp_uart_device);
}

core_initcall(evm_init_mcbsp_uart);
#endif

#ifdef CONFIG_I2C
static struct at24_platform_data at24_eeprom_data = {
       .byte_len       = 0x100000 / 8,
       .page_size      = 256,
       .flags          = AT24_FLAG_ADDR16,
};

static struct i2c_board_info evm_i2c_info[] = {
#ifdef CONFIG_EEPROM_AT24
       { I2C_BOARD_INFO("24c1024", 0x50),
         .platform_data = &at24_eeprom_data,
       },
#endif
};

static void __init board_setup_i2c(void)
{
       i2c_register_board_info(1, evm_i2c_info,
                               ARRAY_SIZE(evm_i2c_info));
}
#else
#define board_setup_i2c()
#endif /* CONFIG_I2C */

static struct pll_data pll1_data = {
	.num       = 1,
	.phys_base = ARCH_PLL1_BASE,
};

static struct clk clkin1 = {
	.name = "clkin1",
//	.rate = 61440000, This is when using SYSCLK (SW5 CORE_CLOK_SEL to ON)
	.rate = 50000000, /* default one is 50MHz clock */
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

static struct clk mcbsp1_clk = {
	.name = "mcbsp1",
	.parent = &pll1_sysclk10,
};

static struct clk mcbsp2_clk = {
	.name = "mcbsp2",
	.parent = &pll1_sysclk10,
};

static struct clk core_clk = {
	.name = "core",
	.parent = &pll1_sysclk7,
};

static struct clk_lookup evm_clks[] = {
	CLK(NULL, "pll1", &pll1_clk),
	CLK(NULL, "pll1_sysclk7", &pll1_sysclk7),
	CLK(NULL, "pll1_sysclk9", &pll1_sysclk9),
	CLK(NULL, "pll1_sysclk10", &pll1_sysclk10),
	CLK(NULL, "pll1_sysclk11", &pll1_sysclk11),
	CLK(NULL, "pll1_sysclk12", &pll1_sysclk12),
	CLK(NULL, "pll1_sysclk13", &pll1_sysclk13),
	CLK("i2c_davinci.1", NULL, &i2c_clk),
	CLK("mcbsp.1", NULL, &mcbsp1_clk),
	CLK("mcbsp.2", NULL, &mcbsp2_clk),
	CLK(NULL, "core", &core_clk),
	CLK("", NULL, NULL)
};

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	int i, ret;

	printk("Designed for the EVM6474 board, Texas Instruments.\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVM6474 specific initialization");
}

__init void evm_init(void)
{
	board_setup_i2c();
}

arch_initcall(evm_init);
