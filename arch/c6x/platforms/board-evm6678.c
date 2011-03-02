/*
 *  linux/arch/c6x/platforms/board-evm6678.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Sandeep Paulraj <s-paulraj@ti.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/clk.h>
#include <linux/kernel_stat.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/timer.h>
#include <asm/percpu.h>
#include <asm/clock.h>

#include <mach/board.h>
#include <mach/nand.h>

SOC_CLK_DEF(100000000); /* SYSCLK is a 100 MHz clock */

static struct clk_lookup evm_clks[] = {
        SOC_CLK(),
	CLK("", NULL, NULL)
};

#if defined(CONFIG_MTD_NAND_DAVINCI)
static struct mtd_partition evm6678_nand_parts[] = {
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

static struct davinci_nand_pdata evmc6678_nand_data = {
	.mask_cle 		= 0x4000,
	.mask_ale 		= 0x2000,
	.parts			= evm6678_nand_parts,
	.nr_parts		= ARRAY_SIZE(evm6678_nand_parts),
	.ecc_mode		= NAND_ECC_HW,
	.ecc_bits		= 4,
	.options		= 0,
};

#define ASYNC_EMIF_CONTROL_BASE		0x20C00000
#define ASYNC_EMIF_DATA_CE0_BASE	0x70000000

static struct resource evmc6678_nand_resources[] = {
	{
		.start		= ASYNC_EMIF_DATA_CE0_BASE,
		.end		= ASYNC_EMIF_DATA_CE0_BASE + 0x3FFFFFF,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= ASYNC_EMIF_CONTROL_BASE,
		.end		= ASYNC_EMIF_CONTROL_BASE + 0xFF,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device evmc6678_nand_device = {
	.name			= "davinci_nand",
	.id			= 0,

	.num_resources		= ARRAY_SIZE(evmc6678_nand_resources),
	.resource		= evmc6678_nand_resources,

	.dev			= {
		.platform_data	= &evmc6678_nand_data,
	},
};

static void __init evm_setup_nand(void)
{
	platform_device_register(&evmc6678_nand_device);
}
#else
static inline void evm_setup_nand(void) {}
#endif

#ifdef CONFIG_I2C
static struct at24_platform_data at24_eeprom_data = {
	.byte_len	= 1024 * 1024 / 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info evm_i2c_info[] = {
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c1024", 0x50),
	  .platform_data = &at24_eeprom_data,
	},
#endif
};

static int __init board_setup_i2c(void)
{
	return i2c_register_board_info(1, evm_i2c_info, ARRAY_SIZE(evm_i2c_info));
}
core_initcall(board_setup_i2c);
#endif /* CONFIG_I2C */

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
#include <linux/serial_8250.h>
#include <linux/serial.h>
/*
 * Configuration for 16550-like UART
 */
static struct plat_serial8250_port serial8250_platform_data [] = {
        {
                .membase  = (void *) UART_BASE_ADDR,
                .mapbase  = UART_BASE_ADDR,
                .irq      = IRQ_UART,
                .flags    = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
                .iotype   = UPIO_MEM32,
                .regshift = 2,
        },
        {
                .flags          = 0
        },
};

static struct platform_device serial8250_device = {
        .id             = PLAT8250_DEV_PLATFORM,
        .name           = "serial8250",
        .dev            = {
                .platform_data = serial8250_platform_data,
        },
};

#include <linux/serial_reg.h>
#define SERIAL_OUT(offset, value) writel(value, UART_BASE_ADDR + ((offset) << 2))

static int __init evm_init_uart(void)
{
	struct clk *clk;

	/* 
	 * Enable UART
	 */
	SERIAL_OUT(0xc, 0x6001); /* UTRST | URRST | FREE */

	/*
	 *  Retrieve the UART clock
	 */
	clk = clk_get(NULL, "uart");
	if (IS_ERR(clk))
		return -ENODEV;
	else
		serial8250_platform_data[0].uartclk = clk_get_rate(clk);

	return platform_device_register(&serial8250_device);
}

core_initcall(evm_init_uart);
#endif

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	printk("Designed for the EVMC6678 board, Texas Instruments.\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVMC6678 specific initialization");
}

__init int evm_init(void)
{
	evm_setup_nand();

	return 0;
}

arch_initcall(evm_init);
