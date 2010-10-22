/*
 *  linux/arch/c6x/platforms/board-dsk6455.c
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

#include <mach/gemac.h>
#include <mach/gmdio.h>
#include <mach/board.h>

/*
 * Resources present on the DSK6455 board
 */
static struct resource _flash_res = {
	.name  = "Flash",
	.start = 0xb0000000,
	.end   = 0xbfffffff,
	.flags = IORESOURCE_MEM,
};
static struct resource _cpld_async_res = {
	.name  = "CPLD async",
	.start = 0xa0000000,
	.end   = 0xa0000008,
	.flags = IORESOURCE_MEM,
};

#define NR_RESOURCES 2
static struct resource *dsk_resources[NR_RESOURCES] = 
	{ &_flash_res, &_cpld_async_res };


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
		.name           = "MDIO_REG_BASE",
		.start          =  MDIO_REG_BASE,
		.end            =  MDIO_REG_BASE + 0x7FF,
		.flags          =  IORESOURCE_IO,
	},
	{
		.name           = "IRQ_SRC",
		.start          =  IRQ_EMAC,
		.flags          =  IORESOURCE_IRQ,
	},
};

static struct platform_device emac_dev0 = {
        .name           = "EMAC",
        .id             = 0,
	.resource       = emac_resources0,
        .num_resources  = ARRAY_SIZE(emac_resources0),
};

static int setup_emac(void)
{
        int status;
	unsigned long val;

        status  = platform_device_register(&emac_dev0);
        if (status != 0)
                pr_debug("setup_emac --> %d\n", status);
	else {
		/* Enable EMAC device (in regs PERLOCK & PERCFG0) */
		val = dscr_get_reg(DSCR_PERCFG0);
		dscr_set_device(val | DSCR_B_PERCFG0_EMAC, DSCR_PERCFG0);

		/* Wait for enabling (reg PERSTAT0) */
		val = 0;
		while (val != 0x1) {
			val = dscr_get_reg(DSCR_PERSTAT0);
			val = ((val & 0x1C0) >> 6);
		}
	}
	return status;
}
#else
static inline int setup_emac(void) { return 0; }
#endif

#ifdef CONFIG_I2C
static struct at24_platform_data at24_eeprom_data = {
	.byte_len	= 0x100000 / 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info dsk_i2c_info[] = {
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c1024", 0x50),
	  .platform_data = &at24_eeprom_data,
	},
#endif
};

static void __init dsk_setup_i2c(void)
{
	i2c_register_board_info(1, dsk_i2c_info,
				ARRAY_SIZE(dsk_i2c_info));
}
#else
#define dsk_setup_i2c()
#endif /* CONFIG_I2C */

static struct pll_data pll1_data = {
	.num       = 1,
	.phys_base = ARCH_PLL1_BASE,
};

static struct clk clkin1 = {
	.name = "clkin1",
	.rate = 50000000,
};

static struct clk pll1_clk = {
	.name = "pll1",
	.parent = &clkin1,
	.pll_data = &pll1_data,
	.flags = CLK_PLL | PLL_HAS_PREDIV,
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

static struct clk_lookup evm_clks[] = {
	CLK(NULL, "pll1", &pll1_clk),
	CLK(NULL, "pll1_sysclk2", &pll1_sysclk2),
	CLK(NULL, "pll1_sysclk3", &pll1_sysclk3),
	CLK(NULL, "pll1_sysclk4", &pll1_sysclk4),
	CLK(NULL, "pll1_sysclk5", &pll1_sysclk5),
	CLK("i2c_davinci.1", NULL, &i2c_clk),
	CLK("", NULL, NULL)
};

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	int i;

	printk("Designed for the DSK6455 board, Spectrum Digital Inc.\n");

	/* Initialize DSK6455 resources */
	iomem_resource.name = "Memory";
	for (i = 0; i < NR_RESOURCES; i++)
		request_resource(&iomem_resource, dsk_resources[i]);

	/* Map our IRQs */
	irq_map(IRQ_TINT1, IRQ_CLOCKEVENTS);
	irq_map(IRQ_EMACINT, IRQ_EMAC);
	irq_map(IRQ_I2CINT, IRQ_DAVINCI_I2C);

	/* Initialize led register */
	cpld_set_reg(DSK6455_CPLD_USER, 0x0);

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of DSK6455 specific initialization");
}

static int __init evm_init(void)
{
	dsk_setup_i2c();
        return setup_emac();
}

arch_initcall(evm_init);

/*
 * NOR Flash support.
 */
#ifdef CONFIG_MTD
static struct map_info nor_map = {
	.name		= "NOR-flash",
	.phys		= 0xB0000000,
	.size		= 0x400000,
	.bankwidth	= 1,
};
static struct mtd_info *mymtd;
#ifdef CONFIG_MTD_PARTITIONS
static int nr_parts;
static struct mtd_partition *parts;
static const char *part_probe_types[] = {
	"cmdlinepart",
	NULL
};
#endif

static __init int nor_init(void)
{
	nor_map.virt = ioremap(nor_map.phys, nor_map.size);
	simple_map_init(&nor_map);
	mymtd = do_map_probe("cfi_probe", &nor_map);
	if (mymtd) {
		mymtd->owner = THIS_MODULE;

#ifdef CONFIG_MTD_PARTITIONS
		nr_parts = parse_mtd_partitions(mymtd,
						part_probe_types,
						&parts, 0);
		if (nr_parts > 0)
			add_mtd_partitions(mymtd, parts, nr_parts);
		else
			add_mtd_device(mymtd);
#else
		add_mtd_device(mymtd);
#endif
	}
	return 0;
}

late_initcall(nor_init);
#endif
