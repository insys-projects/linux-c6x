/*
 *  linux/arch/c6x/platforms/platform-c64xx.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
 *  Updated: Mark Salter <msalter@redhat.com>
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
#include <mach/gemac.h>

/*
 * Resources present on the SoC
 */
static struct resource c6x_soc_res = {
	"C64X+ SOC peripherals",
#if defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6474)
	0x01800000, 0x02f60000
#elif defined (CONFIG_SOC_TMS320C6455)
	0x01800000, 0x2cffffff
#else
#error "No SoC peripheral address space defined"
#endif
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
	.probes    = part_probe_types,
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

#ifdef CONFIG_TMS320C64X_GEMAC
#if defined(CONFIG_TMS320C64X_GEMAC_0) || !defined(CONFIG_SOC_TMS320C6472)
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
		.start          =  IRQ_EMACRXINT, 
		.end            =  IRQ_EMACTXINT,
		.flags          =  IORESOURCE_IRQ,
	},
	{
		.name           = "IRQ_DST",
		.start          =  IRQ_EMAC_RX_0,
		.end            =  IRQ_EMAC_TX_0,
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
		.end            =  IRQ_EMACTXINT1,
		.flags          =  IORESOURCE_IRQ,
	},
	{
		.name           = "IRQ_DST",
		.start          =  IRQ_EMAC_RX_1,
		.end            =  IRQ_EMAC_TX_1,
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

#if defined(CONFIG_TMS320C64X_GEMAC_0) || !defined(CONFIG_SOC_TMS320C6472)
        status  = platform_device_register(&emac_dev0);
        if (status != 0)
                pr_debug("setup_emac0 --> %d\n", status);
#ifdef CONFIG_SOC_TMS320C6455
	else {
		unsigned long val;

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
#endif /* CONFIG_SOC_TMS320C6455 */
#endif /* defined(CONFIG_TMS320C64X_GEMAC_0) || !defined(CONFIG_SOC_TMS320C6472) */

#ifdef CONFIG_TMS320C64X_GEMAC_1
        status  = platform_device_register(&emac_dev1);
        if (status != 0)
                pr_debug("setup_emac1 --> %d\n", status);
#ifdef CONFIG_SOC_TMS320C6472
	else {
		unsigned long val;
		val = dscr_get_reg(DSCR_DEVCTL);
		dscr_set_device(val | DSCR_B_DEVCTL_EMAC1, DSCR_DEVCTL);
	}
#endif /* CONFIG_SOC_TMS320C6472 */
#endif /* CONFIG_TMS320C64X_GEMAC_1 */
}
#else /* CONFIG_TMS320C64X_GEMAC */
static void setup_emac(void) {}
#endif

#if defined(CONFIG_SOC_TMS320C6474)
unsigned int arch_get_silicon_rev(void)
{
	u32   jtagid_val  = *(u32 *) DSCR_JTAGID;
	u32   silicon_rev = (jtagid_val >> 28) & 0xf;

	return silicon_rev;	
}
char* arch_compute_silicon_rev(u32 silicon_rev)
{
	char *str;
	switch(silicon_rev) {
	case 0x1:
		str = "1.2";
		break;
	case 0x2:
		str = "1.3";
		break;
	case 0x3:
		str = "2.0";
		break;
	case 0x4:
		str = "2.1";
		break;
	default:
		str = "unknown";
		break;
	}
	return str;
}
#endif

#ifndef CONFIG_NK
static void init_pll(void)
{
	int i;

#if defined(CONFIG_SOC_TMS320C6474) /* should be done elsewhere */
	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLENSRC);
	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLEN);

	/* Wait 4 cycles of the slowest of PLLOUT or reference clock source (CLKIN).*/
	for (i = 0 ; i < 100 ; i++);

	pll1_setbit_reg(PLLCTL, PLLCTL_PLLRST);
	pll1_set_reg(PLLM, PLLM_VAL(20));

	/*
	 * This can't be right. There is no previder here...
	pll1_set_reg(PLLPREDIV, PLLPREDIV_VAL(10) | PLLPREDIV_EN);
	*/

	pll1_wait_gostat();

	/* Wait until GOSTAT bit is cleared*/
//	while(pll1_get_reg(PLL_PLLSTAT) & PLLSTAT_GOSTAT);
	
	pll1_set_reg(PLLDIV11, PLLDIV_RATIO(10) | PLLDIV_EN);
	pll1_set_reg(PLLDIV13, PLLDIV_RATIO(6)  | PLLDIV_EN);
	
	pll1_setbit_reg(PLLCMD, PLLCMD_GOSTAT);

	pll1_wait_gostat();
	
	/* Wait until GOSTAT bit is cleared*/
//	while(pll1_get_reg(PLLSTAT) & PLLSTAT_GOSTAT);

	for (i = 0 ; i < 1000 ; i++);

	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLRST);

	for (i = 0 ; i < 4000 ; i++);

	pll1_setbit_reg(PLLCTL, PLLCTL_PLLEN);
#endif

#if defined(CONFIG_SOC_TMS320C6472) && defined(CONFIG_TMS320C64X_GEMAC)
	/* PLL2 configuration (EMAC) */
	pll2_clearbit_reg(PLLCTL, PLLCTL_PLLENSRC);
	pll2_clearbit_reg(PLLCTL, PLLCTL_PLLEN);

	/* Wait 4 cycles of the slowest of PLLOUT or reference clock source (CLKIN).*/
	for (i=0 ; i < 100 ; i++);

	pll2_setbit_reg(PLLCTL, PLLCTL_PLLRST);

	/* Wait for PLL to properly reset.*/
	for (i=0 ; i< 4000 ; i++);

	pll2_clearbit_reg(PLLCTL, PLLCTL_PLLRST);

	/* Wait for PLL to lock */
	for (i=0 ; i < 4000 ; i++);

	pll2_setbit_reg(PLLCTL, PLLCTL_PLLEN);
#endif
}
#endif

static void init_power(void)
{
#if defined(CONFIG_SOC_TMS320C6474)

	*(volatile unsigned int *)  PSC_PDCTL0 |= 0x00000001;
	*(volatile unsigned int *)  PSC_MDCTL0 |= 0x00000003;
	*(volatile unsigned int *)  PSC_MDCTL1 |= 0x00000003;
	*(volatile unsigned int *)  PSC_MDCTL2 |= 0x00000003;
 
	*(volatile unsigned int *)  PSC_PDCTL1 |= 0x00000001;
	*(volatile unsigned int *)  PSC_MDCTL6 |= 0x00000003;

	*(volatile unsigned int *)  PSC_PDCTL2 |= 0x00000001;
	*(volatile unsigned int *)  PSC_MDCTL7 |= 0x00000003;

	*(volatile unsigned int *)  PSC_PDCTL3 |= 0x00000001;
	*(volatile unsigned int *)  PSC_MDCTL8 |= 0x00000003;

	*(volatile unsigned int *)  PSC_PDCTL4 |= 0x00000001;
	*(volatile unsigned int *)  PSC_MDCTL9 |= 0x00000003;

	*(volatile unsigned int *)  PSC_PDCTL5 |= 0x00000001;
	*(volatile unsigned int *)  PSC_MDCTL10 |= 0x00000003;

	*(volatile unsigned int *)  PSC_PTCMD  |= 0x0000003f;

#ifdef CONFIG_RAPIDIO_TCI648X
	*(volatile unsigned int *)  PSC_PDCTL2 |= 1;
	*(volatile unsigned int *)  PSC_MDCTL7 |= 3; /* sRIO */
	*(volatile unsigned int *)  PSC_PTCMD  |= 4;

#endif /* CONFIG_RAPIDIO_TCI648X */
#endif /* CONFIG_SOC_TMS320C6474 */

#ifdef CONFIG_SOC_TMS320C6472

#ifdef CONFIG_TMS320C64X_GEMAC_0
	*(volatile unsigned int *)  PSC_MDCTL7 |= 0x00000103; /* GEMAC0 */
#endif /* CONFIG_TMS320C64X_GEMAC_0 */

#ifdef CONFIG_TMS320C64X_GEMAC_1
	*(volatile unsigned int *)  PSC_MDCTL8 |= 0x00000103; /* GEMAC1 */
#endif /* CONFIG_TMS320C64X_GEMAC_1 */

#ifdef CONFIG_RAPIDIO_TCI648X
	*(volatile unsigned int *)  PSC_MDCTL6 |= 0x00000103; /* SRIO */
#endif /* CONFIG_RAPIDIO_TCI648X */

	*(int *)  PSC_PTCMD  |= 0x00000001;

#endif/* CONFIG_SOC_TMS320C6472 */
}

void c6x_soc_setup_arch(void)
{
	unsigned long val;

 	/* Initialize C64x+ IRQs */          	
#ifndef CONFIG_NK
	clear_all_irq(); /* acknowledge all pending irqs */

	init_pll();
#else
	irq_IER = 0;
#endif

	init_power();

#if defined(CONFIG_SOC_TMS320C6474)
	/* Enable timers and devices (in regs PERLOCK & PERCFG0) */
	dscr_set_reg(DSCR_PERLOCK, DSCR_LOCKVAL);

	dscr_set_reg(DSCR_PERCFG0, 
	             DSCR_B_PERCFG0_TIMER0
		     | DSCR_B_PERCFG0_TIMER1 
		     | DSCR_B_PERCFG0_I2C
		     | DSCR_B_PERCFG0_GPIO
		     | DSCR_B_PERCFG0_EMAC); /* 0xc0015555 */

	/* read for enabling (reg PERSTAT0) */
	val = dscr_get_reg(DSCR_PERSTAT0);

	/* Wait 128 cycles */
	_c6x_delay(22);
#endif /* defined(CONFIG_SOC_TMS320C6474) */

#if defined(CONFIG_SOC_TMS320C6472)

	/* Do not allow user mode to access SoC device I/O */
	dscr_set_reg(DSCR_PRIVKEY,  0xbea7);
	dscr_set_reg(DSCR_PRIVPERM, 0xaaaaaaaa);
	dscr_set_reg(DSCR_PRIVKEY,  0x0);

#endif /*defined(CONFIG_SOC_TMS320C6472) */

#if defined(CONFIG_SOC_TMS320C6455)
	/* Enable timers (in regs PERLOCK & PERCFG0) */
	val = dscr_get_reg(DSCR_PERCFG0);
	dscr_set_device(val | DSCR_B_PERCFG0_TIMER0 | DSCR_B_PERCFG0_TIMER1, DSCR_PERCFG0);

	/* Wait for enabling (reg PERSTAT0) */
	val = 0;
	while (val != 0x9) {
	    val = dscr_get_reg(DSCR_PERSTAT0);
	    val = ((val & 0x3E00) >> 9);
	}
#endif /* defined(CONFIG_SOC_TMS320C6455) */

	/* Initialize SoC resources */
	iomem_resource.name = "Memory";
	request_resource(&iomem_resource, &c6x_soc_res);
}


static int __init platform_arch_init(void)
{
	int status = 0;

	/* Intialize EMAC SoC resources */
	setup_emac();

#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
	if (c6x_platram_size) {

		c6x_platram_resource.start = c6x_platram_start;
		c6x_platram_resource.end = c6x_platram_start + c6x_platram_size - 1;

		status  = platform_device_register(&c6x_platram_device);
			printk(KERN_ERR "Could not register platram device: %d\n", status);
	}
#endif
	return status;
}

arch_initcall(platform_arch_init);

