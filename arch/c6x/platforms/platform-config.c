/*
 *  linux/arch/c6x/platforms/platform-config.c
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

	/* Wait until GOSTAT bit is cleared*/
	while(pll1_get_reg(PLL_PLLSTAT) & PLLSTAT_GOSTAT);
	
	pll1_set_reg(PLLDIV11, PLLDIV_VAL(10) | PLLDIV_EN);
	pll1_set_reg(PLLDIV13, PLLDIV_VAL(6)  | PLLDIV_EN);
	
	pll1_setbit_reg(PLLCMD, PLLCMD_GOSTAT);
	
	/* Wait until GOSTAT bit is cleared*/
	while(pll1_get_reg(PLLSTAT) & PLLSTAT_GOSTAT);

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
	*(int *)  PSC_PDCTL0 |= 0x00000001;
	*(int *)  PSC_MDCTL0 |= 0x00000003;
	*(int *)  PSC_MDCTL1 |= 0x00000003;
	*(int *)  PSC_MDCTL2 |= 0x00000003;
 
	*(int *)  PSC_PDCTL1 |= 0x00000001;
	*(int *)  PSC_MDCTL6 |= 0x00000003;

	*(int *)  PSC_PDCTL2 |= 0x00000001;
	*(int *)  PSC_MDCTL7 |= 0x00000003;

	*(int *)  PSC_PDCTL3 |= 0x00000001;
	*(int *)  PSC_MDCTL8 |= 0x00000003;

	*(int *)  PSC_PDCTL4 |= 0x00000001;
	*(int *)  PSC_MDCTL9 |= 0x00000003;

	*(int *)  PSC_PDCTL5 |= 0x00000001;
	*(int *)  PSC_MDCTL10 |= 0x00000003;

	*(int *)  PSC_PTCMD  |= 0x0000003f;
#endif

#if defined(CONFIG_SOC_TMS320C6472) && defined(CONFIG_TMS320C64X_GEMAC)
	*(int *)  PSC_MDCTL8 |= 0x00000103; /* GEMAC0 */
#ifdef CONFIG_TMS320C64X_GEMAC_1
	*(int *)  PSC_MDCTL9 |= 0x00000103; /* GEMAC1 */
#endif
	*(int *)  PSC_PTCMD  |= 0x00000001;
#endif
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
#endif

#if defined(CONFIG_SOC_TMS320C6472)
	/* Do not allow user mode to access SoC device I/O */
	dscr_set_reg(DSCR_PRIVKEY,  0xbea7);
	dscr_set_reg(DSCR_PRIVPERM, 0xaaaaaaaa);
	dscr_set_reg(DSCR_PRIVKEY,  0x0);
#endif

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
#endif

	/* Initialize SoC resources */
	iomem_resource.name = "Memory";
	request_resource(&iomem_resource, &c6x_soc_res);
}


static int __init platform_arch_init(void)
{
#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
	if (c6x_platram_size) {
		int status;

		c6x_platram_resource.start = c6x_platram_start;
		c6x_platram_resource.end = c6x_platram_start + c6x_platram_size - 1;

		status  = platform_device_register(&c6x_platram_device);
			printk(KERN_ERR "Could not register platram device: %d\n", status);
	}
#endif
}

arch_initcall(platform_arch_init);

