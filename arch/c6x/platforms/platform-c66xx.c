/*
 *  linux/arch/c6x/platforms/platform-c66xx.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011, 2012 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
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

/*
 * Resources present on the SoC
 */
static struct resource c6x_soc_res = {
	.name = "C66X SOC peripherals",
	.start = 0x01000000, .end = 0x02ffffff
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
	.probes    = (const char **) part_probe_types,
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

static void init_pll(void)
{
	unsigned int val;

	/* Set ENSAT */
	val = dscr_get_reg(DSCR_MAINPLLCTL1) | 0x40;
	dscr_set_reg(DSCR_MAINPLLCTL1, val);

	/* Set OUTPUT_DIVIDE and Main PLL Bypass enabled */
	pll1_set_reg(SECCTL, 0x810000 | ((PLL_OUTDIV - 1) << 19));

	udelay(10);
	
	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLENSRC);
	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLEN);

	udelay(10);

	/* Reset PLL */
	pll1_setbit_reg(PLLCTL, PLLCTL_PLLRST);
	if (pll1_get_reg(PLLCTL & 0x2) != 0) {
		val = pll1_get_reg(PLLCTL) & 0xfffffffd;
		pll1_setbit_reg(PLLCTL, val);

		/* Wait PLL stabilization time */
		udelay(150);
	}
	
	/* Set PLL multiplier * 2 in PLLM */
	pll1_set_reg(PLLM, PLLM_VAL(PLL_MUL*2));

	/* Program main PLL BWADJ field */
	val = dscr_get_reg(DSCR_MAINPLLCTL0);
	val = (((PLL_MUL + 1)/2  - 1) << 24) & 0xff000000;
	dscr_set_reg(DSCR_MAINPLLCTL0, val);

	/*
	 * This can't be right. There is no predivider here...
	 * pll1_set_reg(PLLPREDIV, PLLPREDIV_VAL(10) | PLLPREDIV_EN);
	 */
	pll1_wait_gostat();

	/*  Set divider */
	pll1_set_reg(PLLDIV2, PLLDIV_RATIO(PLL_DIV2) | PLLDIV_EN);
	pll1_set_reg(PLLDIV5, PLLDIV_RATIO(PLL_DIV5) | PLLDIV_EN);
	pll1_set_reg(PLLDIV8, PLLDIV_RATIO(PLL_DIV8) | PLLDIV_EN);
	
	/* Adjust modified related sysclk align */
	pll1_set_reg(PLLALNCTL, pll1_get_reg(PLLDCHANGE));

	pll1_setbit_reg(PLLCMD, PLLCMD_GOSTAT);

	pll1_wait_gostat();

	/* Wait for PLL to lock */
	udelay(5);

	pll1_clearbit_reg(PLLCTL, PLLCTL_PLLRST);

	udelay(40);

	/* Main PLL Bypass disabled, set OUTPUT_DIVIDE */
	pll1_set_reg(SECCTL, 0x10000 | ((PLL_OUTDIV - 1) << 19)); 
	pll1_setbit_reg(PLLCTL, PLLCTL_PLLEN);
}

static int set_psc_state(unsigned int pd, unsigned int id, unsigned int state)
{
	volatile unsigned int *mdctl;
	volatile unsigned int *mdstat;
	volatile unsigned int *pdctl;
	int timeout;

	/* Only core 0 can set PSC */
	if (get_coreid() == get_master_coreid()) {
		mdctl  = (volatile unsigned int *) (PSC_MDCTL0 + (id << 2));
		mdstat = (volatile unsigned int *) (PSC_MDSTAT0 + (id << 2));
		pdctl  = (volatile unsigned int *) (PSC_PDCTL0 + (pd << 2));
		
		if (state == PSC_SYNCRESET)
			return 0; /* not yet supported */

		/* If state is already set, do nothing */
		if ((*mdstat & 0x1f) == state)
			return 1;

		/* Wait transition and check if we got timeout error while waiting */
		timeout = 0;
		while ((*(volatile unsigned int *) (PSC_PTSTAT)) & (0x1 << pd)) {
			udelay(1);
			if (timeout++ > 150) {
				printk(KERN_DEBUG "PSC: pd %d, id %d timeout\n", pd, id);
				return -1;
			}
		}

		/* Set power domain control */
		*pdctl = (*pdctl) | 0x00000001;
            
		/* Set MDCTL NEXT to new state */
		*mdctl = ((*mdctl) & ~(0x1f)) | state;
			
		/* Start power transition by setting PTCMD GO to 1 */
		*(volatile unsigned int *) (PSC_PTCMD) =
			*(volatile unsigned int *) (PSC_PTCMD) | (0x1 << pd);

		/* Wait for PTSTAT GOSTAT to clear */
		timeout = 0;
		while ((*(volatile unsigned int *) (PSC_PTSTAT)) & (0x1 << pd)) {
			udelay(1);
			if (timeout++ > 150) {
				printk(KERN_DEBUG "PSC: pd %d, id %d timeout\n", pd, id);
				return -1;
			}
		}

		/* Verify that state changed */
		udelay(1);
		if((*mdstat & 0x1f ) != state) {
			printk(KERN_DEBUG "PSC: pd %d, id %d state did not change (%d != %d)\n",
			       pd, id, state, *mdstat);
			return -1;
		}
	}
	return 0;
}

/*
 * Enable needed SoC devices
 */
static void init_power(void)
{
#if defined(CONFIG_SOC_TMS320C6678) && (defined(CONFIG_SPI) || defined(CONFIG_MTD))
	/* EMIF16 and SPI (C6678 only) */
	set_psc_state(0, PSC_EMIF25_SPI, PSC_ENABLE);
#endif
#ifdef CONFIG_TI_KEYSTONE_NETCP
	/* NetCP, PA and SA */
        set_psc_state(2, PSC_CPGMAC, PSC_DISABLE);
        set_psc_state(2, PSC_PA,     PSC_DISABLE);
	mdelay(100);
        set_psc_state(2, PSC_PA,     PSC_ENABLE);
        set_psc_state(2, PSC_CPGMAC, PSC_ENABLE);
/*      set_psc_state(2, PSC_SA,     PSC_ENABLE); */
#endif
#ifdef CONFIG_PCI
	/* PCIe */
        set_psc_state(3, PSC_PCIE, PSC_ENABLE);
#endif
#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
	/* sRIO */
        set_psc_state(4, PSC_SRIO, PSC_ENABLE);
#endif
#if 0
	/* HyperLink */
        set_psc_state(5, PSC_HYPERLINK, PSC_ENABLE);
#endif
	/* MSMC RAM */
        set_psc_state(7, PSC_MSMCSRAM,  PSC_ENABLE);
}

#ifdef CONFIG_TI_KEYSTONE_QM
/*
 * Enable and configure QMSS
 */
#include <mach/keystone_qmss.h>
#include <linux/keystone/qmss.h>

struct qmss_platform_data qmss_data  = {

	/* Use internal link RAM according to SPRUGR9B section 4.1.1.3 */
	.link_ram_base = 0x00080000,
	.link_ram_size = 0x3FFF,

	/* Number and size of descritors */
	.desc_ram_size = DEVICE_QM_DESC_RAM_SIZE,
	.desc_num      = DEVICE_QM_NUM_DESCS,

	/* free queue */
	.free_queue    = DEVICE_QM_FREE_Q,

	/* we are master core by default */
	.slave         = 0,

	/* PDSP firmware for accumulators */
	.qm_pdsp = {
		.pdsp              = 0, /* QM PDSP 0 */
		.firmware          = DEVICE_QM_PDSP_FIRMWARE,
		.firmware_version  = 1,
	},
};

static struct platform_device qmss_dev = {
	.name           = "keystone_qmss",
        .id             = 0,
	.dev = {
		.platform_data = &qmss_data,
	},
};

static int __init setup_qmss(void)
{
	if (get_coreid() != get_master_coreid()) {
		qmss_data.slave = 1;
	}

	return platform_device_register(&qmss_dev);
}
core_initcall(setup_qmss);
#endif /* CONFIG_TI_KEYSTONE_QM */

#ifdef CONFIG_TI_KEYSTONE_NETCP
/*
 * Setup NetCP
 */
#include <mach/keystone_netcp.h>

struct netcp_platform_data netcp_data = {
	.rx_irq            = IRQ_QMH + DEVICE_QM_ETH_ACC_RX_IDX,
	.tx_irq            = IRQ_QMH + DEVICE_QM_ETH_ACC_TX_IDX,
        .pa_pdsp_num       = 6,
	.pa_pdsp           = {
		{
			.pdsp              = 0, /* PA PDSP 0 */
			.firmware          = DEVICE_PA_PDSP_FIRMWARE_012,
			.firmware_version  = 1,
		},
		{
			.pdsp              = 1, /* PA PDSP 1 */
			.firmware          = DEVICE_PA_PDSP_FIRMWARE_012,
			.firmware_version  = 1,
		},
		{
			.pdsp              = 2, /* PA PDSP 2 */
			.firmware          = DEVICE_PA_PDSP_FIRMWARE_012,
			.firmware_version  = 1,
		},
		{
			.pdsp              = 3, /* PA PDSP 3 */
			.firmware          = DEVICE_PA_PDSP_FIRMWARE_3,
			.firmware_version  = 1,
		},
		{
			.pdsp              = 4, /* PA PDSP 4 */
			.firmware          = DEVICE_PA_PDSP_FIRMWARE_45,
			.firmware_version  = 1,
		},
		{
			.pdsp              = 5, /* PA PDSP 5 */
			.firmware          = DEVICE_PA_PDSP_FIRMWARE_45,
			.firmware_version  = 1,
		},
	 },
	.sgmii_port        = 1,
	.phy_id            = 1,
};

static struct platform_device netcp_dev0 = {
	.name           = "keystone_netcp",
        .id             = 0,
	.dev = {
		.platform_data = &netcp_data,
	},
};

static int __init setup_netcp(void)
{
        return platform_device_register(&netcp_dev0);
}

core_initcall(setup_netcp);
#endif /* CONFIG_TI_KEYSTONE_NETCP */

#if defined(CONFIG_PCI)
/*
 * Configure PCIe
 */
#include <mach/pci.h>

static struct keystone_pcie_data c6x_pcie_data = {
	.msi_irq_base	= MSI_IRQ_BASE,
	.msi_irq_num	= MSI_NR_IRQS,
};

static struct resource c6x_pcie_resources[] = {
	{
		/* Register space */
		.name		= "pcie-regs",
		.start		= C6X_PCIE_REG_BASE,
		.end		= C6X_PCIE_REG_BASE + SZ_16K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		/* Non-prefetch memory */
		.name		= "pcie-nonprefetch",
		.start		= C6X_PCIE_MEM_BASE,
		.end		= C6X_PCIE_MEM_BASE + SZ_256M - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		/* IO window */
		.name		= "pcie-io",
		.start		= C6X_PCIE_IO_BASE,
		.end		= C6X_PCIE_IO_BASE + SZ_16K - 1,
		.flags		= IORESOURCE_IO,
	},
	{
		/* Inbound memory window */
	    .name		= "pcie-inbound0",
		.start		= PLAT_PHYS_OFFSET,
		.end		= PLAT_PHYS_OFFSET + SZ_512M - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		/* Legacy Interrupt */
		.name		= "legacy_int",
		.start		= IRQ_PCIEINTA,
		.end		= IRQ_PCIEINTA,
		.flags		= IORESOURCE_IRQ,
	},
#ifdef CONFIG_PCI_MSI
	{
		/* MSI Interrupt Line */
		.name		= "msi_int",
		.start		= IRQ_PCIEMSI0,
		.end		= IRQ_PCIEMSI0,
		.flags		= IORESOURCE_IRQ,
	},
#endif
};

static struct platform_device c6x_pcie_device = {
	.name		= "keystone-pcie",
	.id		= 0,
	.dev		= {
	.platform_data	= &c6x_pcie_data,
	},
	.num_resources	= ARRAY_SIZE(c6x_pcie_resources),
	.resource	= c6x_pcie_resources,
};

__init int c6x_init_pcie(void)
{
	/* Set PCIe as RC */
	dscr_set_reg(DSCR_DEVSTAT, 
		     ((dscr_get_reg(DSCR_DEVSTAT) & ~C6X_PCIE_DEVTYPE_MASK) | 
		      C6X_PCIE_DEVTYPE_RC));
	
	/*
	 * Disable caching of the PCIe outbound non-prefetch region as 
	 * ioremap_nocache() does not uncache region on C6x (due to the lack
	 * of per page cacheability attribute).
	 */
	disable_caching((u32 *) C6X_PCIE_MEM_BASE,
			(u32*) (C6X_PCIE_MEM_BASE + SZ_256M - 1));

	disable_caching((u32 *) C6X_PCIE_REG_BASE,
			(u32*) (C6X_PCIE_REG_BASE + SZ_16K - 1));

	/* Setup SerDes configuration */
	__raw_writel(C6X_PCIE_SERDES_CFG_VAL, C6X_PCIE_SERDES_CFGPLL);

	/* Wait PLL to lock */
	while ((__raw_readl(C6X_PCIE_SERDES_STS) & 0x1) != 0x1);

	platform_device_register(&c6x_pcie_device);

	return 0;
}

core_initcall(c6x_init_pcie);
#endif /* CONFIG_PCI */

void c6x_soc_setup_arch(void)
{
 	/* Initialize C66x IRQs */          	
	clear_all_irq(); /* acknowledge all pending irqs */

	/* Unlock DSCR boot config */
	dscr_set_reg(DSCR_KICK0, DSCR_KICK0_KEY);
	dscr_set_reg(DSCR_KICK1, DSCR_KICK1_KEY);

	init_pll();

	init_power();

	/* 
	 * Do not lock DSCR for multicore issue reason and for drivers that
	 * needs to configure their serdes with DSCR 
	 */

	/* Initialize SoC resources */
	iomem_resource.name = "Memory";
	request_resource(&iomem_resource, &c6x_soc_res);
}

static int __init platform_arch_init(void)
{
	int status = 0;

#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
	if (c6x_platram_size) {

		c6x_platram_resource.start = c6x_platram_start;
		c6x_platram_resource.end   = c6x_platram_start + c6x_platram_size - 1;

		status  = platform_device_register(&c6x_platram_device);
			printk(KERN_ERR "Could not register platram device: %d\n", status);
	}
#endif
	return status;
}

arch_initcall(platform_arch_init);
