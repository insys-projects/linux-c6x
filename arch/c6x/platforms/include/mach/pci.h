/*
 * Platform data for C6x PCIe Root Complex module.
 *
 * Copyright (C) 2010, 2012 Texas Instruments, Inc. - http://www.ti.com/
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

#ifndef __MACH_PCI_H
#define __MACH_PCI_H

struct keystone_pcie_data {
	int msi_irq_base;
	int msi_irq_num;
	int force_x1;
};

/*
 * This is the definitions for the PCIe to RC memory mapping.
 * The current PCIe RC driver is assuming a 1:1 mapping between the RC address
 * space and the PCI address space.
 */
#define C6X_PCIE_REG_BASE	0x21800000   /* PCIe config: this cannot be changed 
						and this is defined per device family.
					        This is same for all KeyStone (I) SoC */
#define C6X_PCIE_MEM_BASE	0x60000000   /* The PCIe data non-prefetched memory */
#define C6X_PCIE_IO_BASE	0x70000000   /* The PCIe I/O does not use real memory,
						so can be mapped everywhere in the 
						address space, we use here a 64MB free
						segment*/
#define PLAT_PHYS_OFFSET        RAM_DDR2_CE0 /* This this the start of the physical 
						DDR memory */

/* PCIe SerDes registers */
#define C6X_PCIE_SERDES_CFGPLL  0x02620358
#define C6X_PCIE_SERDES_STS     0x0262015c

/* Default value for SerDes configuration (PLL = 25x, DIVCLK output) */
#define C6X_PCIE_SERDES_CFG_VAL (BIT(0) | BIT(8) | (0x64 << 1))

/* C6x CONTROL_PCIE_CFG bits */
#define C6X_PCIE_DEVTYPE_SHIFT	14
#define C6X_PCIE_DEVTYPE_MASK	(0x3 << C6X_PCIE_DEVTYPE_SHIFT)
#define C6X_PCIE_DEVTYPE_RC	(0x2 << C6X_PCIE_DEVTYPE_SHIFT)

/* MSI IRQs may get added for TI81XX */
#define MSI_IRQ_BASE		IRQ_PCIEMSI0
#ifdef CONFIG_PCI_MSI
#define MSI_NR_IRQS		32
#else
#define MSI_NR_IRQS		0
#endif
#define MSI_IRQ_END		(MSI_IRQ_BASE + MSI_NR_IRQS)

/* handy sizes */
#define SZ_16			0x00000010
#define SZ_32			0x00000020
#define SZ_64			0x00000040
#define SZ_128			0x00000080
#define SZ_256			0x00000100
#define SZ_512			0x00000200

#define SZ_1K			0x00000400
#define SZ_2K			0x00000800
#define SZ_4K			0x00001000
#define SZ_8K			0x00002000
#define SZ_16K			0x00004000
#define SZ_32K			0x00008000
#define SZ_64K			0x00010000
#define SZ_128K			0x00020000
#define SZ_256K			0x00040000
#define SZ_512K			0x00080000

#define SZ_1M			0x00100000
#define SZ_2M			0x00200000
#define SZ_4M			0x00400000
#define SZ_8M			0x00800000
#define SZ_16M			0x01000000
#define SZ_32M			0x02000000
#define SZ_48M			0x03000000
#define SZ_64M			0x04000000
#define SZ_128M			0x08000000
#define SZ_256M			0x10000000
#define SZ_512M			0x20000000

#define SZ_1G			0x40000000
#define SZ_2G			0x80000000

#endif
