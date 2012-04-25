/*
 *  linux/include/asm-c6x/pci.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <aurelien.jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_PCI_H_
#define __ASM_C6X_PCI_H_

#ifdef __KERNEL__
#include <asm-generic/dma-coherent.h>
#include <asm-generic/pci-dma-compat.h>
#include <mach/pci.h>

/*
 * Mapped PCI addresses (memory, IO, config and PCI host)
 */
#define PCI_MEMORY_VADDR        0xe8000000
#define PCI_CONFIG_VADDR        0xec000000
#define PCI_HOST_VADDR          0xed000000
#define PCI_IO_VADDR            0xee000000

#define PCIO_BASE		PCI_IO_VADDR
#define PCIMEM_BASE		PCI_MEMORY_VADDR

/*
 * PCI Resource allocation.
 *
 * IMPORTANT: Ensure that the values used below match with the ones passed
 * though PCIe RC platform data (from SoC/board file).
 *
*/
#define PCIBIOS_MIN_IO		C6X_PCIE_IO_BASE
#define PCIBIOS_MIN_MEM 	C6X_PCIE_MEM_BASE

/* Enumeration flag: Checked during PCI Enumeration of Bridges */
#define pcibios_assign_all_busses()	        (1)

extern inline void pcibios_set_master(struct pci_dev *dev)
{
	/* No special bus mastering setup handling */
}

extern inline void pcibios_penalize_isa_irq(int irq, int active)
{
	/* We don't do dynamic PCI IRQ allocation */
}

/*
 * The PCI address space does equal the physical memory address space.
 * The networking and block device layers use this boolean for bounce
 * buffer decisions.
 */
#define PCI_DMA_BUS_IS_PHYS     (1)

extern void pcibios_resource_to_bus(struct pci_dev *, struct pci_bus_region *,
				    struct resource *);

extern void pcibios_bus_to_resource(struct pci_dev *dev, struct resource *res,
				    struct pci_bus_region *region);

/****************************************************** 
   +++TODO:  ref arch/arm/include/asm/mach/pci.h 
*******************************************************/

struct pci_sys_data;
struct pci_bus;

struct hw_pci {
#ifdef CONFIG_PCI_DOMAINS
	int		domain;
#endif
	struct list_head	buses;
	int					nr_controllers;
	int					(*setup)(int nr, struct pci_sys_data *);
	struct pci_bus		*(*scan)(int nr, struct pci_sys_data *);
	void				(*preinit)(void);
	void				(*postinit)(void);
	u8					(*swizzle)(struct pci_dev *dev, u8 *pin);
	int					(*map_irq)(struct pci_dev *dev, u8 slot, u8 pin);
};

/*
 * Per-controller structure
 */
struct pci_sys_data {
#ifdef CONFIG_PCI_DOMAINS
	int				domain;
#endif
	struct list_head	node;
	int					busnr;			/* primary bus number			*/
	u64					mem_offset;		/* bus->cpu memory mapping offset	*/
	unsigned long		io_offset;		/* bus->cpu IO mapping offset		*/
	struct pci_bus		*bus;			/* PCI bus				*/
	struct resource		*resource[3];	/* Primary PCI bus resources		*/
					/* Bridge swizzling			*/
	u8		(*swizzle)(struct pci_dev *, u8 *);
					/* IRQ mapping				*/
	int		(*map_irq)(struct pci_dev *, u8, u8);
	struct hw_pci		*hw;
	void				*private_data;	/* platform controller private data	*/
};

/*
 * This is the standard PCI-PCI bridge swizzling algorithm.
 */
#define pci_std_swizzle pci_common_swizzle

/*
 * Call this with your hw_pci struct to initialise the PCI system.
 */
void pci_common_init(struct hw_pci *);

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_PCI_H_ */
