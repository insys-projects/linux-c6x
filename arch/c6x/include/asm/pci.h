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

/*
 * Mapped PCI addresses (memory, IO, config and PCI host)
 */
#define PCI_MEMORY_VADDR        0xe8000000
#define PCI_CONFIG_VADDR        0xec000000
#define PCI_HOST_VADDR          0xed000000
#define PCI_IO_VADDR            0xee000000

#define PCIO_BASE		PCI_IO_VADDR
#define PCIMEM_BASE		PCI_MEMORY_VADDR

#define PCIBIOS_MIN_IO		0x6000
#define PCIBIOS_MIN_MEM 	0x00100000

#define pcibios_assign_all_busses()	        (1)
#define pci_dac_dma_supported(pci_dev, mask)	(0)
#define PCI_DMA_BUS_IS_PHYS     (0)

extern inline void pcibios_set_master(struct pci_dev *dev)
{
	/* No special bus mastering setup handling */
}

extern inline void pcibios_penalize_isa_irq(int irq)
{
	/* We don't do dynamic PCI IRQ allocation */
}

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_PCI_H_ */
