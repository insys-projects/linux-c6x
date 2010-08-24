/*
 *  linux/arch/c6x/kernel/bios32.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Bios 32 replacement
 */
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/pci.h>
#include <asm/uaccess.h>

struct pci_fixup pcibios_fixups[] =
{
	{ 0 }
};

/*
 * pcibios_fixup_bus - Called after each bus is probed,
 * but before its children are examined.
 */
void __init pcibios_fixup_bus(struct pci_bus *bus)
{
}

void __init pcibios_init(void)
{
	printk("PCI: No PCI bus detected\n");
}

char * __init pcibios_setup(char *str)
{
	return str;
}

void __init
pcibios_update_resource(struct pci_dev *dev, struct resource *root,
			struct resource *res, int resource)
{
}

/*
 * From arch/i386/kernel/pci-i386.c:
 *
 * We need to avoid collisions with `mirrored' VGA ports
 * and other strange ISA hardware, so we always want the
 * addresses to be allocated in the 0x000-0x0ff region
 * modulo 0x400.
 *
 * Why? Because some silly external IO cards only decode
 * the low 10 bits of the IO address. The 0x00-0xff region
 * is reserved for motherboard devices that decode all 16
 * bits, so it's ok to allocate at, say, 0x2800-0x28ff,
 * but we want to try to avoid allocating at 0x2900-0x2bff
 * which might be mirrored at 0x0100-0x03ff..
 */
void pcibios_align_resource(void *data,
			    struct resource *res,
			    unsigned long size,
			    unsigned long align)
{
}

/**
 * pcibios_enable_device - Enable I/O and memory.
 * @dev: PCI device to be enabled
 */
int pcibios_enable_device(struct pci_dev *dev, int mask)
{
	return 0;
}
