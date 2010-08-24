/*
 *  linux/include/asm-c6x/virtconvert.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_VIRTCONVERT_H
#define __ASM_C6X_VIRTCONVERT_H

#ifdef __KERNEL__

/*
 * Macros used for converting between virtual and physical mappings.
 */
#include <asm/setup.h>

#ifndef __ASSEMBLY__
/*
 * Change virtual addresses to physical addresses and vv.
 */
extern unsigned long mm_vtop(unsigned long addr);
extern unsigned long mm_ptov(unsigned long addr);

static inline unsigned long virt_to_phys(volatile void * address)
{
	return (unsigned long) mm_vtop((unsigned long)address);
}

static inline void * phys_to_virt(unsigned long address)
{
	return (void *) mm_ptov(address);
}
#endif /*__ASSEMBLER__*/

#define virt_to_bus virt_to_phys
#define bus_to_virt phys_to_virt

#define __page_address(page) (PAGE_OFFSET + (((page) - mem_map) << PAGE_SHIFT))
#define page_to_phys(page)   virt_to_phys((void *)__page_address(page))

#endif /*__KERNEL__ */
#endif /* __ASM_C6X_VIRTCONVERT_H */
