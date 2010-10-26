/*
 *  linux/include/asm-c6x/virtconvert.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
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

/*
 * Change virtual addresses to physical addresses and vv.
 */
#define virt_to_phys(virt) ((unsigned long) (virt))
#define phys_to_virt(phys) ((void *)(phys))

#define virt_to_bus virt_to_phys
#define bus_to_virt phys_to_virt

#define __page_address(page) (PAGE_OFFSET + (((page) - mem_map) << PAGE_SHIFT))
#define page_to_phys(page)   virt_to_phys((void *)__page_address(page))

#endif /*__KERNEL__ */
#endif /* __ASM_C6X_VIRTCONVERT_H */
