/*
 *  linux/arch/c6x/mm/memory.c
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
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/traps.h>
#include <asm/virtconvert.h>

/*
 * The following two routines map from a physical address to a kernel
 * virtual address and vice versa.
 */
unsigned long mm_vtop (unsigned long vaddr)
{
	return vaddr;
}

unsigned long mm_ptov (unsigned long paddr)
{
	return paddr;
}

/*
 * cache_clear() semantics: Clear any cache entries for the area in question,
 * without writing back dirty entries first. This is useful if the data will
 * be overwritten anyway, e.g. by DMA to memory. The range is defined by a
 * _physical_ address.
 */
void cache_clear (unsigned long paddr, int len)
{
#ifdef CONFIG_TMS320C6X_CACHES_ON
	L2_cache_block_invalidate((u32) paddr, (u32) paddr + len);
#endif
}


/*
 * cache_push() semantics: Write back any dirty cache data in the given area,
 * and invalidate the range in the instruction cache. It needs not (but may)
 * invalidate those entries also in the data cache. The range is defined by a
 * _physical_ address.
 */
void cache_push (unsigned long paddr, int len)
{
#ifdef CONFIG_TMS320C6X_CACHES_ON
	L2_cache_block_writeback_invalidate((u32) paddr, (u32) paddr + len);
#endif
}


/*
 * cache_push_v() semantics: Write back any dirty cache data in the given
 * area, and invalidate those entries at least in the instruction cache. This
 * is intended to be used after data has been written that can be executed as
 * code later. The range is defined by a _user_mode_ _virtual_ address  (or,
 * more exactly, the space is defined by the %sfc/%dfc register.)
 */
void cache_push_v (unsigned long vaddr, int len)
{
#ifdef CONFIG_TMS320C6X_CACHES_ON
	L2_cache_block_writeback_invalidate((u32) vaddr, (u32) vaddr + len);
#endif
}

unsigned long mm_phys_to_virt (unsigned long addr)
{
    return mm_ptov(addr);
}

/* Map some physical address range into the kernel address space. The
 * code is copied and adapted from map_chunk().
 */
unsigned long kernel_map(unsigned long paddr, unsigned long size,
			 int nocacheflag, unsigned long *memavailp )
{
	return paddr;
}

void kernel_set_cachemode( unsigned long address, unsigned long size,
						   unsigned cmode )
{
}

#ifdef MAGIC_ROM_PTR
int is_in_rom(unsigned long addr) {
	return 0;
}
#endif
