/*
 *  linux/include/asm-c6x/pgalloc.h
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
#ifndef __ASM_C6X_CACHEFLUSH_H
#define __ASM_C6X_CACHEFLUSH_H

#include <linux/spinlock.h>

#include <asm/setup.h>
#include <asm/virtconvert.h>
#include <asm/cache.h>
#include <asm/mman.h>
#include <asm/page.h>
#include <asm/string.h>

/*
 * Caches handling functions
 */
#ifndef CONFIG_TMS320C64XPLUS

#define flush_cache_all() L2_cache_global_writeback_invalidate()

#define flush_cache_mm(mm)                               \
        do {                                             \
        if ((mm) == current->active_mm)                  \
		 L2_cache_global_writeback_invalidate(); \
        } while(0)

#define flush_cache_range(mm, start, end)                            \
        do {                                                         \
        if ((mm) == current->mm)                                     \
                L2_cache_block_writeback_invalidate((start), (end)); \
        } while(0)

#define flush_cache_page(vma, vmaddr)                                          \
        L2_cache_block_writeback_invalidate((vma),                             \
                                            (unsigned long) (vma) + PAGE_SIZE)

#define flush_page_to_ram(page)                                                  \
        L2_cache_block_writeback_invalidate(page_address(page),                  \
                                 (unsigned long) page_address(page) + PAGE_SIZE)

#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 1
#define flush_dcache_page(page)                                                  \
        L1D_cache_block_writeback_invalidate(page_address(page),                 \
                                 (unsigned long) page_address(page) + PAGE_SIZE)

#define flush_icache_range(s,e)                                   \
        do {                                                      \
                L1D_cache_block_writeback_invalidate((s), (e));   \
                L1P_cache_block_invalidate((s), (e));             \
        } while (0)

#else /* CONFIG_TMS320C64XPLUS */

#define flush_cache_all()                                \
        do {                                             \
	     L1P_cache_global_invalidate();              \
             L2_cache_global_writeback_invalidate();     \
	} while(0)

#define flush_cache_mm(mm)                               \
        do {                                             \
        if ((mm) == current->active_mm)                  \
                 L1P_cache_global_invalidate();          \
		 L2_cache_global_writeback_invalidate(); \
        } while(0)

#define flush_cache_range(mm, start, end)                            \
        do {                                                         \
        if ((mm) == current->mm)                                     \
                L1P_cache_block_invalidate((start), (end));          \
                L2_cache_block_writeback_invalidate((start), (end)); \
        } while(0)

#define flush_cache_page(vma, vmaddr)                                           \
        do {                                                                    \
                L1P_cache_block_invalidate((vma),                               \
                                            (unsigned long) (vma) + PAGE_SIZE); \
                L2_cache_block_writeback_invalidate((vma),                      \
                                            (unsigned long) (vma) + PAGE_SIZE); \
        } while(0)

#define flush_page_to_ram(page)                                                   \
        do {                                                                      \
                L1P_cache_block_invalidate((page_address(page),                   \
                                 (unsigned long) page_address(page) + PAGE_SIZE); \
                L2_cache_block_writeback_invalidate(page_address(page),           \
                                 (unsigned long) page_address(page) + PAGE_SIZE); \
       } while(0)

#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 1
#define flush_dcache_page(page)                                                  \
        L1D_cache_block_writeback_invalidate(page_address(page),                 \
                                 (unsigned long) page_address(page) + PAGE_SIZE)

#define flush_icache_range(s,e)                                   \
        do {                                                      \
                L1D_cache_block_writeback((s), (e));              \
                L1P_cache_block_invalidate((s), (e));             \
        } while (0)

#endif /* CONFIG_TMS320C64XPLUS */

#define flush_icache_user_range(vma,pg,adr,len) \
        L2_cache_global_writeback_invalidate()

#define flush_icache_page(vma, page)	                                  \
        do {                                                              \
        if ((vma)->vm_flags & PROT_EXEC)                                  \
                L1D_cache_block_writeback_invalidate(page_address(page),  \
                        (unsigned long) page_address(page) + PAGE_SIZE)); \
                L1P_cache_block_invalidate(page_address(page),            \
                        (unsigned long) page_address(page) + PAGE_SIZE)); \
        } while (0)


#define flush_icache() \
        L1P_cache_global_invalidate()

#define flush_dcache_mmap_lock(mapping) \
	write_lock_irq(&(mapping)->tree_lock)
#define flush_dcache_mmap_unlock(mapping) \
	write_unlock_irq(&(mapping)->tree_lock)

#define copy_to_user_page(vma, page, vaddr, dst, src, len) \
	memcpy(dst, src, len)
#define copy_from_user_page(vma, page, vaddr, dst, src, len) \
	memcpy(dst, src, len)

#endif /* __ASM_C6X_CACHEFLUSH_H */
