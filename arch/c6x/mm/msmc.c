/*
 *  linux/arch/c6x/mm/msmc.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/bitmap.h>

#include <asm/cache.h>
#include <asm/io.h>
#include <asm/hardware.h>

#ifdef ARCH_HAS_MSM

static u32 __msm_memory_start = RAM_MSM_BASE;
static u32 __msm_memory_size  = RAM_MSM_SIZE;
static u32 __msm_page_heap    = 0;
static u32 __msm_page_top     = 0;

#define MSM_BBMAP_BITS (RAM_MSM_SIZE >> PAGE_SHIFT)

static DECLARE_BITMAP(__msm_bitmap, MSM_BBMAP_BITS);

DEFINE_SPINLOCK(__msmc_mem_lock);

/*
 * Return a MSM memory chunk
 */
static inline u32 __msm_alloc(int order)
{
	unsigned long flags;
	u32 offset;
	u32 paddr;

	spin_lock_irqsave(&__msmc_mem_lock, flags);
	offset = bitmap_find_free_region(__msm_bitmap, MSM_BBMAP_BITS, order);

	paddr = __msm_memory_start + (offset << PAGE_SHIFT);

	if (__msm_page_heap < paddr)
		__msm_page_heap = paddr;
	
	spin_unlock_irqrestore(&__msmc_mem_lock, flags);

	return paddr;
}

static inline void __msm_free(u32 paddr, int order)
{
	unsigned long flags;
	u32 offset = (paddr - __msm_memory_start) >> PAGE_SHIFT;

	if ((paddr < __msm_memory_start) || (offset + (1 << order) >= MSM_BBMAP_BITS)) {
		printk(KERN_ERR "%s: freeing outside MSM range\n", __func__);
		BUG();
	}
		
	spin_lock_irqsave(&__msmc_mem_lock, flags);
	bitmap_release_region(__msm_bitmap, offset, order);
	spin_unlock_irqrestore(&__msmc_mem_lock, flags);
}

/*
 * the MSM is a one to one mapping, for the time being do it simple as we know
 * the hardcoded underneath mapping...
 */
u32 msm_virt_to_phys(u32 vaddr)
{
	return (vaddr - (RAM_MSM_CO_BASE - RAM_MSM_BASE));
}
EXPORT_SYMBOL(msm_virt_to_phys);

u32 msm_phys_to_virt(u32 paddr)
{
	return (paddr + (RAM_MSM_CO_BASE - RAM_MSM_BASE));
}
EXPORT_SYMBOL(msm_phys_to_virt);

u32 msm_get_heap(void) 
{
	return msm_phys_to_virt(__msm_page_heap);
}
EXPORT_SYMBOL(msm_get_heap);

u32 msm_get_top(void)
{
	return msm_phys_to_virt(__msm_page_top);
}
EXPORT_SYMBOL(msm_get_top);

/*
 * Allocate an MSM memory chunk and returns its virtual address and the physical
 * address for DMA devices. 
 * Note that the virtual address points to a coherent region (not-cached)
 */
void *
msm_alloc_coherent(size_t size, dma_addr_t *handle)
{
	u32 paddr;
	u32 virt;
	int order;

	if (in_interrupt())
		BUG();

	if (!__msm_memory_size || !size)
		return NULL;

	order = get_order(size);
	paddr = __msm_alloc(order);
	
	if (!paddr)
		return NULL;

	if (handle)
		*handle = __phys_to_bus(paddr);

	virt = msm_phys_to_virt(paddr);
	if (!virt)
		return NULL;

	/*
	 * We need to ensure that there are no cachelines in use, or
	 * worse dirty in this area.
	 */
	L2_cache_block_invalidate(paddr, paddr + size);

#ifdef DEBUG
	printk(KERN_CRIT "%s: paddr = 0x%x, virt = 0x%x, size = 0x%x\n",
	       __func__, paddr, virt, size);
#endif	
	return (void *) virt;
}
EXPORT_SYMBOL(msm_alloc_coherent);

void
msm_free_coherent(size_t size, void *vaddr, dma_addr_t *handle)
{
	int order;

	if (in_interrupt())
		BUG();

	if (!__msm_memory_size || !size)
		return;

	order = get_order(size);
	__msm_free(msm_virt_to_phys((u32) vaddr), order);

#ifdef DEBUG
	printk(KERN_CRIT "%s: vaddr = 0x%x, size = 0x%x\n",
	       __func__, vaddr, size);
#endif	
}
EXPORT_SYMBOL(msm_free_coherent);

void 
msm_mem_init(void)
{	
	/*
	 * If the device has KeyStone MSM, set it cacheable but non-prefetchable for L1!
	 * (data corruption issue)
	 */
	enable_caching((unsigned int *) RAM_MSM_BASE,
		       (unsigned int *) (RAM_MSM_BASE + IMCR_MAR_SIZE - 1));
	disable_prefetching((unsigned int *) RAM_MSM_BASE,
			    (unsigned int *) (RAM_MSM_BASE + IMCR_MAR_SIZE - 1));
	
#ifdef ARCH_HAS_XMC_MPAX
	/*
	 * Map MSM in a non cacheable/prefetchable region
	 */
	xmc_map_region(3,
		       RAM_MSM_CO_BASE,
		       RAM_MSM_BASE,
		       XMC_SEG_SIZE_4MB,
		       XMC_PERM_SX | XMC_PERM_SW | XMC_PERM_SR |
		       XMC_PERM_UX | XMC_PERM_UW | XMC_PERM_UR);
	
	disable_caching((unsigned int *) RAM_MSM_CO_BASE,
			(unsigned int *) (RAM_MSM_CO_BASE + IMCR_MAR_SIZE - 1));
#endif /* ARCH_HAS_XMC_MPAX */

	__msm_page_heap = __msm_memory_start;
	__msm_page_top  = __msm_memory_start + __msm_memory_size;

	bitmap_zero(__msm_bitmap, MSM_BBMAP_BITS);
}
#endif /* ARCH_HAS_MSM */
