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

#include <asm/cache.h>
#include <asm/io.h>
#include <asm/hardware.h>

#ifdef ARCH_HAS_MSM

static u32 __msm_memory_start = RAM_MSM_BASE;
static u32 __msm_memory_size  = RAM_MSM_SIZE;
static u32 __msm_page_heap    = 0;
static u32 __msm_page_top     = 0;

DEFINE_SPINLOCK(__msmc_mem_lock);

/*
 * Return a MSM memory chunk
 */
static inline u32 __msm_alloc(size_t size)
{
	u32 paddr;

	if ((__msm_page_heap + size) > __msm_page_top)
		return -1;
	
	paddr            = __msm_page_heap;
	__msm_page_heap += size;

	return paddr;
}

/*
 * the MSM is a one to one mapping, for the time being do it simple as we know
 * the hardcoded underneath mapping...
 */
u32 msm_virt_to_phys(u32 vaddr)
{
	return (vaddr - (RAM_MSM_CO_BASE - RAM_MSM_BASE));
}

u32 msm_phys_to_virt(u32 paddr)
{
	return (paddr + (RAM_MSM_CO_BASE - RAM_MSM_BASE));
}

u32 msm_get_heap(void)
{
	return msm_phys_to_virt(__msm_page_heap);
}

u32 msm_get_top(void)
{
	return msm_phys_to_virt(__msm_page_top);
}

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

	if (in_interrupt())
		BUG();

	/* Round up to a page */
	size = PAGE_ALIGN(size);

	spin_lock_irq(&__msmc_mem_lock);

	paddr = __msm_alloc(size);
	
	spin_unlock_irq(&__msmc_mem_lock);

	if (paddr == -1)
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
	printk("%s: paddr = 0x%x, virt = 0x%x, size = 0x%x\n",
	       __FUNCTION__, paddr, virt, size);
#endif	
	return (void *) virt;
}

void 
msm_mem_init(void)
{	
	/*
	 * If the device has KeyStone MSM, set it cacheable/prefetchable for L1
	 */
	enable_caching((unsigned int *) RAM_MSM_BASE,
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
}
#endif /* ARCH_HAS_MSM */
