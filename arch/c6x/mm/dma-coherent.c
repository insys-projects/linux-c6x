/*
 *  linux/arch/c6x/mm/dma-coherent.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <aurelien.jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  DMA uncached mapping support.
 *
 *  Using code pulled from ARM 
 *  Copyright (C) 2000-2004 Russell King
 *
 */
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/wait.h>

#include <asm-generic/dma-coherent.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/cache.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

/*
 * DMA coherent memory management, can be redefined using the memdma= kernel command line
 */
unsigned long dma_memory_start = 0; /* by default at the end of the Linux physical memory */
unsigned long dma_memory_size  = 0; /* none by default */

static u32    dma_page_heap;
static u32    dma_page_top;

DEFINE_SPINLOCK(dma_mem_lock);

/*
 * Allocate DMA-coherent memory space and return both the kernel remapped
 * virtual and bus address for that space.
 *
 * Note that this does *not* zero the allocated area!
 */
void *
dma_alloc_coherent(struct device *dev, size_t size, dma_addr_t *handle, gfp_t gfp)
{
	u32 paddr;
	u32 virt;

	/* Round up to a page */
	size = PAGE_ALIGN(size);

	spin_lock_irq(&dma_mem_lock);
	
	if ((dma_page_heap + size) > dma_page_top) {
		spin_unlock_irq(&dma_mem_lock);
		goto no_page;
	}
	
	paddr          = dma_page_heap;
	dma_page_heap += size;
	
	spin_unlock_irq(&dma_mem_lock);

	if (handle)
		*handle = paddr;

	/*
	 * In a near future we can expect having a partial MMU with 
	 * chaching attributes 
	 */
	virt = (u32) ioremap_nocache(paddr, size);
	if (!virt)
		goto no_remap;

	/*
	 * We need to ensure that there are no cachelines in use, or
	 * worse dirty in this area.
	 */
	L2_cache_block_invalidate(paddr, paddr + size);

	return (void *) virt;

no_remap:

no_page:
	return NULL;
}
EXPORT_SYMBOL(dma_alloc_coherent);

/*
 * Free a page as defined by the above mapping.
 * Must not be called with IRQs disabled.
 */
void dma_free_coherent(struct device *dev, size_t size, void *cpu_addr, dma_addr_t handle)
{
	void *virt;

	if (in_interrupt())
		BUG();
	
	/* Do nothing */
	virt = bus_to_virt(handle);
	
	iounmap(cpu_addr);
}
EXPORT_SYMBOL(dma_free_coherent);

/*
 * Make an area consistent for devices.
 * Note: Drivers should NOT use this function directly, as it will break
 * platforms with CONFIG_DMABOUNCE.
 * Use the driver DMA support - see dma-mapping.h (dma_sync_*)
 */
void __dma_single_cpu_to_dev(const void *kaddr, size_t size,
			     enum dma_data_direction dir)
{
	unsigned long paddr;
	
	BUG_ON(!virt_addr_valid(kaddr) || !virt_addr_valid(kaddr + size - 1));
	
	paddr = __pa(kaddr);
	switch(dir) {
	case DMA_FROM_DEVICE:
		L2_cache_block_invalidate(paddr, paddr + size);
		break;
	case DMA_TO_DEVICE:
		L2_cache_block_writeback(paddr, paddr + size);
		break;
	case DMA_BIDIRECTIONAL:
		L2_cache_block_writeback_invalidate(paddr, paddr + size);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(__dma_single_cpu_to_dev);

void __dma_single_dev_to_cpu(const void *kaddr, size_t size,
			     enum dma_data_direction dir)
{
	BUG_ON(!virt_addr_valid(kaddr) || !virt_addr_valid(kaddr + size - 1));

	/* don't bother invalidating if DMA to device */
	if (dir != DMA_TO_DEVICE) {
		unsigned long paddr = __pa(kaddr);
		L2_cache_block_invalidate(paddr, paddr + size);
	}
}
EXPORT_SYMBOL(__dma_single_dev_to_cpu);

void __dma_page_cpu_to_dev(struct page *page, unsigned long off,
			   size_t size, enum dma_data_direction dir)
{
	unsigned long paddr;

	paddr = page_to_phys(page) + off;
	switch(dir) {
	case DMA_FROM_DEVICE:
		L2_cache_block_invalidate(paddr, paddr + size);
		break;
	case DMA_TO_DEVICE:
		L2_cache_block_writeback(paddr, paddr + size);
		break;
	case DMA_BIDIRECTIONAL:
		L2_cache_block_writeback_invalidate(paddr, paddr + size);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(__dma_page_cpu_to_dev);

void __dma_page_dev_to_cpu(struct page *page, unsigned long off,
			   size_t size, enum dma_data_direction dir)
{
	unsigned long paddr = page_to_phys(page) + off;

	/* don't bother invalidating if DMA to device */
	if (dir != DMA_TO_DEVICE)
		L2_cache_block_invalidate(paddr, paddr + size);
}
EXPORT_SYMBOL(__dma_page_dev_to_cpu);

/*
 * Initialise the coherent memory and its allocator
 */
int coherent_mem_init(void)
{
	/*
	 * Define the (DMA) coherent memory
	 */
	if (dma_memory_size != 0) {

		/* Round it to the (upper) MAR granularity  */
		dma_memory_size = CACHE_REGION_END(dma_memory_size);

		if (!dma_memory_start) {
			/*
			 * Take the coherent memory from the end of the physical
			 * memory and round it to the lower MAR.
			 * We may waste some cacheable memory if memory_end is not
			 * aligned on a MAR region.
			 */
			dma_memory_start =
				CACHE_REGION_START(memory_end - dma_memory_size);
			
			/* Then remove the coherent memory from the paged one */
			memory_end = dma_memory_start;
			

		} else {
			/* Align it on MAR */
			dma_memory_start = CACHE_REGION_START(dma_memory_start);

			/*
			 * Check if the defined coherent memory is within the paged
			 * memory. If so remove the corresponding memory
			 */
			if ((dma_memory_start < memory_end) && (dma_memory_start > memory_start))
				memory_end = dma_memory_start;
		}

		printk(KERN_INFO "Coherent memory (DMA) region start=0x%lx size=0x%lx\n",
		       dma_memory_start,
		       dma_memory_size);
		
		/*
		 * We need to ensure that there are no cachelines in use, or
		 * worse dirty in this area.
		 */
		L2_cache_block_writeback(dma_memory_start,
					 dma_memory_start + dma_memory_size - 1);

		/* Make this memory coherent (so non-cacheable) */
		disable_caching((unsigned int *) dma_memory_start,
				(unsigned int *) (dma_memory_start + dma_memory_size - 1));

		printk(KERN_INFO "disabling caching for 0x%lx to 0x%lx\n",
		       dma_memory_start,
		       dma_memory_start + dma_memory_size - 1);

		/* The allocator starts here */
		dma_page_heap = dma_memory_start;

		/* And finish here */
		dma_page_top = PAGE_ALIGN(dma_memory_start + dma_memory_size);
	}

	return 0;
}

