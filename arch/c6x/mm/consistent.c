/*
 *  linux/include/arch/c6x/mm/consistent.c
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
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/scatterlist.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/cache.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

/*
 * This allocates one page of cache-coherent memory space and returns
 * both the virtual and a "dma" address to that space.  It is not clear
 * whether this could be called from an interrupt context or not.  For
 * now, we expressly forbid it, especially as some of the stuff we do
 * here is not interrupt context safe.
 *
 * Note that this does *not* zero the allocated area!
 */
void *consistent_alloc(int gfp, size_t size, dma_addr_t *dma_handle)
{
#ifndef CONFIG_CONTIGUOUS_PAGE_ALLOC
	struct page *page, *end, *free;
	unsigned long order;
#endif
	void *ret, *virt;

	if (in_interrupt())
		BUG();

#ifdef CONFIG_CONTIGUOUS_PAGE_ALLOC
	virt = kmalloc(size < PAGE_SIZE ? PAGE_SIZE : size, gfp);
	if (!virt)
		goto no_page;
#else
	size = PAGE_ALIGN(size);
	order = get_order(size);

	page = alloc_pages(gfp, order);
	if (!page)
		goto no_page;

	/*
	 * We could do with a page_to_phys and page_to_bus here.
	 */
	virt = page_address(page);
#endif
	*dma_handle = virt_to_bus(virt);
	ret = __ioremap(virt_to_phys(virt), size, 0);
	if (!ret)
		goto no_remap;

	/*
	 * we need to ensure that there are no cachelines in use, or
	 * worse dirty in this area.
	 */
	L2_cache_block_invalidate(virt, virt + size);

#ifndef CONFIG_CONTIGUOUS_PAGE_ALLOC
	/*
	 * free wasted pages.  We skip the first page since we know
	 * that it will have count = 1 and won't require freeing.
	 * We also mark the pages in use as reserved so that
	 * remap_page_range works.
	 */
	page = virt_to_page(virt);
	free = page + (size >> PAGE_SHIFT);
	end  = page + (1 << order);

	for (; page < end; page++) {
		init_page_count(page);
		if (page >= free)
			__free_page(page);
		else
			SetPageReserved(page);
	}
#endif
	return ret;

no_remap:
#ifdef CONFIG_CONTIGUOUS_PAGE_ALLOC
	kfree(virt);
#else
	__free_pages(page, order);
#endif
no_page:
	return NULL;
}

void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size, dma_addr_t *handle)
{
	void *__ret;
	int __gfp = GFP_KERNEL;

#ifdef CONFIG_PCI
	if ((hwdev) == NULL ||
	    (hwdev)->dma_mask != 0xffffffff)
#endif
		__gfp |= GFP_DMA;

	__ret = consistent_alloc(__gfp, (size),
				 (handle));
	return __ret;
}

/*
 * free a page as defined by the above mapping.  We expressly forbid
 * calling this from interrupt context.
 */
void consistent_free(void *vaddr, size_t size, dma_addr_t handle)
{
#ifndef CONFIG_CONTIGUOUS_PAGE_ALLOC
	struct page *page, *end;
#endif
	void *virt;

	if (in_interrupt())
		BUG();

	virt = bus_to_virt(handle);

#ifdef CONFIG_CONTIGUOUS_PAGE_ALLOC
	kfree(virt);
#else
	/*
	 * More messing around with the MM internals.  This is
	 * sick, but then so is remap_page_range().
	 */
	size = PAGE_ALIGN(size);
	page = virt_to_page(virt);
	end = page + (size >> PAGE_SHIFT);

	for (; page < end; page++) {
		ClearPageReserved(page);
		__free_page(page);
	}
#endif
	iounmap(vaddr);
}

/*
 * make an area consistent.
 */
void consistent_sync(void *vaddr, size_t size, int direction)
{
	unsigned long start = (unsigned long)vaddr;
	unsigned long end   = start + size;

	switch (direction) {
	case PCI_DMA_NONE:
		BUG();
	case PCI_DMA_FROMDEVICE:	/* invalidate only */
		L2_cache_block_invalidate(start, end);
		break;
	case PCI_DMA_TODEVICE:		/* writeback only */
		L2_cache_block_writeback(start, end);
		break;
	case PCI_DMA_BIDIRECTIONAL:	/* writeback and invalidate */
		L2_cache_block_writeback_invalidate(start, end);
		break;
	}
}
