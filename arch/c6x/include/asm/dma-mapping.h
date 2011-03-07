/*
 *  linux/arch/c6x/include/asm/dma-mapping.h
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
 */
#ifndef _C6X_DMA_MAPPING_H
#define _C6X_DMA_MAPPING_H

#ifdef __KERNEL__

#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/scatterlist.h>

#include <asm/io.h>
#include <asm/types.h>
#include <asm-generic/dma-coherent.h>

#define dma_supported(d, m)         (1)

#define __pfn_to_bus(pfn)          ((pfn) << PAGE_SHIFT)
#define __bus_to_pfn(paddr)        ((paddr) >> PAGE_SHIFT)
#define __bus_to_phys(x)           (x)
#define __phys_to_bus(x)           (x)
#define __bus_to_virt(b)           phys_to_virt(__bus_to_phys(b))
#define __virt_to_bus(v)           __phys_to_bus(virt_to_phys(v))

/*
 * page_to_dma/dma_to_virt/virt_to_dma are architecture private functions
 * used internally by the DMA-mapping API to provide DMA addresses. They
 * must not be used by drivers.
 */
static inline dma_addr_t page_to_dma(struct device *dev, struct page *page)
{
	return (dma_addr_t)__pfn_to_bus(page_to_pfn(page));
}

static inline struct page *dma_to_page(struct device *dev, dma_addr_t addr)
{
	return pfn_to_page(__bus_to_pfn(addr));
}

static inline void *dma_to_virt(struct device *dev, dma_addr_t addr)
{
	return (void *)__bus_to_virt(addr);
}

static inline dma_addr_t virt_to_dma(struct device *dev, void *addr)
{
	return (dma_addr_t)__virt_to_bus((unsigned long)(addr));
}

static inline int dma_map_sg(struct device *dev, struct scatterlist *sglist, int nents,
			     enum dma_data_direction direction)
{
	struct scatterlist *sg;
        int i;

        BUG_ON(direction == DMA_NONE);

	for_each_sg(sglist, sg, nents, i) {
		BUG_ON(!sg_page(sg));

		sg->dma_address = sg_phys(sg);
	}

        return nents;
}

static inline void dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nhwentries,
				enum dma_data_direction direction)
{
        BUG_ON(direction == DMA_NONE);
}

static inline int dma_get_cache_alignment(void)
{
	return L2_CACHE_BYTES;
}

/*
 * DMA errors are defined by all-bits-set in the DMA address.
 */
static inline int dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return dma_addr == ~0;
}

/**
 * dma_alloc_coherent - allocate consistent memory for DMA
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @size: required memory size
 * @handle: bus-specific DMA address
 *
 * Allocate some uncached, unbuffered memory for a device for
 * performing DMA.  This function allocates pages, and will
 * return the CPU-viewed address, and sets @handle to be the
 * device-viewed address.
 */
extern void *dma_alloc_coherent(struct device *, size_t, dma_addr_t *, gfp_t);

/**
 * dma_free_coherent - free memory allocated by dma_alloc_coherent
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @size: size of memory originally requested in dma_alloc_coherent
 * @cpu_addr: CPU-view address returned from dma_alloc_coherent
 * @handle: device-view address returned from dma_alloc_coherent
 *
 * Free (and unmap) a DMA buffer previously allocated by
 * dma_alloc_coherent().
 *
 * References to memory and mappings associated with cpu_addr/handle
 * during and after this call executing are illegal.
 */
extern void dma_free_coherent(struct device *, size_t, void *, dma_addr_t);

#define dma_alloc_noncoherent(d, s, h, f) dma_alloc_coherent((d), (s), (h), (f))
#define dma_free_noncoherent(d, s, v, h)  dma_free_coherent((d), (s), (v), (h))

extern void __dma_single_cpu_to_dev(const void *kaddr, size_t size, 
				    enum dma_data_direction dir);

extern void __dma_single_dev_to_cpu(const void *kaddr, size_t size,
				    enum dma_data_direction dir);

extern void __dma_page_cpu_to_dev(struct page *page, unsigned long off,
				  size_t size, enum dma_data_direction dir);

extern void __dma_page_dev_to_cpu(struct page *page, unsigned long off,
				  size_t size, enum dma_data_direction dir);

/**
 * dma_map_single - map a single buffer for streaming DMA
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @cpu_addr: CPU direct mapped address of buffer
 * @size: size of buffer to map
 * @dir: DMA transfer direction
 *
 * Ensure that any data held in the cache is appropriately discarded
 * or written back.
 *
 * The device owns this memory once this call has completed.  The CPU
 * can regain ownership by calling dma_unmap_single() or
 * dma_sync_single_for_cpu().
 */
static inline dma_addr_t dma_map_single(struct device *dev, void *cpu_addr,
		size_t size, enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));

	__dma_single_cpu_to_dev(cpu_addr, size, dir);

	return virt_to_dma(dev, cpu_addr);
}

/**
 * dma_map_page - map a portion of a page for streaming DMA
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @page: page that buffer resides in
 * @offset: offset into page for start of buffer
 * @size: size of buffer to map
 * @dir: DMA transfer direction
 *
 * Ensure that any data held in the cache is appropriately discarded
 * or written back.
 *
 * The device owns this memory once this call has completed.  The CPU
 * can regain ownership by calling dma_unmap_page().
 */
static inline dma_addr_t dma_map_page(struct device *dev, struct page *page,
	     unsigned long offset, size_t size, enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));

	__dma_page_cpu_to_dev(page, offset, size, dir);

	return page_to_dma(dev, page) + offset;
}

/**
 * dma_unmap_single - unmap a single buffer previously mapped
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @handle: DMA address of buffer
 * @size: size of buffer (same as passed to dma_map_single)
 * @dir: DMA transfer direction (same as passed to dma_map_single)
 *
 * Unmap a single streaming mode DMA translation.  The handle and size
 * must match what was provided in the previous dma_map_single() call.
 * All other usages are undefined.
 *
 * After this call, reads by the CPU to the buffer are guaranteed to see
 * whatever the device wrote there.
 */
static inline void dma_unmap_single(struct device *dev, dma_addr_t handle,
		size_t size, enum dma_data_direction dir)
{
	__dma_single_dev_to_cpu(dma_to_virt(dev, handle), size, dir);
}

/**
 * dma_unmap_page - unmap a buffer previously mapped through dma_map_page()
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @handle: DMA address of buffer
 * @size: size of buffer (same as passed to dma_map_page)
 * @dir: DMA transfer direction (same as passed to dma_map_page)
 *
 * Unmap a page streaming mode DMA translation.  The handle and size
 * must match what was provided in the previous dma_map_page() call.
 * All other usages are undefined.
 *
 * After this call, reads by the CPU to the buffer are guaranteed to see
 * whatever the device wrote there.
 */
static inline void dma_unmap_page(struct device *dev, dma_addr_t handle,
		size_t size, enum dma_data_direction dir)
{
	__dma_page_dev_to_cpu(dma_to_page(dev, handle), handle & ~PAGE_MASK,
		size, dir);
}

/**
 * dma_sync_single_range_for_cpu
 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
 * @handle: DMA address of buffer
 * @offset: offset of region to start sync
 * @size: size of region to sync
 * @dir: DMA transfer direction (same as passed to dma_map_single)
 *
 * Make physical memory consistent for a single streaming mode DMA
 * translation after a transfer.
 *
 * If you perform a dma_map_single() but wish to interrogate the
 * buffer using the cpu, yet do not wish to teardown the PCI dma
 * mapping, you must call this function before doing so.  At the
 * next point you give the PCI dma address back to the card, you
 * must first the perform a dma_sync_for_device, and then the
 * device again owns the buffer.
 */
static inline void dma_sync_single_range_for_cpu(struct device *dev,
		dma_addr_t handle, unsigned long offset, size_t size,
		enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));

	__dma_single_dev_to_cpu(dma_to_virt(dev, handle) + offset, size, dir);
}

static inline void dma_sync_single_range_for_device(struct device *dev,
		dma_addr_t handle, unsigned long offset, size_t size,
		enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));

	__dma_single_cpu_to_dev(dma_to_virt(dev, handle) + offset, size, dir);
}

static inline void dma_sync_single_for_cpu(struct device *dev,
		dma_addr_t handle, size_t size, enum dma_data_direction dir)
{
	dma_sync_single_range_for_cpu(dev, handle, 0, size, dir);
}

static inline void dma_sync_single_for_device(struct device *dev,
		dma_addr_t handle, size_t size, enum dma_data_direction dir)
{
	dma_sync_single_range_for_device(dev, handle, 0, size, dir);
}

static inline void
dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg, int nents,
		    enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));
}

static inline void
dma_sync_sg_for_device(struct device *dev, struct scatterlist *sg, int nents,
		    enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));
}

extern int coherent_mem_init(void);

#endif /* __KERNEL__ */
#endif  /* _C6X_DMA_MAPPING_H */
