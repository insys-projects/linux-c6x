/*
 *  linux/include/asm-c6x/dma-mapping.h
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
 */
#ifndef _C6X_DMA_MAPPING_H
#define _C6X_DMA_MAPPING_H

#include <linux/mm.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/scatterlist.h>

/* arch/c6x/mm/consistent.c */

#define dma_supported(d, m)         (1)

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

static inline dma_addr_t dma_map_single(struct device *dev, void *ptr, size_t size,
        enum dma_data_direction direction)
{
        BUG_ON(direction == DMA_NONE);

        return __pa(ptr);
}

static inline void dma_unmap_single(struct device *dev, dma_addr_t dma_addr, size_t size,
                 enum dma_data_direction direction)
{
        BUG_ON(direction == DMA_NONE);
}


static inline void *dma_alloc_coherent(struct device *dev, size_t size,
			 dma_addr_t *dma_handle, int flag)
{
	return consistent_alloc(flag, size, dma_handle);
}

static inline void dma_free_coherent(struct device *dev, size_t size,
		       void *vaddr, dma_addr_t dma_handle)
{
	consistent_free(vaddr, size, dma_handle);
}

#define dma_alloc_noncoherent(d, s, h, f) dma_alloc_coherent((d), (s), (h), (f))
#define dma_free_noncoherent(d, s, v, h)  dma_free_coherent((d), (s), (v), (h))

static inline void
dma_sync_single_range_for_cpu(struct device *dev, dma_addr_t handle,
			      unsigned long offset, size_t size,
			      enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));
}

static inline void
dma_sync_single_range_for_device(struct device *dev, dma_addr_t handle,
				 unsigned long offset, size_t size,
				 enum dma_data_direction dir)
{
	/* _dma_sync(handle + offset, size, dir); */
}

static inline void
dma_sync_single_for_cpu(struct device *dev, dma_addr_t handle, size_t size,
			enum dma_data_direction dir)
{
	dma_sync_single_range_for_cpu(dev, handle, 0, size, dir);
}

static inline void
dma_sync_single_for_device(struct device *dev, dma_addr_t handle, size_t size,
			   enum dma_data_direction dir)
{
	dma_sync_single_range_for_device(dev, handle, 0, size, dir);
}

static inline void
dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg, int nents,
		    enum dma_data_direction dir)
{
	BUG_ON(!valid_dma_direction(dir));
}



#endif  /* _C6X_DMA_MAPPING_H */
