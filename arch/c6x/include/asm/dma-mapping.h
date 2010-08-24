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
extern void *consistent_alloc(int gfp, size_t size, dma_addr_t *handle);
extern void consistent_free(void *vaddr, size_t size, dma_addr_t handle);

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

#endif  /* _C6X_DMA_MAPPING_H */
