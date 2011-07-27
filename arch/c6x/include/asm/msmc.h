/*
 *  arch/c6x/include/asm/msmc.h
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
#ifndef __ASM_C6X_MSMC_H
#define __ASM_C6X_MSMC_H

#ifdef __KERNEL__
u32 msm_virt_to_phys(u32 vaddr);
u32 msm_phys_to_virt(u32 paddr);
u32 msm_get_heap(void);
u32 msm_get_top(void);
void *msm_alloc_coherent(size_t size, dma_addr_t *handle);
void msm_mem_init(void);

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_MSMC_H */
