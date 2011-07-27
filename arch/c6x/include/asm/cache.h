/*
 *  linux/include/asm-c6x/cache.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2005, 2006, 2009, 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_CACHE_H_
#define __ASM_C6X_CACHE_H_

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/xmc.h>
#include <mach/cache.h>

/*
 * For practical reasons the L1_CACHE_BYTES defines should not be smaller than
 * the L2 line size
 */
#define L1_CACHE_BYTES        L2_CACHE_BYTES 

#define L2_CACHE_ALIGN_LOW(x) (((x) & ~(L2_CACHE_BYTES - 1)))
#define L2_CACHE_ALIGN_UP(x)  (((x) + (L2_CACHE_BYTES - 1)) & ~(L2_CACHE_BYTES - 1))
#define L2_CACHE_ALIGN_CNT(x) (((x) + (sizeof(int) - 1)) & ~(sizeof(int) - 1))

#define L1_CACHE_ALIGN_LOW(x) (((x) & ~(L1_CACHE_BYTES - 1)))
#define L1_CACHE_ALIGN_UP(x)  (((x) + (L1_CACHE_BYTES - 1)) & ~(L1_CACHE_BYTES - 1))
#define L1_CACHE_ALIGN_CNT(x) (((x) + (sizeof(int) - 1)) & ~(sizeof(int) - 1))

/*
 * Some drivers use DMA to access kmalloc'd buffers.
 */
#define ARCH_KMALLOC_MINALIGN L2_CACHE_BYTES

/*
 * We want 8-byte alignment for the slab caches.
 */
#define ARCH_SLAB_MINALIGN    8

/*
 * Current C6x architecture does not support hw cache coherency
 */
#define arch_is_coherent()    0

/*
 * Align a physical address to MAR regions
 */
#define CACHE_REGION_START(v) (((u32) (v)) & ~(IMCR_MAR_SIZE - 1))
#define CACHE_REGION_END(v)   (((u32) (v) + (IMCR_MAR_SIZE - 1)) & ~(IMCR_MAR_SIZE - 1))

/*
 * CCFG register values and bits
 */
#define L2MODE_0K_CACHE       0x0
#define L2MODE_32K_CACHE      0x1
#define L2MODE_64K_CACHE      0x2
#define L2MODE_128K_CACHE     0x3
#define L2MODE_256K_CACHE     0x4
#define L2MODE_512K_CACHE     0x5
#define L2MODE_1024K_CACHE    0x6
#define L2MODE_MAX_CACHE      0x7

#define L2PRIO_URGENT         0x0
#define L2PRIO_HIGH           0x1
#define L2PRIO_MEDIUM         0x2
#define L2PRIO_LOW            0x3

#define CCFG_ID               0x100   /* Invalidate L1P bit */
#define CCFG_IP               0x200   /* Invalidate L1D bit */
	
/*
 * L1 & L2 caches generic functions
 */
#define imcr_get(reg) 	  *((volatile unsigned int *) (reg))
#define imcr_set(reg, value)                                      \
            do {                                                  \
	            *((volatile unsigned int *) (reg)) = (value); \
	            (value) = *((volatile unsigned int *) (reg)); \
            }  while(0)

#define CACHE_IS_L2(wc_reg) ((wc_reg == IMCR_L2WWC)	\
			     | (wc_reg == IMCR_L2WIWC)	\
			     | (wc_reg == IMCR_L2IWC))

/*
 * Generic function to perform a block cache operation as
 * invalidate or writeback/invalidate
 */	
static inline void cache_block_operation(unsigned int *start,
					 unsigned int *end,
					 unsigned int bar_reg,
					 unsigned int wc_reg)
{
	unsigned long flags;
	unsigned int wcnt = CACHE_IS_L2(wc_reg) ?
		(L2_CACHE_ALIGN_CNT((unsigned int) end)
		 - L2_CACHE_ALIGN_LOW((unsigned int) start)) >> 2 :
		(L1_CACHE_ALIGN_CNT((unsigned int) end)
		 - L1_CACHE_ALIGN_LOW((unsigned int) start)) >> 2;
	unsigned int wc = 0;

	xmc_prefetch_buffer_invalidate();
	
	for (; wcnt; wcnt -= wc, start += wc) {
loop:		
		save_global_flags(flags);
		global_cli();

		/*
		 * If another cache operation is occuring
		 */
	        if(unlikely(*((volatile unsigned int *) wc_reg))) {
		    restore_global_flags(flags);

		    /* Wait for previous operation completion */
		    while (*((volatile unsigned int *) wc_reg));

		    /* Try again */
		    goto loop;
		}

		*((volatile unsigned int *) bar_reg) = CACHE_IS_L2(wc_reg) ?
			L2_CACHE_ALIGN_LOW((unsigned int) start) :
			L1_CACHE_ALIGN_LOW((unsigned int) start);
		
		if (wcnt > 0xffff)
			wc = 0xffff;
		else
			wc = wcnt;

		/* Set word count value in the WC register */
		*((volatile unsigned int *) wc_reg) = wc & 0xffff;

		restore_global_flags(flags);
		
		/* Wait for completion */
		mfence();
		while (*((volatile unsigned int *) wc_reg));
	}
}

static inline void cache_block_operation_nowait(unsigned int *start,
						unsigned int *end,
						unsigned int bar_reg,
						unsigned int wc_reg)
{
	unsigned long flags;
	unsigned int wcnt = CACHE_IS_L2(wc_reg) ?
		(L2_CACHE_ALIGN_CNT((unsigned int) end)
		 - L2_CACHE_ALIGN_LOW((unsigned int) start)) >> 2 :
		(L1_CACHE_ALIGN_CNT((unsigned int) end)
		 - L1_CACHE_ALIGN_LOW((unsigned int) start)) >> 2;
	unsigned int wc = 0;

	xmc_prefetch_buffer_invalidate();
	
	for (; wcnt; wcnt -= wc, start += wc) {

		save_global_flags(flags);
		global_cli();

		*((volatile unsigned int *) bar_reg) = CACHE_IS_L2(wc_reg) ?
			L2_CACHE_ALIGN_LOW((unsigned int) start) :
			L1_CACHE_ALIGN_LOW((unsigned int) start);

		if (wcnt > 0xffff)
			wc = 0xffff;
		else
			wc = wcnt;
		
		/* Set word count value in the WC register */
		*((volatile unsigned int *) wc_reg) = wc & 0xffff;

		restore_global_flags(flags);
		
		/* Don't wait for completion on last cache operation */
		if (wcnt > 0xffff)
			while (*((volatile unsigned int *) wc_reg));
	}
}

static inline void cache_block_operation_wait(unsigned int wc_reg)
{
	/* Wait for completion */
	mfence();
	while (*((volatile unsigned int *) wc_reg));
}

/*
 * L1 caches management
 */
    
/*
 * Disable L1 caches
 */
static inline void L1_cache_off(void)
{
#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)
	unsigned int cfg = 0;

	xmc_prefetch_buffer_invalidate();

	imcr_set(IMCR_L1PCFG, cfg);
	imcr_set(IMCR_L1DCFG, cfg);
	mfence();
#else
	CSR &= ~(0xfc);
#endif
}

/* 
 * Enable L1 caches
 */
static inline void L1_cache_on(void)
{
#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)
	unsigned int cfg = 7;

	xmc_prefetch_buffer_invalidate();

	imcr_set(IMCR_L1PCFG, cfg);
	imcr_set(IMCR_L1DCFG, cfg);
	mfence();
#else
	CSR &= ~(0xfc);
	CSR |= 0x48;
#endif
}

/*
 *  L1P global-invalidate all
 */    
#if !defined(CONFIG_TMS320C64XPLUS) && !defined(CONFIG_TMS320C66X)
static inline void L1P_cache_global_invalidate(void)
{
	unsigned int ccfg = imcr_get(IMCR_CCFG);
	ccfg |= CCFG_IP;
	imcr_set(IMCR_CCFG, ccfg);
}
#else
static inline void L1P_cache_global_invalidate(void)
{
	unsigned int set = 1;

	xmc_prefetch_buffer_invalidate();

	imcr_set(IMCR_L1PINV, set);
	while (imcr_get(IMCR_L1PINV) & 1);

	mfence();
}
#endif

/*
 *  L1D global-invalidate all
 *
 * Warning: this operation causes all updated data in L1D to
 * be discarded rather than written back to the lower levels of
 * memory
 */
#if !defined(CONFIG_TMS320C64XPLUS) && !defined(CONFIG_TMS320C66X)
static inline void L1D_cache_global_invalidate(void)
{
	unsigned int ccfg = imcr_get(IMCR_CCFG);
	ccfg |= CCFG_ID;
	imcr_set(IMCR_CCFG, ccfg);
}
#else
static inline void L1D_cache_global_invalidate(void)
{
	unsigned int set = 1;

	xmc_prefetch_buffer_invalidate();

	imcr_set(IMCR_L1DINV, set);
	while (imcr_get(IMCR_L1DINV) & 1);

	mfence();
}

static inline void L1D_cache_global_writeback(void)
{
	unsigned int set = 1;

	xmc_prefetch_buffer_invalidate();
	imcr_set(IMCR_L1DWB, set);
	mfence();
	while (imcr_get(IMCR_L1DWB) & 1);
}

static inline void L1D_cache_global_writeback_invalidate(void)
{
	unsigned int set = 1;

	xmc_prefetch_buffer_invalidate();
	imcr_set(IMCR_L1DWBINV, set);
	mfence();
	while (imcr_get(IMCR_L1DWBINV) & 1);
}
#endif

/*
 *  L1 block operations
 */
#define L1P_cache_block_invalidate(start, end)             \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L1PIBAR, IMCR_L1PIWC)

#define L1D_cache_block_invalidate(start, end)             \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L1DIBAR, IMCR_L1DIWC)

#define L1D_cache_block_writeback_invalidate(start, end)   \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L1DWIBAR, IMCR_L1DWIWC)

#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)
#define L1D_cache_block_writeback(start, end)              \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L1DWBAR, IMCR_L1DWWC)
#endif

#ifdef CONFIG_TMS320C6X_CACHES_ON
/*
 * Set L2 operation mode
 */    
static inline void L2_cache_set_mode(unsigned int mode)
{
	unsigned int ccfg = imcr_get(IMCR_CCFG);

	xmc_prefetch_buffer_invalidate();	

	/* Clear and set the L2MODE bits in CCFG */
	ccfg &= ~7;
	ccfg |= (mode & 7);
	imcr_set(IMCR_CCFG, ccfg);
	ccfg = imcr_get(IMCR_CCFG);
	mfence();
}

/*
 *  L2 global-writeback and global-invalidate all
 */    
static inline void L2_cache_global_writeback_invalidate(void)
{
	xmc_prefetch_buffer_invalidate();
	*((volatile unsigned int *) (IMCR_L2WBINV)) = 1;
	mfence();
	while (*((volatile unsigned int *) (IMCR_L2WBINV)));
}

/*
 *  L2 global-writeback all
 */    
static inline void L2_cache_global_writeback(void)
{
	xmc_prefetch_buffer_invalidate();
	*((volatile unsigned int *) (IMCR_L2WB)) = 1;
	mfence();
	while (*((volatile unsigned int *) (IMCR_L2WB)));
}

/*
 *  L2 block operations
 */    
#define L2_cache_block_invalidate(start, end)              \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L2IBAR, IMCR_L2IWC)

#define L2_cache_block_writeback(start, end)               \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L2WBAR, IMCR_L2WWC)

#define L2_cache_block_writeback_invalidate(start, end)    \
        cache_block_operation((unsigned int *) (start),    \
                              (unsigned int *) (end),      \
                              IMCR_L2WIBAR, IMCR_L2WIWC)

#define L2_cache_block_invalidate_nowait(start, end)              \
        cache_block_operation_nowait((unsigned int *) (start),    \
				     (unsigned int *) (end),      \
				     IMCR_L2IBAR, IMCR_L2IWC)


#define L2_cache_block_invalidate_wait()	\
        cache_block_operation_wait(IMCR_L2IWC)

#define L2_cache_block_writeback_nowait(start, end)               \
        cache_block_operation_nowait((unsigned int *) (start),    \
				     (unsigned int *) (end),      \
				     IMCR_L2WBAR, IMCR_L2WWC)

#define L2_cache_block_writeback_wait()		\
        cache_block_operation_wait(IMCR_L2WWC)

#define L2_cache_block_writeback_invalidate_nowait(start, end)    \
        cache_block_operation_nowait((unsigned int *) (start),    \
				     (unsigned int *) (end),      \
				     IMCR_L2WIBAR, IMCR_L2WIWC)

#define L2_cache_block_writeback_invalidate_wait()    \
        cache_block_operation_wait(IMCR_L2WIWC)

/*
 * Cacheability controls
 */
static inline void enable_caching(unsigned int *start,	unsigned int *end)
{
	unsigned int *mar   = (unsigned int *) IMCR_MAR_BASE\
		+ ((unsigned int) start >> 24);
	unsigned int *mar_e = (unsigned int *) IMCR_MAR_BASE\
		+ ((unsigned int) end >> 24);
	
	L2_cache_global_writeback_invalidate();

	for (;mar <= mar_e; mar++) {
		*mar |= IMCR_MAR_PC;
#ifdef ARCH_HAS_XMC_PREFETCHW
		*mar |= IMCR_MAR_PFX;
#endif
	}
	mfence();
}

static inline void disable_caching(unsigned int *start, unsigned int *end)
{
	unsigned int *mar   = (unsigned int *) IMCR_MAR_BASE\
		+ ((unsigned int) start >> 24);
	unsigned int *mar_e = (unsigned int *) IMCR_MAR_BASE\
		+ ((unsigned int) end >> 24);

	L2_cache_global_writeback_invalidate();
	
	for (;mar <= mar_e; mar++) {
		*mar &= ~IMCR_MAR_PC;
#ifdef ARCH_HAS_XMC_PREFETCHW
		*mar &= ~IMCR_MAR_PFX;
#endif
	}
	mfence();
}

#else /* CONFIG_TMS320C6X_CACHES_ON */
#define L2_cache_set_mode(mode)
#define L2_cache_global_writeback_invalidate()
#define L2_cache_global_writeback()
#define L2_cache_block_invalidate(start, end)
#define L2_cache_block_writeback(start, end)
#define L2_cache_block_writeback_invalidate(start, end)
#define L2_cache_block_invalidate_nowait(start, end)
#define L2_cache_block_invalidate_wait()
#define L2_cache_block_writeback_nowait(start, end)
#define L2_cache_block_writeback_wait()
#define L2_cache_block_writeback_invalidate_nowait(start, end)
#define L2_cache_block_writeback_invalidate_wait()

#endif /* CONFIG_TMS320C6X_CACHES_ON */
#endif /* __ASM_C6X_CACHE_H_ */
