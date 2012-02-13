/*
 *  arch/c6x/boot/compressed/cache.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2012 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifdef __TMS320C66X__
#define mfence() asm volatile (" .word 0x10008000\n")
#else
#define mfence()
#endif

/*
 * Common to all C64x+/C6xx GEMs (thus platforms)
 */
#define IMCR_CCFG     0x01840000
#define IMCR_L1PCFG   0x01840020
#define IMCR_L1DCFG   0x01840040
#define IMCR_L2WBINV  0x01845004
#define IMCR_MAR_BASE 0x01848000

#define IMCR_MAR_PC      (1 << 0)

#define L2MODE_0K_CACHE  0x0
#define L2MODE_MAX_CACHE 0x7

#define CACHE_ENABLE  0
#define CACHE_DISABLE 1

/*
 * Generic DDR range
 */
#define RAM_START     0x80000000
#define RAM_END       0xFFFFFFFF

#define imcr_get(reg) *((volatile unsigned int *) (reg))
#define imcr_set(reg, value)                                      \
            do {                                                  \
	            *((volatile unsigned int *) (reg)) = (value); \
	            (value) = *((volatile unsigned int *) (reg)); \
            }  while(0)

/*
 * Set L2 operation mode
 */    
static inline void L2_cache_set_mode(unsigned int mode)
{
	unsigned int ccfg = imcr_get(IMCR_CCFG);

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
static void flush_and_invalidate(void)
{
	*((volatile unsigned int *) (IMCR_L2WBINV)) = 1;
	mfence();
	while (*((volatile unsigned int *) (IMCR_L2WBINV)));
}

void cache_control(unsigned int command)
{
	unsigned int cfg;

	unsigned int *mar   = (unsigned int *) IMCR_MAR_BASE
		+ ((unsigned int) RAM_START >> 24);
	unsigned int *mar_e = (unsigned int *) IMCR_MAR_BASE
		+ ((unsigned int) RAM_END >> 24);
	
	if (command == CACHE_ENABLE) {
		cfg = 7;
		L2_cache_set_mode(L2MODE_MAX_CACHE);
	}
	else {
		cfg = 0;
		L2_cache_set_mode(L2MODE_0K_CACHE);
	}

	/* Setup L1 */
	imcr_set(IMCR_L1PCFG, cfg);
	imcr_set(IMCR_L1DCFG, cfg);
	mfence();

	flush_and_invalidate();

	for (;mar <= mar_e; mar++) {
		if (command == CACHE_ENABLE)
			*mar |= IMCR_MAR_PC;
		else
			*mar &= ~IMCR_MAR_PC;
	}
	mfence();
}
