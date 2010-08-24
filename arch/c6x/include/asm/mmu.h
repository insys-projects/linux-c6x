/*
 *  linux/include/asm-c6x/mmu.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_MMU_H
#define __ASM_C6X_MMU_H

typedef struct {
	unsigned long		end_brk;
#ifdef CONFIG_BINFMT_ELF_DSBT
	unsigned long	exec_dsbt_loadmap;
	unsigned long	interp_dsbt_loadmap;
#endif
} mm_context_t;

#endif /* __ASM_C6X_MMU_H */
