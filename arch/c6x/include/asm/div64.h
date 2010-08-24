/*
 *  linux/include/asm-c6x/div64.h
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
#ifndef __ASM_C6X_DIV64_H
#define __ASM_C6X_DIV64_H

#include <asm/types.h>

#if 0
static inline unsigned long __do_div(__u64 *n, unsigned long base)
{					\
	unsigned long __res;
	__res = ((unsigned long) *n) % (unsigned) base;
	*n = (__u64) (((unsigned long) *n) / (unsigned) base);
	return __res;
}

#define do_div(n,b) (__do_div((__u64*) (&(n)), (unsigned long) (b)))
#else
#include <asm-generic/div64.h>
#endif

#endif /* __ASM_C6X_DIV64_H */
