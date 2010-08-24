/*
 *  linux/include/asm-c6x/string.h
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
#ifndef __ASM_C6X_STRING_H
#define __ASM_C6X_STRING_H

#include <asm/page.h>
#include <linux/linkage.h>

asmlinkage extern void * memcpy(void * to, const void * from, size_t n);
extern void * memset(void * s, int c, size_t count);
extern int memcmp(const void * cs,const void * ct,size_t count);
extern void *memmove(void *s1, const void *s2, size_t n);

#define __HAVE_ARCH_MEMCPY
#define __HAVE_ARCH_MEMMOVE
#define __HAVE_ARCH_MEMSET
#define __HAVE_ARCH_MEMCMP

#endif /* __ASM_C6X_STRING_H */
