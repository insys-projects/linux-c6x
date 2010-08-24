/*
 *  linux/include/asm-c6x/unaligned.h
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
#ifndef __ASM_C6X_UNALIGNED_H
#define __ASM_C6X_UNALIGNED_H

/*
 * The C6x can do unaligned accesses itself. 
 *
 * The strange macros are there to make sure these can't
 * be misused in a way that makes them not work on other
 * architectures where unaligned accesses aren't as simple.
 */
#include <linux/unaligned/access_ok.h>
#include <linux/unaligned/generic.h>

#define get_unaligned(ptr)      (*(ptr))
#define put_unaligned(val, ptr) ((void)( *(ptr) = (val) ))

#endif /* __ASM_C6X_UNALIGNED_H */
