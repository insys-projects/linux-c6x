/*
 *  linux/include/asm-c6x/segment.h
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
#ifndef __ASM_C6X_SEGMENT_H
#define __ASM_C6X_SEGMENT_H

/* define constants */
/* Address spaces (FC0-FC2) */
#define USER_DATA     (1)
#ifndef __USER_DS
#define __USER_DS     (USER_DATA)
#endif
#define USER_PROGRAM  (2)
#define SUPER_DATA    (5)
#ifndef __KERNEL_DS
#define __KERNEL_DS   (SUPER_DATA)
#endif
#define SUPER_PROGRAM (6)
#define CPU_SPACE     (7)

#ifndef __ASSEMBLY__

typedef unsigned long mm_segment_t;

#define MAKE_MM_SEG(s)  ((mm_segment_t) (s))
#define USER_DS         MAKE_MM_SEG(__USER_DS)
#define KERNEL_DS       MAKE_MM_SEG(__KERNEL_DS)

/*
 * Get/set the SFC/DFC registers for MOVES instructions
 */

#define get_fs()	(current_thread_info()->addr_limit)
#define set_fs(x)	(current_thread_info()->addr_limit = (x))

#define get_ds()	KERNEL_DS

#define segment_eq(a,b) ((a) == (b))

#endif /* __ASSEMBLY__ */
#endif /* __ASM_C6X_SEGMENT_H */
