/*
 *  linux/include/asm-c6x/types.h
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
#ifndef __ASM_C6X_TYPES_H
#define __ASM_C6X_TYPES_H

/*
 * This file is never included by application software unless
 * explicitly requested (e.g., via linux/types.h) in which case the
 * application is Linux specific so (user-) name space pollution is
 * not a major issue.  However, for interoperability, libraries still
 * need to be careful to avoid a name clashes.
 */
typedef unsigned short umode_t;

/*
 * __xx is ok: it doesn't pollute the POSIX namespace. Use these in the
 * header files exported to user space
 */

typedef /*__signed__*/ char __s8;
typedef unsigned char __u8;

typedef /*__signed__*/ short __s16;
typedef unsigned short __u16;

typedef /*__signed__*/ int __s32;
typedef unsigned int __u32;

typedef /*__signed__*/ long long __s64;
typedef unsigned long long __u64;

/*
 * These aren't exported outside the kernel to avoid name space clashes
 */
#ifdef __KERNEL__

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

typedef s64	loff_t;
typedef u32     dma_addr_t;
         
typedef unsigned short kmem_bufctl_t;
     
#define BITS_PER_LONG 32

typedef		__u64		uint64_t;
typedef		__u64		u_int64_t;
typedef		__s64		int64_t;

#define _Bool  unsigned int

#endif /* __KERNEL__ */

#endif /* __ASM_C6X_TYPES_H */
