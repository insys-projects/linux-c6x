/*
 *  linux/include/asm-c6x/uaccess.h
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
#ifndef __ASM_C6X_UACCESS_H_
#define __ASM_C6X_UACCESS_H_

#include <asm/segment.h>
#include <asm/errno.h>
#include <asm/setup.h>

 /* Align on 32 bits */
#define __alignof__(x) 4

#define VERIFY_READ 0
#define VERIFY_WRITE 1

/* We let the MMU do all checking */
#define access_ok(type,addr,size) 1

extern unsigned int memory_end;
extern int text_start;

static inline int verify_area(int type, const void * addr, unsigned long size)
{
	if ((unsigned long) addr > (unsigned long) memory_end) {
		printk("Bad verify_area: %lx\n", (unsigned long) addr);
		return -EFAULT;
	}
	return 0;
}

/*
 * The exception table consists of pairs of addresses: the first is the
 * address of an instruction that is allowed to fault, and the second is
 * the address at which the program should continue.  No registers are
 * modified, so it is entirely up to the continuation code to figure out
 * what to do.
 *
 * All the routines below use bits of fixup code that are out of line
 * with the main instruction path.  This means when everything is well,
 * we don't even have to jump over them.  Further, they do not intrude
 * on our cache or tlb entries.
 */
struct exception_table_entry
{
	unsigned long insn, fixup;
};

/*
 * These are the main single-value transfer routines.  They automatically
 * use the right size if we just have the right pointer type.
 */
static inline int __generic_put_user(unsigned long x, void *y, int size)
{
        switch (size) {
                case 1:
                        *((unsigned char *) y) = (unsigned char) x;
                        break;
                case 2:
                        *((unsigned short *) y) = (unsigned short) x;
                        break;
                case 4:
                        *((unsigned int *) y) = (unsigned int) x;
                        break;
                default:
                        return -EFAULT;
        }
	return 0;
}

static inline int __generic_get_user(unsigned long *x, const void *y, int size)
{
        unsigned long result;

        switch (size) {
                case 1:
                        *(unsigned char *)x = *(unsigned char *) y;
			break;
                case 2:
                        *(unsigned short *)x = *(unsigned short *) y;
			break;
                case 4:
                        *(unsigned int *)x = *(unsigned int *) y;
			break;
                default:
                        return -EFAULT;
        }
	return 0;
}

#define put_user(x, ptr)   __generic_put_user((unsigned long) (x), (void *)(ptr), sizeof(*(ptr)))
#define __put_user(x, ptr) put_user(x, ptr)
#define get_user(x, ptr)   __generic_get_user((unsigned long *) &(x), (void *)(ptr), sizeof(*(ptr)))
#define __get_user(x, ptr) get_user(x, ptr)

static inline int __copy_from_user(void *to, const void *from, unsigned long n)
{
	memcpy(to, from, n);
	return 0;
}

static inline int __copy_to_user(void *to, const void *from, unsigned long n)
{
	memcpy(to, from, n);
	return 0;
}

#define __copy_to_user_inatomic __copy_to_user
#define __copy_from_user_inatomic __copy_from_user

#define copy_from_user(to, from, n) \
    (__copy_from_user((void *)(to), (const void *)(from), (unsigned long)(n)))
#define copy_to_user(to, from, n) \
    (__copy_from_user((void *)(to), (const void *)(from), (unsigned long)(n)))

/*
 * Copy a null terminated string from userspace.
 */
static inline long
strncpy_from_user(char *dst, const char *src, long count)
{
	char *tmp;
	strncpy(dst, src, count);
	for (tmp = dst; *tmp && count > 0; tmp++, count--)
		;
	return(tmp - dst);
}

/*
 * Return the size of a string (including the ending 0)
 *
 * Return 0 on exception, a value greater than N if too long
 */
static inline long strnlen_user(const char *src, long n)
{
	return(strlen(src) + 1);
}

#define strlen_user(str) strnlen_user(str, 32767)

/*
 * Zero Userspace
 */
static inline unsigned long
clear_user(void *to, unsigned long n)
{
	memset(to, 0, n);
	return(0);
}
#define __clear_user(to, n) clear_user((void *)(to), (unsigned long)(n))

static inline void __generic_memcpy_tofs(void * to, const void * from, unsigned long n)
{
	memcpy(to, from, n);
}

static inline void __constant_memcpy_tofs(void * to, const void * from, unsigned long n)
{
	memcpy(to, from, n);
}

static inline void __generic_memcpy_fromfs(void * to, const void * from, unsigned long n)
{
	memcpy(to, from, n);
}

static inline void __constant_memcpy_fromfs(void * to, const void * from, unsigned long n)
{
	memcpy(to, from, n);
}

static inline void memcpy_fromfs(void * to, const void * from, unsigned long n)
{
	memcpy(to, from, n);
}

static inline void memcpy_tofs(void * to, const void * from, unsigned long n)
{
	memcpy(to, from, n);
}

/* If gcc inlines memset, it will use st.q instructions.  Therefore, we need
   kmalloc allocations to be 8-byte aligned.  Without this, the alignment
   becomes BYTE_PER_WORD i.e. only 4 (since sizeof(long)==sizeof(void*)==4 on
   sh64 at the moment). */
#define ARCH_KMALLOC_MINALIGN 8

/*
 * We want 8-byte alignment for the slab caches as well, otherwise we have
 * the same BYTES_PER_WORD (sizeof(void *)) min align in kmem_cache_create().
 */
#define ARCH_SLAB_MINALIGN 8

#endif /* __ASM_C6X_UACCESS_H_ */

