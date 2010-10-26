/*
 *  linux/include/asm-c6x/bitops.h
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
#ifndef __ASM_C6X_BITOPS_H_
#define __ASM_C6X_BITOPS_H_

#ifdef __KERNEL__

#include <linux/bitops.h>

#include <asm/system.h>
#include <asm/byteorder.h>

/*
 * clear_bit() doesn't provide any barrier for the compiler.
 */
#define smp_mb__before_clear_bit() barrier()
#define smp_mb__after_clear_bit()  barrier()

/* Use generic versions of non-atomic ops */
#include <asm-generic/bitops/non-atomic.h>

static __inline__ void set_bit(int nr, volatile void *addr)
{
	volatile unsigned long *a = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long mask = BIT_MASK(nr);
	unsigned long flags;

	local_irq_save(flags);
	*a |= mask;
	local_irq_restore(flags);
}

static __inline__ void clear_bit(int nr, volatile void *addr)
{
	volatile unsigned long *a = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long mask = BIT_MASK(nr);
	unsigned long flags;

	local_irq_save(flags);
	*a &= ~mask;
	local_irq_restore(flags);
}

static __inline__ void change_bit(int nr, volatile void * addr)
{
	volatile unsigned long *a = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long mask = BIT_MASK(nr);
	unsigned long flags;

	local_irq_save(flags);
	*a ^= mask;
	local_irq_restore(flags);
}

static inline int test_and_set_bit(int nr, volatile void *addr)
{
	volatile unsigned long *a = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long mask = BIT_MASK(nr);
	unsigned long flags;
	int retval;

	local_irq_save(flags);
	retval = (mask & *a) != 0;
	*a |= mask;
	local_irq_restore(flags);
	return retval;
}

static inline int test_and_clear_bit(int nr, volatile void *addr)
{
	unsigned long *a = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long mask = BIT_MASK(nr);
	unsigned long retval;
	unsigned long flags;

	local_irq_save(flags);
	retval = (mask & *a) != 0;
	*a &= ~mask;
	local_irq_restore(flags);
	return retval;
}

static __inline__ int test_and_change_bit(int nr, volatile void *addr)
{
	unsigned long *a = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long mask = BIT_MASK(nr);
	unsigned long flags;
	int retval;

	local_irq_save(flags);
	retval = (mask & *a) != 0;
	*a ^= mask;
	local_irq_restore(flags);
	return retval;
}

/*
 * We are lucky, DSP is perfect for bitops: do it in 3 cycles
 */
#ifdef CONFIG_TI_C6X_COMPILER
unsigned int _lmbd(unsigned int, unsigned int);
unsigned int _bitr(unsigned int);

#define ffz(a)     (_lmbd(0, _bitr(a)))
#define __ffs(a)   (_lmbd(1, _bitr(a)))
#define fls(a)     (!(a)?0:(32 - _lmbd(1, (a))))

#else /* CONFIG_TI_C6X_COMPILER */

/**
 * __ffs - find first bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 * Note __ffs(0) = undef, __ffs(1) = 0, __ffs(0x80000000) = 31.
 *
 */
static inline unsigned long __ffs(unsigned long x)
{
	asm (" bitr  .M1  %0,%0\n"
	     " nop\n"
	     " lmbd  .L1  1,%0,%0\n"
	     : "+a"(x));

	return x;
}

/*
 * ffz - find first zero in word.
 * @word: The word to search
 *
 * Undefined if no zero exists, so code should check against ~0UL first.
 */
#define ffz(x) __ffs(~(x))

/**
 * fls - find last (most-significant) bit set
 * @x: the word to search
 *
 * This is defined the same way as ffs.
 * Note fls(0) = 0, fls(1) = 1, fls(0x80000000) = 32.
 */
static inline unsigned long fls(unsigned long x)
{
	if (!x)
		return 0;

	asm (" lmbd  .L1  1,%0,%0\n" : "+a"(x));

	return 32 - x;
}

#endif /* CONFIG_TI_C6X_COMPILER */

/**
 * ffs - find first bit set
 * @x: the word to search
 *
 * This is defined the same way as
 * the libc and compiler builtin ffs routines, therefore
 * differs in spirit from the above ffz (man ffs).
 * Note ffs(0) = 0, ffs(1) = 1, ffs(0x80000000) = 32.
 */
static inline int ffs(int x)
{
	if (!x)
		return 0;

	return __ffs(x) + 1;
}

#include <asm-generic/bitops/__fls.h>
#include <asm-generic/bitops/fls64.h>
#include <asm-generic/bitops/find.h>
#include <asm-generic/bitops/sched.h>
#include <asm-generic/bitops/hweight.h>
#include <asm-generic/bitops/lock.h>
#include <asm-generic/bitops/find.h>
#include <asm-generic/bitops/ext2-non-atomic.h>
#include <asm-generic/bitops/ext2-atomic.h>
#include <asm-generic/bitops/minix.h>

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_BITOPS_H */
