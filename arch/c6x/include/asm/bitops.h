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

#if defined(CONFIG_TMS320C64X) || defined(CONFIG_TMS320C64XPLUS)
unsigned int _lmbd(unsigned int, unsigned int);
unsigned int _bitr(unsigned int);

/*
 * We are lucky, DSP is perfect for bitops: do it in 3 cycles
 */
#define ffz(a)     (_lmbd(0, _bitr(a)))
#define __ffs(a)   (_lmbd(1, _bitr(a)))
#define fls(a)     (32 - _lmbd(1, (a)))
/*
 * ffs: find first bit set. This is defined the same way as
 * the libc and compiler builtin ffs routines, therefore
 * differs in spirit from the above ffz (man ffs).
 */
#define ffs(x) ((__ffs(x) + 1) % 33)

#else /* CONFIG_TMS320C64X || CONFIG_TMS320C64XPLUS */
#include <asm-generic/bitops/ffz.h>
#include <asm-generic/bitops/__ffs.h>
#include <asm-generic/bitops/fls.h>
#include <asm-generic/bitops/ffs.h>
#endif /* CONFIG_TMS320C64X || CONFIG_TMS320C64XPLUS */

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
