#ifndef __LINUX_SPINLOCK_TYPES_UP_H
#define __LINUX_SPINLOCK_TYPES_UP_H

#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

/*
 * We only need spinlocks for SMP but the TI compiler doesn't
 * allow static initialization of nested empty structs quite
 * like gcc. 
 */
#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

typedef struct {
	volatile unsigned int slock;
} arch_spinlock_t;


typedef struct {
	volatile unsigned int slock;
} arch_rwlock_t;


/*  
 * In the debug case, 1 means unlocked, 0 means locked. (the values
 * are inverted, to catch initialization bugs)
 */
#ifdef CONFIG_DEBUG_SPINLOCK
#define __ARCH_SPIN_LOCK_UNLOCKED	{ 1 }
#define __ARCH_RW_LOCK_UNLOCKED		{ 1 }
#else
#define __ARCH_SPIN_LOCK_UNLOCKED	{ 0 }
#define __ARCH_RW_LOCK_UNLOCKED		{ 0 }
#endif

#endif
