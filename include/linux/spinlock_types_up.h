#ifndef __LINUX_SPINLOCK_TYPES_UP_H
#define __LINUX_SPINLOCK_TYPES_UP_H

#ifndef __LINUX_SPINLOCK_TYPES_H
# error "please don't include this file directly"
#endif

/*
 * include/linux/spinlock_types_up.h - spinlock type definitions for UP
 *
 * portions Copyright 2005, Red Hat, Inc., Ingo Molnar
 * Released under the General Public License (GPL).
 */

#ifdef CONFIG_DEBUG_SPINLOCK

typedef struct {
	volatile unsigned int slock;
} arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED { 1 }

#else

#ifdef CONFIG_TI_C6X_COMPILER
/*
 * TI compiler doesn't allow static initialization of nested empty
 * structs quite like gcc, so...
 */
typedef struct {
	volatile unsigned int slock;
} arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED { 0 }

#else
typedef struct { } arch_spinlock_t;

#define __ARCH_SPIN_LOCK_UNLOCKED { }
#endif

#endif

#ifdef CONFIG_TI_C6X_COMPILER
typedef struct {
	volatile unsigned int slock;
} arch_rwlock_t;

#ifdef CONFIG_DEBUG_SPINLOCK
#define __ARCH_RW_LOCK_UNLOCKED		{ 1 }
#else
#define __ARCH_RW_LOCK_UNLOCKED		{ 0 }
#endif

#else
typedef struct {
	/* no debug version on UP */
} arch_rwlock_t;

#define __ARCH_RW_LOCK_UNLOCKED { }
#endif

#endif /* __LINUX_SPINLOCK_TYPES_UP_H */
