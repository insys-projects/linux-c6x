/*
 * linux/include/asm-c6x/processor.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  Updated for 2.6.34: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_PROCESSOR_H
#define __ASM_C6X_PROCESSOR_H
/*
 * Default implementation of macro that returns current
 * instruction pointer ("program counter").
 */
#ifdef CONFIG_TI_C6X_COMPILER
extern void *current_text_addr(void);
#else
#define current_text_addr()			\
({						\
	void *__pc;				\
	asm("mvc .S2 pce1,%0\n" : "=b"(__pc));	\
	__pc;					\
})
#endif

#include <asm/segment.h>
#include <asm/ptrace.h>
#include <asm/page.h>
#include <linux/linkage.h>
#include <asm/current.h>

/*
 * User space process size . This is hardcoded into a few places,
 * so don't change it unless you know what you are doing.
 */
#define TASK_SIZE	        (0xF0000000UL)

/*
 * This decides where the kernel will search for a free chunk of vm
 * space during mmap's. We won't be using it
 */
#define TASK_UNMAPPED_BASE	0

/*
 * Bus types
 */
#define EISA_bus__is_a_macro	1
#define EISA_bus 0
#define MCA_bus__is_a_macro	1
#define MCA_bus 0

/*
 * The C6x has no problems with write protection
 */
#define wp_works_ok__is_a_macro	1
#define wp_works_ok 1

struct thread_struct {
	unsigned long  ksp;		/* kernel stack pointer */
	unsigned long  usp;		/* user stack pointer */
	unsigned long  csr;		/* saved control status register */
	unsigned long  fs;	        /* saved fs (sfc, dfc) */
	unsigned long  tls;             /* thread-local storage */
};

#define INIT_THREAD					\
{							\
	.ksp = sizeof(init_stack) + (long) init_stack,	\
	.usp = 0,					\
	.csr = DEFAULT_CSR,				\
	.fs  = __KERNEL_DS,				\
	.tls = 0,				        \
}

#define INIT_MMAP \
{ &init_mm, 0, 0, NULL, PAGE_SHARED, VM_READ | VM_WRITE | VM_EXEC, 1, \
  NULL, NULL }

#define task_pt_regs(task) \
	((struct pt_regs *)(THREAD_START_SP + task_stack_page(task)) - 1)

#define alloc_kernel_stack()    __get_free_page(GFP_KERNEL)
#define free_kernel_stack(page) free_page((page))


/* Forward declaration, a strange C thing */
struct task_struct;

extern void start_thread(struct pt_regs * regs, unsigned int pc,  
			 unsigned long usp);

/* Free all resources held by a thread. */
static inline void release_thread(struct task_struct *dead_task)
{
}

/* Prepare to copy thread state - unlazy all lazy status */
#define prepare_to_copy(tsk)	do { } while (0)

extern int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags);

#define copy_segments(tsk, mm)		do { } while (0)
#define release_segments(mm)		do { } while (0)

/*
 * Return saved PC of a blocked thread.
 */
#define thread_saved_pc(tsk) \
	((tsk)->thread.ksp? ((struct switch_stack *)(tsk)->thread.ksp)->retpc: 0)

extern unsigned long get_wchan(struct task_struct *p);
extern unsigned long __kstk_eip(struct task_struct *p);

#define KSTK_EIP(tsk)   __kstk_eip((tsk))
#define	KSTK_ESP(tsk)	(tsk)->thread.usp

#define cpu_relax()             do { } while (0)

#ifdef CONFIG_TI_C6X_COMPILER
#define ARCH_HAS_PREFETCH
#define prefetch(x) (0)

#define ARCH_HAS_PREFETCHW
#define prefetchw(x) (0)

#define ARCH_HAS_SPINLOCK_PREFETCH
#define spin_lock_prefetch(x) (0)
#endif

#endif /* ASM_C6X_PROCESSOR_H */
