/*
 *  linux/include/asm-c6x/system.h
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
#ifndef __ASM_C6X_SYSTEM_H
#define __ASM_C6X_SYSTEM_H

#include <linux/linkage.h>
#include <asm/segment.h>

/*
 * switch_to() saves the extra registers, that are not saved
 * automatically by SAVE_SWITCH_STACK in resume().
 * Pass previous in A4, next in B4, offset in A6, and shared in B6
 * (We just respect the C call convention...)
 */

#define prepare_to_switch()    do { } while(0)

asmlinkage void * resume(void *prev, void *next, int thread, char shared);

#define switch_to(prev,next,last) { \
  register void *_prev    = (prev); \
  register void *_next    = (next); \
  register int _threadoff = (int)&((struct task_struct *)0)->thread; \
  register char _shared   = ((prev)->mm == (next)->mm); \
  (last) = resume(_prev, _next, _threadoff, _shared); \
}

/* Reset the board */
#define HARD_RESET_NOW()

extern cregister volatile unsigned int IRP;     /* Interrupt Return Pointer */
extern cregister volatile unsigned int NRP;     /* NMI Return Pointer */
extern cregister volatile unsigned int CSR;     /* Control Status Register */
extern cregister volatile unsigned int IER;     /* Interrupt Enable Register */
extern cregister volatile unsigned int IFR;     /* Interrupt Flag Register */
extern cregister volatile unsigned int ISR;     /* Interrupt Set Register */
extern cregister volatile unsigned int ICR;     /* Interrupt Clear Register */
extern cregister volatile unsigned int ISTP;    /* Interrupt Service Table Pointer */
#ifdef __TMS320C6XPLUS__
extern cregister volatile unsigned int IERR;    /* Internal Exception Report Register */
extern cregister volatile unsigned int ECR;     /* Exception Clear Register */
extern cregister volatile unsigned int EFR;     /* Exception Flag Register */
extern cregister volatile unsigned int TSR;     /* Task State Register */
extern cregister volatile unsigned int ITSR;    /* Interrupt Task State Register */
extern cregister volatile unsigned int NTSR;    /* NMI/exception Task State Register */
extern cregister volatile unsigned int TSCL;    /* Time Stamp Counter Register - Low Half  */
extern cregister volatile unsigned int TSCH;    /* Time Stamp Counter Register - High Half */
extern cregister volatile unsigned int DNUM;    /* Core number */

#define get_coreid()             (DNUM & 0xff)

#endif

/* 
 * Interrupt management
 */

/* Return from interrupt function */
#define iret()                   { asm("	B.S2	IRP"); \
 		                   asm("	NOP	5"); }

/* Set/get IST */
#define set_IST(x)               ISTP = x
#define get_IST()                ISTP

/* Enable/disable interrupts */
extern unsigned int irq_IER;

#ifdef __TMS320C6XPLUS__
#define __dint()                 asm(" DINT")
#define __rint()                 asm(" RINT")
#endif
#define global_sti()             CSR |= 1
#define global_cli()             CSR &= -2
#define partial_sti()            do { CSR &= -2; IER |= irq_IER; CSR |= 1; } while(0)
#define partial_cli()            do { CSR &= -2; IER &= ~(irq_IER); CSR |= 1; } while(0)

#define enable_irq_mask(mask)    IER |= mask
#define disable_irq_mask(mask)   IER &= ~(mask)
#define set_irq_mask(mask)       ISR = IFR | (mask)
#define clear_irq_mask(mask)     ICR = IFR & ~(mask)
#define init_irq_mask()          IER = 2
#define clear_all_irq()          ICR = -1

#define save_global_flags(x)     x = CSR
#define restore_global_flags(x)  CSR = x
#define save_partial_flags(x)    x = IER
#define restore_partial_flags(x) IER = x

#ifdef CONFIG_NK
#define __cli()                  partial_cli()
#define __sti()                  partial_sti()
#define __save_flags(x)          save_partial_flags(x)
#define __restore_flags(x)       restore_partial_flags(x)
#else
#define __cli()                  global_cli()
#define __sti()                  global_sti()
#define __save_flags(x)          save_global_flags(x)
#define __restore_flags(x)       restore_global_flags(x)
#endif /* CONFIG_NK */

/* For spinlocks etc */
#define local_irq_save(x)	 do { __save_flags(x); __cli(); } while(0)
#define local_irq_restore(x)	 __restore_flags(x)
#define local_irq_disable()	 __cli()
#define local_irq_enable()	 __sti()

#define local_save_flags(x)      __save_flags(x)

#ifdef CONFIG_NK
#define	irqs_disabled()			\
({					\
	unsigned long flags;		\
	local_save_flags(flags);	\
	(!(flags & irq_IER));	        \
})
#else
#define	irqs_disabled()			\
({					\
	unsigned long flags;		\
	local_save_flags(flags);	\
	(!(flags & 0x1));	        \
})
#endif

/* 
 * Exception management
 */

#ifdef __TMS320C6XPLUS__
asmlinkage void enable_exception(void);
#define disable_exception()	 
#define get_except_type()        EFR
#define ack_exception(type)      (ECR = 1 << (type))
#define get_iexcept()            IERR
#define set_iexcept(mask)        (IERR = mask)
#else
#define enable_exception()
#define disable_exception()
#define get_except_type()	 
#define ack_exception(type)
#define get_iexcept()
#define set_iexcept(mask) 
#endif

/*
 * Misc. functions
 */

/* Return from exception function */
#define eret()                   { asm("	B.S2	NRP"); \
 		                   asm("	NOP	5"); }

#define nop()                    asm("	NOP");
#define mb()                     barrier()
#define rmb()                    barrier()
#define wmb()                    barrier()
#define set_mb(var, value)       do { var = value;  mb(); } while (0)
#define set_wmb(var, value)      do { var = value; wmb(); } while (0)

#define smp_mb()	         barrier()
#define smp_rmb()	         barrier()
#define smp_wmb()	         barrier()
#define smp_read_barrier_depends()	do { } while(0)

#define xchg(ptr,x) (__xchg((unsigned int)(x),(void *) (ptr),sizeof(*(ptr))))
#define tas(ptr)    (xchg((ptr),1))

unsigned int _lmbd(unsigned int, unsigned int);
unsigned int _bitr(unsigned int);

struct __xchg_dummy { unsigned int a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

static inline unsigned int __xchg(unsigned int x, volatile void * ptr, int size)
{
	unsigned int tmp, flags;

	local_irq_save(flags);

	switch (size) {
	case 1:
		tmp = 0;
		tmp = (unsigned char) *((unsigned char *) ptr);
		*((unsigned char *) ptr) = (unsigned char) x;
		break;
	case 2:
		tmp = 0;
		tmp = (unsigned short) *((unsigned short *) ptr);
		*((unsigned short *) ptr) = (unsigned short) x;
		break;
	case 4:
		tmp = 0;
		tmp = (unsigned int) *((unsigned int *) ptr);
		*((unsigned int *) ptr) = (unsigned int) x;
		break;
	}
	local_irq_restore(flags);
	return tmp;
}

#endif /* __ASM_C6X_SYSTEM_H */
