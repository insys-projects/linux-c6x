/*
 *  linux/include/asm-c6x/signal.h
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
#ifndef __ASM_C6X_SIGNAL_H
#define __ASM_C6X_SIGNAL_H

#include <linux/types.h>

#ifndef __ASSEMBLY__
/* Avoid too many header ordering problems.  */
struct siginfo;

#ifdef __KERNEL__
/* Most things should be clean enough to redefine this at will, if care
   is taken to make libc match.  */

#define _NSIG		64
#define _NSIG_BPW	32
#define _NSIG_WORDS	(_NSIG / _NSIG_BPW)

typedef unsigned long old_sigset_t;		/* at least 32 bits */

typedef struct {
	unsigned long sig[_NSIG_WORDS];
} sigset_t;

#else
/* Here we must cater to libcs that poke about in kernel headers.  */

#define NSIG		32
typedef unsigned long sigset_t;

#endif /* __KERNEL__ */
#endif /* __ASSEMBLY__ */

#define SIGHUP		 1
#define SIGINT		 2
#define SIGQUIT		 3
#define SIGILL		 4
#define SIGTRAP		 5
#define SIGABRT		 6
#define SIGIOT		 6
#define SIGBUS		 7
#define SIGFPE		 8
#define SIGKILL		 9
#define SIGUSR1		10
#define SIGSEGV		11
#define SIGUSR2		12
#define SIGPIPE		13
#define SIGALRM		14
#define SIGTERM		15
#define SIGSTKFLT	16
#define SIGCHLD		17
#define SIGCONT		18
#define SIGSTOP		19
#define SIGTSTP		20
#define SIGTTIN		21
#define SIGTTOU		22
#define SIGURG		23
#define SIGXCPU		24
#define SIGXFSZ		25
#define SIGVTALRM	26
#define SIGPROF		27
#define SIGWINCH	28
#define SIGIO		29
#define SIGPOLL		SIGIO
/*
#define SIGLOST		29
*/
#define SIGPWR		30
#define SIGSYS		31
#define	SIGUNUSED	31

/* These should not be considered constants from userland.  */
#define SIGRTMIN	32
#define SIGRTMAX	_NSIG

/*
 * SA_FLAGS values:
 *
 * SA_NOCLDSTOP		flag to turn off SIGCHLD when children stop.
 * SA_NOCLDWAIT		flag on SIGCHLD to inhibit zombies.
 * SA_SIGINFO		deliver the signal with SIGINFO structs
 * SA_THIRTYTWO		delivers the signal in 32-bit mode, even if the task 
 *			is running in 26-bit.
 * SA_ONSTACK		allows alternate signal stacks (see sigaltstack(2)).
 * SA_RESTART		flag to get restarting signals (which were the default long ago)
 * SA_INTERRUPT		is a no-op, but left due to historical reasons. Use the
 * SA_NODEFER		prevents the current signal from being masked in the handler.
 * SA_RESETHAND		clears the handler when the signal is delivered.
 *
 * SA_ONESHOT and SA_NOMASK are the historical Linux names for the Single
 * Unix names RESETHAND and NODEFER respectively.
 */
#define SA_NOCLDSTOP	0x00000001
#define SA_NOCLDWAIT	0x00000002 /* not supported yet */
#define SA_SIGINFO	0x00000004
#define SA_THIRTYTWO	0x02000000
#define SA_RESTORER	0x04000000
#define SA_ONSTACK	0x08000000
#define SA_RESTART	0x10000000
#define SA_NODEFER	0x40000000
#define SA_RESETHAND	0x80000000

#define SA_NOMASK	SA_NODEFER
#define SA_ONESHOT	SA_RESETHAND
#define SA_INTERRUPT	0x20000000 /* dummy -- ignored */

/* 
 * sigaltstack controls
 */
#define SS_ONSTACK	1
#define SS_DISABLE	2

#define MINSIGSTKSZ	2048
#define SIGSTKSZ	8192

#include <asm-generic/signal-defs.h>

#ifndef __ASSEMBLY__
#ifdef __KERNEL__
struct old_sigaction {
	__sighandler_t sa_handler;
	old_sigset_t sa_mask;
	unsigned long sa_flags;
	__sigrestore_t sa_restorer;
};

struct sigaction {
	__sighandler_t sa_handler;
	unsigned long sa_flags;
	__sigrestore_t sa_restorer;
	sigset_t sa_mask;		/* mask last for extensibility */
};

struct k_sigaction {
	struct sigaction sa;
};

#else
/* Here we must cater to libcs that poke about in kernel headers.  */

struct sigaction {
	union {
	  __sighandler_t _sa_handler;
	  void (*_sa_sigaction)(int, struct siginfo *, void *);
	} _u;
	sigset_t sa_mask;
	unsigned long sa_flags;
	void (*sa_restorer)(void);
};

#define sa_handler	_u._sa_handler
#define sa_sigaction	_u._sa_sigaction

#endif /* __KERNEL__ */

typedef struct sigaltstack {
	void *ss_sp;
	int ss_flags;
	size_t ss_size;
} stack_t;

#ifdef __KERNEL__
#include <asm/sigcontext.h>


struct pt_regs;
#define ptrace_signal_deliver(regs, cookie) do { } while (0)

#ifndef __GNU__
#define __HAVE_ARCH_SIG_SETOPS
#include <linux/string.h>

#if _NSIG_WORDS == 4
#define _SIG_SET_BINOP(name, op)					\
static inline void name(sigset_t *r, const sigset_t *a, const sigset_t *b) \
{									\
	unsigned long a0, a1, a2, a3, b0, b1, b2, b3;			\
	a3 = a->sig[3]; a2 = a->sig[2];					\
	b3 = b->sig[3]; b2 = b->sig[2];					\
	r->sig[3] = op(a3, b3);						\
	r->sig[2] = op(a2, b2);						\
	a1 = a->sig[1]; b1 = b->sig[1];					\
	r->sig[1] = op(a1, b1);						\
	a0 = a->sig[0]; b0 = b->sig[0];					\
	r->sig[0] = op(a0, b0);						\
}
#elif _NSIG_WORDS == 2
#define _SIG_SET_BINOP(name, op)					\
static inline void name(sigset_t *r, const sigset_t *a, const sigset_t *b) \
{									\
        unsigned long a0, a1, b0, b1;					\
									\
	a1 = a->sig[1]; b1 = b->sig[1];					\
	r->sig[1] = op(a1, b1);						\
	a0 = a->sig[0]; b0 = b->sig[0];					\
	r->sig[0] = op(a0, b0);						\
}
#elif _NSIG_WORDS == 1
static inline void name(sigset_t *r, const sigset_t *a, const sigset_t *b) \
{									\
	unsigned long a0, b0;						\
									\
	a0 = a->sig[0]; b0 = b->sig[0];					\
	r->sig[0] = op(a0, b0);						\
}
#else
#error "_NSIG_WORDS is unsupported size"
#endif


#define _sig_or(x,y)	((x) | (y))
_SIG_SET_BINOP(sigorsets, _sig_or)

#define _sig_and(x,y)	((x) & (y))
_SIG_SET_BINOP(sigandsets, _sig_and)

#define _sig_nand(x,y)	((x) & ~(y))
_SIG_SET_BINOP(signandsets, _sig_nand)

#undef _SIG_SET_BINOP
#undef _sig_or
#undef _sig_and
#undef _sig_nand

#if _NSIG_WORDS == 4
#define _SIG_SET_OP(name, op)				\
static inline void name(sigset_t *set)			\
{							\
	set->sig[3] = op(set->sig[3]);			\
	set->sig[2] = op(set->sig[2]);			\
	set->sig[1] = op(set->sig[1]);			\
	set->sig[0] = op(set->sig[0]);			\
}
#elif _NSIG_WORDS == 2
#define _SIG_SET_OP(name, op)				\
static inline void name(sigset_t *set)			\
{							\
	set->sig[1] = op(set->sig[1]);			\
	set->sig[0] = op(set->sig[0]);			\
}
#elif _NSIG_WORDS == 1
#define _SIG_SET_OP(name, op)				\
static inline void name(sigset_t *set)			\
{							\
	set->sig[0] = op(set->sig[0]);			\
}
#else
#error "_NSIG_WORDS is unsupported size"
#endif


#define _sig_not(x)	(~(x))
_SIG_SET_OP(signotset, _sig_not)

#undef _SIG_SET_OP
#undef _sig_not

static inline void sigemptyset(sigset_t *set)
{
#if _NSIG_WORDS == 4
	memset(set, 0, sizeof(sigset_t));
#elif _NSIG_WORDS == 2
	set->sig[1] = 0;
	set->sig[0] = 0;
#elif _NSIG_WORDS == 1
	set->sig[0] = 0;
#else
#error "_NSIG_WORDS is unsupported size"
#endif
}

static inline void sigfillset(sigset_t *set)
{
#if _NSIG_WORDS == 4
	memset(set, -1, sizeof(sigset_t));
#elif _NSIG_WORDS == 2
	set->sig[1] = -1;
	set->sig[0] = -1;
#elif _NSIG_WORDS == 1
	set->sig[0] = -1;
#else
#error "_NSIG_WORDS is unsupported size"
#endif
}

/* Some extensions for manipulating the low 32 signals in particular.  */

static inline void sigaddsetmask(sigset_t *set, unsigned long mask)
{
	set->sig[0] |= mask;
}

static inline void sigdelsetmask(sigset_t *set, unsigned long mask)
{
	set->sig[0] &= ~mask;
}

static inline int sigtestsetmask(sigset_t *set, unsigned long mask)
{
	return (set->sig[0] & mask) != 0;
}

static inline void siginitset(sigset_t *set, unsigned long mask)
{
	set->sig[0] = mask;
#if _NSIG_WORDS == 4
	memset(&set->sig[1], 0, sizeof(long)*(_NSIG_WORDS-1));
#elif _NSIG_WORDS == 2
	set->sig[1] = 0;
#elif _NSIG_WORDS != 1
#error "_NSIG_WORDS is unsupported size"
#endif
}

static inline void siginitsetinv(sigset_t *set, unsigned long mask)
{
	set->sig[0] = ~mask;
#if _NSIG_WORDS == 4
	memset(&set->sig[1], -1, sizeof(long)*(_NSIG_WORDS-1));
#elif _NSIG_WORDS == 2
	set->sig[1] = -1;
#elif _NSIG_WORDS != 1
#error "_NSIG_WORDS is unsupported size"
#endif
}
#endif /* __GNU__ */

#endif /* __KERNEL__ */
#endif /* __ASSEMBLY__ */
#endif /* __ASM_C6X_ */
