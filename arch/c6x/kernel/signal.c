/*
 *  linux/arch/c6x/kernel/signal.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  Updated for 2.6.34: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/ptrace.h>
#include <linux/unistd.h>
#include <linux/stddef.h>
#include <linux/highuid.h>
#include <linux/personality.h>
#include <linux/tracehook.h>

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/ucontext.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/traps.h>
#include <asm/cacheflush.h>

#define _BLOCKABLE (~(sigmask(SIGKILL) | sigmask(SIGSTOP)))

asmlinkage int sys_waitpid(pid_t pid,unsigned long * stat_addr, int options);

/*
 * atomically swap in the new signal mask, and wait for a signal.
 */
asmlinkage int do_sigsuspend(struct pt_regs *regs)
{
	old_sigset_t mask = regs->a6; /* third parameter of sigsuspend */

	mask &= _BLOCKABLE;
	spin_lock_irq(&current->sighand->siglock);
	current->saved_sigmask = current->blocked;
	siginitset(&current->blocked, mask);
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);

	current->state = TASK_INTERRUPTIBLE;
	schedule();
	set_thread_flag(TIF_RESTORE_SIGMASK);
	return -ERESTARTNOHAND;
}


asmlinkage int sys_sigaction(int sig,
			     const struct old_sigaction __user *act,
			     struct old_sigaction __user *oact)
{
	struct k_sigaction new_ka, old_ka;
	int ret;

	if (act) {
		old_sigset_t mask;
		if (!access_ok(VERIFY_READ, act, sizeof(*act)) ||
		    __get_user(new_ka.sa.sa_handler, &act->sa_handler) ||
		    __get_user(new_ka.sa.sa_restorer, &act->sa_restorer))
			return -EFAULT;
		__get_user(new_ka.sa.sa_flags, &act->sa_flags);
		__get_user(mask, &act->sa_mask);
		siginitset(&new_ka.sa.sa_mask, mask);
	}

	ret = do_sigaction(sig, act ? &new_ka : NULL, oact ? &old_ka : NULL);

	if (!ret && oact) {
		if (!access_ok(VERIFY_WRITE, oact, sizeof(*oact)) ||
		    __put_user(old_ka.sa.sa_handler, &oact->sa_handler) ||
		    __put_user(old_ka.sa.sa_restorer, &oact->sa_restorer))
			return -EFAULT;
		__put_user(old_ka.sa.sa_flags, &oact->sa_flags);
		__put_user(old_ka.sa.sa_mask.sig[0], &oact->sa_mask);
	}

	return ret;
}

asmlinkage int sys_sigaltstack(const stack_t *uss, stack_t *uoss)
{
	return do_sigaltstack(uss, uoss, current->thread.usp);
}

/*
 * Do a signal return, undo the signal stack.
 */

#define RETCODE_SIZE (9 << 2)   /* 9 instructions = 36 bytes */

struct sigframe
{
	struct sigcontext sc;
	unsigned long extramask[_NSIG_WORDS - 1];
	unsigned long retcode[RETCODE_SIZE >> 2];
};

struct rt_sigframe
{
	struct siginfo *pinfo;
	void *puc;
	struct siginfo info;
	struct ucontext uc;
	unsigned long retcode[RETCODE_SIZE >> 2];
};

static int restore_sigcontext(struct pt_regs *regs, struct sigcontext *sc)
{
	int err = 0;

#define COPY(x) err |= __get_user(regs->x, &sc->sc_##x)
	COPY(sp); COPY(a4); COPY(b4); COPY(a6); COPY(b6); COPY(a8); COPY(b8);
        COPY(a0); COPY(a1); COPY(a2); COPY(a3); COPY(a5); COPY(a7); COPY(a9);
	COPY(b0); COPY(b1); COPY(b2); COPY(b3); COPY(b5); COPY(b7); COPY(b9);

#if defined CONFIG_TMS320C64X || defined CONFIG_TMS320C64XPLUS
	COPY(a16); COPY(a17); COPY(a18); COPY(a19); COPY(a20); COPY(a21); COPY(a22); COPY(a23);
	COPY(a24); COPY(a25); COPY(a26); COPY(a27); COPY(a28); COPY(a29); COPY(a30); COPY(a31);
	COPY(b16); COPY(b17); COPY(b18); COPY(b19); COPY(b20); COPY(b21); COPY(b22); COPY(b23);
	COPY(b24); COPY(b25); COPY(b26); COPY(b27); COPY(b28); COPY(b29); COPY(b30); COPY(b31);
#endif
	COPY(csr); COPY(pc);

#undef COPY

	return err;
}

asmlinkage int do_sigreturn(struct pt_regs *regs)
{
	struct sigframe *frame;
	sigset_t set;

	/*
	 * Since we stacked the signal on a dword boundary,
	 * then 'sp' should be dword aligned here.  If it's
	 * not, then the user is trying to mess with us.
	 */
	if (regs->sp & 7)
		goto badframe;

	frame = (struct sigframe *) ((unsigned long) regs->sp + 8);

	if (!access_ok(VERIFY_READ, frame, sizeof (*frame)))
		goto badframe;
	if (__get_user(set.sig[0], &frame->sc.sc_mask))
		goto badframe;
	if (_NSIG_WORDS > 1 &&
	    __copy_from_user(&set.sig[1], &frame->extramask,
			     sizeof(frame->extramask)))
		goto badframe;

	sigdelsetmask(&set, ~_BLOCKABLE);
	spin_lock_irq(&current->sighand->siglock);
	current->blocked = set;
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);

	if (restore_sigcontext(regs, &frame->sc))
		goto badframe;
	
	return regs->a4;

badframe:
	force_sig(SIGSEGV, current);
	return 0;
}

asmlinkage int do_rt_sigreturn(struct pt_regs *regs)
{
	struct rt_sigframe *frame;
	sigset_t set;

	/*
	 * Since we stacked the signal on a dword boundary,
	 * then 'sp' should be dword aligned here.  If it's
	 * not, then the user is trying to mess with us.
	 */
	if (regs->sp & 7)
		goto badframe;

	frame = (struct rt_sigframe *) ((unsigned long) regs->sp + 8);

	if (!access_ok(VERIFY_READ, frame, sizeof (*frame)))
		goto badframe;
	if (__copy_from_user(&set, &frame->uc.uc_sigmask, sizeof(set)))
		goto badframe;

	sigdelsetmask(&set, ~_BLOCKABLE);
	spin_lock_irq(&current->sighand->siglock);
	current->blocked = set;
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);

	if (restore_sigcontext(regs, &frame->uc.uc_mcontext))
		goto badframe;
	
	return regs->a4;

badframe:
	force_sig(SIGSEGV, current);
	return 0;
}

static int
setup_sigcontext(struct sigcontext *sc, struct pt_regs *regs, unsigned long mask)
{
	int err = 0;

	err |= __put_user(mask, &sc->sc_mask);

#define COPY(x) err |= __put_user(regs->x, &sc->sc_##x)
	COPY(sp); COPY(a4); COPY(b4); COPY(a6); COPY(b6); COPY(a8); COPY(b8);
        COPY(a0); COPY(a1); COPY(a2); COPY(a3); COPY(a5); COPY(a7); COPY(a9);
	COPY(b0); COPY(b1); COPY(b2); COPY(b3); COPY(b5); COPY(b7); COPY(b9);

#if defined CONFIG_TMS320C64X || defined CONFIG_TMS320C64XPLUS
	COPY(a16); COPY(a17); COPY(a18); COPY(a19); COPY(a20); COPY(a21); COPY(a22); COPY(a23);
	COPY(a24); COPY(a25); COPY(a26); COPY(a27); COPY(a28); COPY(a29); COPY(a30); COPY(a31);
	COPY(b16); COPY(b17); COPY(b18); COPY(b19); COPY(b20); COPY(b21); COPY(b22); COPY(b23);
	COPY(b24); COPY(b25); COPY(b26); COPY(b27); COPY(b28); COPY(b29); COPY(b30); COPY(b31);
#endif
	COPY(csr); COPY(pc);

#undef COPY

	return err;
}

static inline void *get_sigframe(struct k_sigaction *ka,
				 struct pt_regs *regs,
				 unsigned long framesize)
{
	unsigned long sp = regs->sp;

	/*
	 * This is the X/Open sanctioned signal stack switching.
	 */
	if ((ka->sa.sa_flags & SA_ONSTACK) && sas_ss_flags(sp) == 0)
		sp = current->sas_ss_sp + current->sas_ss_size;

	/*
	 * No matter what happens, 'sp' must be dword
	 * aligned otherwise nasty things will happen
	 */
	return (void *)((sp - framesize) & ~7);
}

static int setup_frame(int signr, struct k_sigaction *ka,
		       sigset_t *set, struct pt_regs *regs)
{
	struct sigframe __user *frame;
	unsigned long *retcode;
	int err = 0;	

	frame = get_sigframe(ka, regs, sizeof(*frame));

	if (!access_ok(VERIFY_WRITE, frame, sizeof (*frame)))
		goto segv_and_exit;

	/* Set up the sigcontext */
	if (setup_sigcontext(&frame->sc, regs, set->sig[0]))
		goto segv_and_exit;

	/* Set up the extramask */
	if (_NSIG_WORDS > 1)
		if(__copy_to_user(frame->extramask, &set->sig[1],
				  sizeof(frame->extramask)))
			goto segv_and_exit;

	/* Set up to return from userspace. If provided, use a stub
	   already in userspace. */
	if (ka->sa.sa_flags & SA_RESTORER)
		retcode = (unsigned long *) ka->sa.sa_restorer;
	else {
		retcode = (unsigned long *) &frame->retcode;
		put_user(0x00003BAAUL, retcode++); /* MVK 119,B0 ; __NR_sigreturn in B0 */
#ifndef CONFIG_TMS320C64XPLUS
		put_user(0x010403E2UL, retcode++); /* MVC CSR,B2 */
		put_user(0x008800CAUL, retcode++); /* CLR B2,0,0,B1 */
		put_user(0x008403A2UL, retcode++); /* MVC B1,CSR */
		put_user(0x008803E2UL, retcode++); /* MVC IFR,B1 */
		put_user(0x0084C68AUL, retcode++); /* SET B1,6,6,B1 */
		put_user(0x010403A2UL, retcode++); /* MVC B1,ISR */
		put_user(0x008803A2UL, retcode++); /* MVC B2,CSR */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
#else /* CONFIG_TMS320C64XPLUS */
		put_user(0x10000000UL, retcode++); /* SWE */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
#endif /* CONFIG_TMS320C64XPLUS */
		flush_icache_range((unsigned long) &frame->retcode,
				   (unsigned long) &frame->retcode + RETCODE_SIZE);

		retcode = (unsigned long *) &frame->retcode;
	}

	if (err)
		goto segv_and_exit;

	/* Change user context to branch to signal handler */
	regs->sp = (unsigned long) frame - 8;
	regs->b3 = (unsigned long) retcode;
	regs->pc = (unsigned long) ka->sa.sa_handler;

	/* Give the signal number to the handler */
	regs->a4 = signr;
	regs->b4 = (unsigned long) &frame->sc;

	return 0;

segv_and_exit:
	force_sig(SIGSEGV, current);
	return -EFAULT;
}

static int setup_rt_frame(int signr, struct k_sigaction *ka, siginfo_t *info,
			   sigset_t *set, struct pt_regs *regs)
{
	struct rt_sigframe *frame;
	unsigned long *retcode;
	int err = 0;

	frame = get_sigframe(ka, regs, sizeof(*frame));

	if (!access_ok(VERIFY_WRITE, frame, sizeof (*frame)))
		goto segv_and_exit;

	err |= __put_user(&frame->info, &frame->pinfo);
	err |= __put_user(&frame->uc, &frame->puc);
	err |= copy_siginfo_to_user(&frame->info, info);

	/* Clear all the bits of the ucontext we don't use.  */
	err |= __clear_user(&frame->uc, offsetof(struct ucontext, uc_mcontext));
	
	err |= setup_sigcontext(&frame->uc.uc_mcontext,	regs, set->sig[0]);
	err |= __copy_to_user(&frame->uc.uc_sigmask, set, sizeof(*set));

	/* Set up to return from userspace.  If provided, use a stub
	   already in userspace.  */
	if (ka->sa.sa_flags & SA_RESTORER)
		retcode = (unsigned long *) ka->sa.sa_restorer;
	else {
		retcode = (unsigned long *) &frame->retcode;
		put_user(0x000056AAUL, retcode++); /* MVK 173,B0 ; __NR_rt_sigreturn in B0 */
#ifndef CONFIG_TMS320C64XPLUS
		put_user(0x010403E2UL, retcode++); /* MVC CSR,B2 */
		put_user(0x008800CAUL, retcode++); /* CLR B2,0,0,B1 */
		put_user(0x008403A2UL, retcode++); /* MVC B1,CSR */
		put_user(0x008803E2UL, retcode++); /* MVC IFR,B1 */
		put_user(0x0084C68AUL, retcode++); /* SET B1,6,6,B1 */
		put_user(0x010403A2UL, retcode++); /* MVC B1,ISR */
		put_user(0x008803A2UL, retcode++); /* MVC B2,CSR */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
#else /* CONFIG_TMS320C64XPLUS */
		put_user(0x10000000UL, retcode++); /* SWE */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
		put_user(0x00006000UL, retcode++); /* NOP 4 */
#endif /* CONFIG_TMS320C64XPLUS */
		flush_icache_range((unsigned long) &frame->retcode,
				   (unsigned long) &frame->retcode + RETCODE_SIZE);

		retcode = (unsigned long *) &frame->retcode;
	}

	if (err)
		goto segv_and_exit;

	/* Change user context to branch to signal handler */
	regs->sp = (unsigned long) frame - 8;
	regs->b3 = (unsigned long) retcode;
	regs->pc = (unsigned long) ka->sa.sa_handler;

	/* Give the signal number to the handler */
	regs->a4 = signr;

	/*
	 * For realtime signals we must also set the second and third
	 * arguments for the signal handler.
	 *   -- Peter Maydell <pmaydell@chiark.greenend.org.uk> 2000-12-06
	 */
	regs->b4 = (unsigned long)frame->pinfo;
	regs->a6 = (unsigned long)frame->puc;

	return 0;

segv_and_exit:
	force_sig(SIGSEGV, current);
	return -EFAULT;
}

static inline void
handle_restart(struct pt_regs *regs, struct k_sigaction *ka, int has_handler)
{
	switch (regs->a4) {
	case -ERESTARTNOHAND:
		if (!has_handler)
			goto do_restart;
		regs->a4 = -EINTR;
		break;

	case -ERESTARTSYS:
		if (has_handler && !(ka->sa.sa_flags & SA_RESTART)) {
			regs->a4 = -EINTR;
			break;
		}
	/* fallthrough */
	case -ERESTARTNOINTR:
	do_restart:
		regs->a4 = regs->orig_a4;
#ifdef CONFIG_TMS320C6X_SYSCALL_COMPAT
		if ((*((unsigned int *)(regs->pc - 12))  == 0x0084c68a) &&
		    (*((unsigned int *)(regs->pc - 8))  == 0x010403a2))
			regs->pc -= 32;
		else
			regs->pc -= SYSCALL_INT_SIZE;
#else
		regs->pc -= SYSCALL_INT_SIZE;
#endif
		break;
	}
}

/*
 * handle the actual delivery of a signal to userspace
 */
static int handle_signal(int sig,
			 siginfo_t *info, struct k_sigaction *ka,
			 sigset_t *oldset, struct pt_regs *regs,
			 int syscall)
{
	int ret;

	/* Are we from a system call? */
	if (syscall) {
		/* If so, check system call restarting.. */
		switch (regs->a4) {
		case -ERESTART_RESTARTBLOCK:
		case -ERESTARTNOHAND:
			regs->a4 = -EINTR;
			break;

		case -ERESTARTSYS:
			if (!(ka->sa.sa_flags & SA_RESTART)) {
				regs->a4 = -EINTR;
				break;
			}

			/* fallthrough */
		case -ERESTARTNOINTR:
			regs->a4 = regs->orig_a4;
			regs->pc -= 4;
		}
	}

	/* Set up the stack frame */
	if (ka->sa.sa_flags & SA_SIGINFO)
		ret = setup_rt_frame(sig, ka, info, oldset, regs);
	else
		ret = setup_frame(sig, ka, oldset, regs);

	if (ret == 0) {
		spin_lock_irq(&current->sighand->siglock);
		sigorsets(&current->blocked, &current->blocked,
			  &ka->sa.sa_mask);
		if (!(ka->sa.sa_flags & SA_NODEFER))
			sigaddset(&current->blocked, sig);
		recalc_sigpending();
		spin_unlock_irq(&current->sighand->siglock);
	}

	return ret;
}

/*
 * handle a potential signal
 */
static void do_signal(struct pt_regs *regs, int syscall)
{
	struct k_sigaction ka;
	siginfo_t info;
	sigset_t *oldset;
	int signr;

	/* we want the common case to go fast, which is why we may in certain
	 * cases get here from kernel mode */
	if (!user_mode(regs))
		return;

	if (test_thread_flag(TIF_RESTORE_SIGMASK))
		oldset = &current->saved_sigmask;
	else
		oldset = &current->blocked;

	signr = get_signal_to_deliver(&info, &ka, regs, NULL);
	if (signr > 0) {
		if (handle_signal(signr, &info, &ka, oldset, regs, syscall) == 0) {
			/* a signal was successfully delivered; the saved
			 * sigmask will have been stored in the signal frame,
			 * and will be restored by sigreturn, so we can simply
			 * clear the TIF_RESTORE_SIGMASK flag */
			if (test_thread_flag(TIF_RESTORE_SIGMASK))
				clear_thread_flag(TIF_RESTORE_SIGMASK);

			tracehook_signal_handler(signr, &info, &ka, regs,0);
		}

		return;
	}

	/* did we come from a system call? */
	if (syscall) {
		/* restart the system call - no handlers present */
		switch (regs->a4) {
		case -ERESTARTNOHAND:
		case -ERESTARTSYS:
		case -ERESTARTNOINTR:
			regs->a4 = regs->orig_a4;
			regs->pc -= 4;
			break;

		case -ERESTART_RESTARTBLOCK:
			regs->a4 = regs->orig_a4;
			regs->b0 = __NR_restart_syscall;
			regs->pc -= 4;
			break;
		}
	}

	/* if there's no signal to deliver, we just put the saved sigmask
	 * back */
	if (test_thread_flag(TIF_RESTORE_SIGMASK)) {
		clear_thread_flag(TIF_RESTORE_SIGMASK);
		sigprocmask(SIG_SETMASK, &current->saved_sigmask, NULL);
	}
}

/*
 * notification of userspace execution resumption
 * - triggered by current->work.notify_resume
 */
asmlinkage void do_notify_resume(struct pt_regs *regs, u32 thread_info_flags, int syscall)
{
	/* deal with pending signal delivery */
	if (thread_info_flags & ((1 << TIF_SIGPENDING) | (1 << TIF_RESTORE_SIGMASK)))
		do_signal(regs, syscall);

	if (thread_info_flags & (1 << TIF_NOTIFY_RESUME)) {
		clear_thread_flag(TIF_NOTIFY_RESUME);
		tracehook_notify_resume(regs);
		if (current->replacement_session_keyring)
			key_replace_session_keyring();
	}
}
