/*
 *  linux/arch/c6x/kernel/process.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This file handles the architecture-dependent parts of process handling.
 */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/user.h>
#include <linux/a.out.h>
#include <linux/reboot.h>
#include <linux/init_task.h>
#include <linux/mqueue.h>
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/tick.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/traps.h>
#include <asm/machdep.h>
#include <asm/setup.h>
#include <asm/pgtable.h>
#include <asm/pm.h>
#include <asm/string.h>

#include <mach/board.h>  /* for c6x_arch_idle_led() */

static struct signal_struct init_signals = INIT_SIGNALS(init_signals);
static struct sighand_struct init_sighand = INIT_SIGHAND(init_sighand);
/*
 * Initial thread structure.
 *
 * We need to make sure that this is THREAD_SIZE aligned due to the
 * way process stacks are handled. This is done by having a special
 * "init_task" linker map entry..
 */
union thread_union init_thread_union __init_task_data =
	{ INIT_THREAD_INFO(init_task) };

/*
 * Initial task structure.
 *
 * All other task structs will be allocated on slabs in fork.c
 */
struct task_struct init_task = INIT_TASK(init_task);
EXPORT_SYMBOL(init_task);

extern asmlinkage void ret_from_fork(void);
extern void remove_bkpt(void);

/*
 * power off function, if any
 */
void (*pm_power_off)(void);
EXPORT_SYMBOL(pm_power_off);

/*
 * power management idle function, if any..
 */
void (*pm_idle)(void);
EXPORT_SYMBOL(pm_idle);

static void default_idle(void)
{
#if defined(CONFIG_PM)
	pwrd_set(PWRD_PD1A);
#endif
#ifndef CONFIG_ARCH_SIM
	do_idle(); /* do nothing until interrupt */
#endif
}

void (*idle)(void) = default_idle;

/*
 * The idle loop for C64x
 */ 
void cpu_idle(void)
{
	/* endless idle loop with no priority at all */
	while (1) {
#ifdef CONFIG_TICK_ONESHOT
		tick_nohz_stop_sched_tick(1);
#endif
#ifdef CONFIG_IDLE_LED
		c6x_arch_idle_led(0);
#endif
		while (!need_resched()) {
			void (*idle)(void);

			smp_rmb();
			idle = pm_idle;
			if (!idle)
				idle = default_idle;
			idle();
		}
#ifdef CONFIG_IDLE_LED
		c6x_arch_idle_led(1);
#endif
#ifdef CONFIG_TICK_ONESHOT
		tick_nohz_restart_sched_tick();
#endif
		preempt_enable_no_resched();
		schedule();
		preempt_disable();
	}
}

asmlinkage int sys_idle(void)
{	
	if (current->pid != 0)
		return -EPERM;

	cpu_idle();

	return 0;
}

void machine_restart(char * __unused)
{
	for (;;);
}

void machine_halt(void)
{
	for (;;);
}

void machine_power_off(void)
{
	for (;;);
}

void show_regs(struct pt_regs * regs)
{
	printk("\n");
	printk("PC: %08lx SP: %08lx\n",
	       regs->pc, regs->sp);
	printk("Status: %08lx ORIG_A4: %08lx\n",
	       regs->csr, regs->orig_a4);
	printk("A0: %08lx  B0: %08lx\n",
	       regs->a0, regs->b0);
	printk("A1: %08lx  B1: %08lx\n",
	       regs->a1, regs->b1);
	printk("A2: %08lx  B2: %08lx\n",
	       regs->a2, regs->b2);
	printk("A3: %08lx  B3: %08lx\n",
	       regs->a3, regs->b3);
	printk("A4: %08lx  B4: %08lx\n",
	       regs->a4, regs->b4);
	printk("A5: %08lx  B5: %08lx\n",
	       regs->a5, regs->b5);
	printk("A6: %08lx  B6: %08lx\n",
	       regs->a6, regs->b6);
	printk("A7: %08lx  B7: %08lx\n",
	       regs->a7, regs->b7);
	printk("A8: %08lx  B8: %08lx\n",
	       regs->a8, regs->b8);
	printk("A9: %08lx  B9: %08lx\n",
	       regs->a9, regs->b9);
#if defined(CONFIG_TMS320C64X) || defined(_TMS320C6400)
	printk("A16: %08lx  B16: %08lx\n",
	       regs->a0, regs->b0);
	printk("A17: %08lx  B17: %08lx\n",
	       regs->a1, regs->b1);
	printk("A18: %08lx  B18: %08lx\n",
	       regs->a2, regs->b2);
	printk("A19: %08lx  B19: %08lx\n",
	       regs->a3, regs->b3);
	printk("A20: %08lx  B20: %08lx\n",
	       regs->a4, regs->b4);
	printk("A21: %08lx  B21: %08lx\n",
	       regs->a5, regs->b5);
	printk("A22: %08lx  B22: %08lx\n",
	       regs->a6, regs->b6);
	printk("A23: %08lx  B23: %08lx\n",
	       regs->a7, regs->b7);
	printk("A24: %08lx  B24: %08lx\n",
	       regs->a8, regs->b8);
	printk("A25: %08lx  B25: %08lx\n",
	       regs->a9, regs->b9);
	printk("A26: %08lx  B26: %08lx\n",
	       regs->a0, regs->b0);
	printk("A27: %08lx  B27: %08lx\n",
	       regs->a1, regs->b1);
	printk("A28: %08lx  B28: %08lx\n",
	       regs->a2, regs->b2);
	printk("A29: %08lx  B29: %08lx\n",
	       regs->a3, regs->b3);
	printk("A30: %08lx  B30: %08lx\n",
	       regs->a4, regs->b4);
	printk("A31: %08lx  B31: %08lx\n",
	       regs->a5, regs->b5);
#endif
}


static void kernel_thread_helper(int dummy, void *arg, int (*fn)(void *))
{
	do_exit(fn(arg));
}

/*
 * Create a kernel thread
 */
int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags)
{
	struct {
		struct switch_stack s;
		struct pt_regs      r;
	} regs;

	/*
	 * copy_thread sets a4 to zero (child return from fork)
	 * so we can't just set things up to directly return to
	 * fn.
	 */
	memset(&regs, 0, sizeof(regs));
	regs.r.b4 = (unsigned long) arg;
	regs.r.a6 = (unsigned long) fn;
	regs.r.pc = (unsigned long) kernel_thread_helper;
	local_save_flags(regs.r.csr);
	regs.r.csr |= 1;
#ifdef CONFIG_TMS320C64XPLUS
	regs.r.tsr = 5; /* Set GEE and GIE in TSR */
#endif

	/* Ok, create the new process.. */
	return do_fork(flags | CLONE_VM | CLONE_UNTRACED, -1, &regs.r, 0, NULL, NULL);
}
EXPORT_SYMBOL(kernel_thread);

/*
 * Free current thread data structures etc..
 */
void flush_thread(void)
{
	set_fs(USER_DS);
	current->thread.fs = __USER_DS;
}

void exit_thread(void)
{
	if (current->ptrace & PT_PTRACED)
		remove_bkpt();
}

/*
 * c6x_fork()
 */
asmlinkage int c6x_fork(struct pt_regs *regs)
{
#ifdef CONFIG_NO_FORK
	/* fork almost works, enough to trick you into looking elsewhere :-( */
	return(-EINVAL);
#else
	return do_fork(CLONE_VFORK | CLONE_VM | SIGCHLD, regs->sp, regs, 0, NULL, NULL);
#endif
}

asmlinkage int c6x_vfork(struct pt_regs *regs)
{
	return do_fork(CLONE_VFORK | CLONE_VM | SIGCHLD, regs->sp, regs, 0, NULL, NULL);
}

asmlinkage int c6x_clone(struct pt_regs *regs)
{
	unsigned long clone_flags;
	unsigned long newsp;

	/* syscall puts clone_flags in A4 and usp in B4 */
	clone_flags = regs->orig_a4;
	if (regs->b4)
		newsp = regs->b4;
	else
		newsp = regs->sp;

	return do_fork(clone_flags, newsp, regs, 0, (int __user *)regs->a6, (int __user *)regs->b6);
}

/*
 * Do necessary setup to start up a newly executed thread.
 */
void start_thread(struct pt_regs * regs, unsigned int pc, unsigned long usp)
{
	/*
	 * The binfmt loader will setup a "full" stack, but the C6X
	 * operates an "empty" stack. So we adjust the usp so that
	 * argc doesn't get destroyed if an interrupt is taken before
	 * it is read from the stack.
	 *
	 * NB: Library startup code needs to match this.
	 */
	usp -= 8;

	set_fs(USER_DS);
	regs->pc  = pc;
	regs->sp  = usp;
#ifdef CONFIG_TMS320C64XPLUS
	regs->tsr |= 0x40; /* set user mode */
#endif
	current->thread.usp = usp;
}

/*
 * Copy a new thread context in its stack.
 */
int copy_thread(unsigned long clone_flags, unsigned long usp,
		unsigned long ustk_size,
		struct task_struct * p, struct pt_regs * regs)
{
	struct pt_regs *childregs;
	struct switch_stack *childstack;

	childregs = task_pt_regs(p);
	childstack = (struct switch_stack*)childregs - 1;

	*childregs = *regs;
	childregs->a4 = 0;

	/* Set the return PC of the child */
	childstack->retpc = (unsigned int) ret_from_fork;

	if (usp == -1)
		/* case of  __kernel_thread: we return to supervisor space */
		childregs->sp = (unsigned long)(childregs + 1);
	else
		/* Otherwise use the given stack */
		childregs->sp = usp;

	/* Set usp/ksp */
	p->thread.usp = childregs->sp;
	p->thread.ksp = (unsigned long)childstack - 8;
	return 0;
}

/*
 * c6x_execve() executes a new program.
 */
extern int do_execve(char *, char **, char **, struct pt_regs *);

asmlinkage int c6x_execve(char *name, char **argv, char **envp, struct pt_regs *regs)
{
	int error;
	char * filename;

	filename = getname(name);
	error = PTR_ERR(filename);
	if (IS_ERR(filename))
		goto out;

	error = do_execve(filename, argv, envp, regs);
	putname(filename);
out:
	return error;
}

/*
 * These bracket the sleeping functions..
 */
extern void scheduling_functions_start_here(void);
extern void scheduling_functions_end_here(void);
#define first_sched	((unsigned long) scheduling_functions_start_here)
#define last_sched	((unsigned long) scheduling_functions_end_here)

unsigned long __kstk_eip(struct task_struct *p)
{
	return ((struct pt_regs *) (task_stack_page(p) + THREAD_SIZE))->pc;
}

unsigned long get_wchan(struct task_struct *p)
{
#if 0
	unsigned long fp, pc;
	unsigned long stack_page;
	int count = 0;

	if (!p || p == current || p->state == TASK_RUNNING)
		return 0;
 
	/* TO BE DONE */
#endif
	return 0;
}
