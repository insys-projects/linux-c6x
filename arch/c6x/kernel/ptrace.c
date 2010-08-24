/*
 *  linux/arch/c6x/kernel/ptrace.c
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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ptrace.h>
#include <linux/user.h>
#include <linux/smp_lock.h>
#include <linux/timer.h>
#include <linux/security.h>
#include <linux/tracehook.h>

#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>

/*
 * does not yet catch signals sent when the child dies.
 * in exit.c or in signal.c.
 */

/* Find the stack offset for a register */
#define PT_REG(reg)	  ((int)&((struct pt_regs *)0)->reg)
#define SW_REG(reg)	  ((int)&((struct switch_stack *)0)->reg \
			   - sizeof(struct switch_stack))

#define PT_REG_SIZE       (sizeof(struct pt_regs) + sizeof(struct switch_stack))

#ifdef _BIG_ENDIAN
#define PT_REGPAIR(odd,even) PT_REG(odd), PT_REG(even)
#define SW_REGPAIR(odd,even) SW_REG(odd), SW_REG(even)
#else
#define PT_REGPAIR(odd,even) PT_REG(even), PT_REG(odd)
#define SW_REGPAIR(odd,even) SW_REG(even), SW_REG(odd)
#endif

/* Mapping from PT_xxx to the stack offset at which the register is
   saved.  Notice that usp has no stack-slot and needs to be treated
   specially (see get_reg/put_reg below). */

static int regoff[] = {
#if defined(CONFIG_TMS320C64XPLUS) || defined(__TMS320C6XPLUS__)
	PT_REGPAIR(tsr,orig_a4),
#else
	PT_REGPAIR(stkadj,orig_a4),
#endif
	PT_REGPAIR(pc,csr),

#if defined CONFIG_TMS320C64X || defined CONFIG_TMS320C64XPLUS
	PT_REGPAIR(b17,b16), PT_REGPAIR(b19,b18), PT_REGPAIR(b21,b20), PT_REGPAIR(b23,b22),
	PT_REGPAIR(b25,b24), PT_REGPAIR(b27,b26), PT_REGPAIR(b29,b28), PT_REGPAIR(b31,b30),
#endif
	PT_REGPAIR(b1,b0), PT_REGPAIR(b3,b2), PT_REGPAIR(b5,b4), PT_REGPAIR(b7,b6),

#if defined CONFIG_TMS320C64X || defined CONFIG_TMS320C64XPLUS
	PT_REGPAIR(a17,a16), PT_REGPAIR(a19,a18), PT_REGPAIR(a21,a20), PT_REGPAIR(a23,a22),
	PT_REGPAIR(a25,a24), PT_REGPAIR(a27,a26), PT_REGPAIR(a29,a28), PT_REGPAIR(a31,a30),
#endif
	PT_REGPAIR(a1,a0), PT_REGPAIR(a3,a2), PT_REGPAIR(a5,a4), PT_REGPAIR(a7,a6),

	PT_REGPAIR(b9,b8), PT_REGPAIR(a9,a8),
	PT_REGPAIR(sp,dp),

#ifdef CONFIG_TMS320C64XPLUS
	SW_REGPAIR(ilc,rilc),
#endif
	SW_REGPAIR(a11,a10),
	SW_REGPAIR(a13,a12),
	SW_REGPAIR(a15,a14),
	SW_REGPAIR(b11,b10),
	SW_REGPAIR(b13,b12)
};

#define START_CODE_OFFSET  ((int)&((struct user *)0)->start_code)
#define START_STACK_OFFSET ((int)&((struct user *)0)->start_stack)
#define START_DATA_OFFSET  ((int)&((struct user *)0)->start_data)
#define END_CODE_OFFSET    ((int)&((struct user *)0)->end_code)
#define END_DATA_OFFSET    ((int)&((struct user *)0)->end_data)

#ifndef CONFIG_TMS320C64XPLUS
static struct timer_list   bkpt_timer;
static atomic_t            bkpt_timer_refc = 0;
#endif

/*
 * Get a register number from live pt_regs for the specified task.
 */
static inline long get_reg(struct task_struct *task, int regno)
{
	unsigned long *addr;
	struct pt_regs *regs = task_pt_regs(task);

	if (regno < sizeof(regoff)/sizeof(regoff[0]))
		addr = (unsigned long *)regs + regoff[regno];
	else
		return 0;
	return *addr;
}

#ifndef CONFIG_TMS320C64XPLUS
/*
 * Look up if there is a pending breakpoint
 */
static void check_bkpt(unsigned long dummy)
{
	unsigned *pc = (unsigned *) get_reg(current, PT_PC);
	unsigned opcode = BKPT_OPCODE | ((((unsigned) pc & 0x1f) >> 2) << 16);

	if ((*pc == opcode) && (current->ptrace & PT_PTRACED)) {
		siginfo_t info;

		info.si_signo = SIGTRAP;
		info.si_errno = 0;
		info.si_code  = TRAP_BRKPT;
		info.si_addr  = pc;
		
		force_sig_info(SIGTRAP, &info, current);
	}
	mod_timer(&bkpt_timer, jiffies + (HZ/10));
}
#endif

/*
 * Write contents of register REGNO in task TASK.
 */
static inline int put_reg(struct task_struct *task, 
			  int regno,
			  unsigned long data)
{
	unsigned long *addr;
	struct pt_regs *regs = task_pt_regs(task);

	if (regno < sizeof(regoff)/sizeof(regoff[0]))
		addr = (unsigned long *)regs + regoff[regno];
	else
		return -1;
	*addr = data;
	return 0;
}

static inline unsigned long get_long(struct task_struct *tsk, 
				     unsigned long addr)
{
	return *(unsigned long*)addr;
}

static inline int put_long(struct task_struct *tsk, 
			    unsigned long addr,
			    unsigned long data)
{
	*(unsigned long*) addr = data;
	return 0;
}

static inline int read_long(struct task_struct *tsk,
			    unsigned long addr,
			    unsigned long *result)
{
	*result = *(unsigned long *)addr;
	return 0;
}

static inline int write_long(struct task_struct *tsk,
			     unsigned long addr,
			     unsigned long data)
{
	*(unsigned long *) addr = data;
	return 0;
}

/*
 * Called by exit_thread when exiting.
 */
void remove_bkpt(void)
{
#ifndef CONFIG_TMS320C64XPLUS
	/* if we are the last traced thread, remove the timer */
	if (atomic_dec_and_test(&bkpt_timer_refc))
		del_timer(&bkpt_timer);
#endif
}

void activate_bkpt(void)
{
#ifndef CONFIG_TMS320C64XPLUS
	/* add breakpoint emulation */
	if (bkpt_timer_refc == 0) {
		init_timer(&bkpt_timer);
		bkpt_timer.function = check_bkpt;
		check_bkpt(0);
	}
	atomic_inc(&bkpt_timer_refc);
#endif
}

/*
 * Called by kernel/ptrace.c when detaching.
 */
void ptrace_disable(struct task_struct *child) 
{
	remove_bkpt();
}

/*
 * Perform ptrace request
 */
long arch_ptrace(struct task_struct *child, long request, long addr, long data)
{
	unsigned long tmp;
	int ret;

	switch (request) {
		/*
		 * read word at location "addr" in the child process.
		 */
	case PTRACE_PEEKTEXT:
	case PTRACE_PEEKDATA:
		ret = read_long(child, addr, &tmp);
		if (!ret)
			ret = put_user(tmp, (unsigned long *) data);
		break;
		
		/*
		 * read the word at location "addr" in the user registers.
		 */
	case PTRACE_PEEKUSR:
		ret = -EIO;
		if ((addr & 3) || addr < 0 || addr >= sizeof(struct user))
			break;
		
		tmp = 0;  /* Default return condition */
		if (addr < PT_REG_SIZE) {
			tmp = get_reg(child, (int)addr >> 2);
		} else if (addr == START_CODE_OFFSET) {
			tmp = child->mm->start_code;
		} else if (addr == START_STACK_OFFSET) {
			tmp = child->mm->start_stack;
		} else if (addr == START_DATA_OFFSET) {
			tmp = child->mm->start_data;
		} else if (addr == END_CODE_OFFSET) {
			tmp = child->mm->end_code;
		} else if (addr == END_DATA_OFFSET) {
			tmp = child->mm->end_data;
		}
		ret = put_user(tmp, (unsigned long *)data);
		break;

		/*
		 * write the word at location addr.
		 */
	case PTRACE_POKETEXT:
		ret = write_long(child, addr, data);
		flush_icache_range(addr, addr + 4);
		break;

	case PTRACE_POKEDATA:
		ret = write_long(child, addr, data);
		break;
		
		/*
		 * write the word at location addr in the user registers.
		 */
	case PTRACE_POKEUSR:
		ret = -EIO;
		if ((addr & 3) || addr < 0 || addr >= sizeof(struct user))
			break;
		
		if (addr < PT_REG_SIZE)
			ret = put_reg(child, (int)addr >> 2, data);
		break;

		/*
		 * continue/restart and stop at next (return from) syscall
		 */
	case PTRACE_SYSCALL:
	case PTRACE_CONT:
		ret = -EIO;
		if ((unsigned long) data > _NSIG)
			break;
		if (request == PTRACE_SYSCALL)
		        set_tsk_thread_flag(child, TIF_SYSCALL_TRACE);
		else
		        clear_tsk_thread_flag(child, TIF_SYSCALL_TRACE);
		child->exit_code = data;
		wake_up_process(child);
		ret = 0;
		break;
		
		/*
		 * make the child exit.  Best I can do is send it a sigkill. 
		 * perhaps it should be put in the status that it wants to 
		 * exit.
		 */
	case PTRACE_KILL:
		/* already dead */
		ret = 0;
		if (child->exit_state == EXIT_ZOMBIE) /* already dead */
			break;
		child->exit_code = SIGKILL;
		wake_up_process(child);
		ret = 0;
		break;

		/*
		 * execute single instruction.
		 */
	case PTRACE_SINGLESTEP:
		ret = -EIO;
		if ((unsigned long) data > _NSIG)
			break;
		clear_tsk_thread_flag(child, TIF_SYSCALL_TRACE);
		child->exit_code = data;
		wake_up_process(child);
		ret = 0;
		break;
		
		/*
		 * detach a process that was attached.
		 */
	case PTRACE_DETACH:
		ret = ptrace_detach(child, data);
		break;
		
		/*
		 * get all gp regs from the child.
		 */
	case PTRACE_GETREGS:
		struct pt_regs *regs = task_pt_regs(child);
		
		ret = 0;
		if (copy_to_user((void *)data, regs,
				 sizeof(struct pt_regs)))
			ret = -EFAULT;
		break;
		
		/*
		 * set all gp regs in the child.
		 */
	case PTRACE_SETREGS:
		struct pt_regs newregs;
		
		ret = -EFAULT;
		if (copy_from_user(&newregs, (void *)data,
				   sizeof(struct pt_regs)) == 0) {
			struct pt_regs *regs = task_pt_regs(child);
			*regs = newregs;
			ret = 0;
		}
		break;
		
	default:
		ret = -EIO;
		break;
	}

	return ret;
}

#if 0
int sys_ptrace(struct task_struct *child, long request, long addr, long data)
{
	struct task_struct *child;
	int ret;

	lock_kernel();
	ret = -EPERM;
	if (request == PTRACE_TRACEME) {
		/* are we already being traced? */
		if (current->ptrace & PT_PTRACED)
			goto out;
		ret = security_ptrace(current->parent, current);
		if (ret)
			goto out;
		/* set the ptrace bit in the process flags. */
		current->ptrace |= PT_PTRACED;
		activate_bkpt();
		ret = 0;
		goto out;
	}
	ret = -ESRCH;
	read_lock(&tasklist_lock);
	child = find_task_by_pid(pid);
	if (child)
		get_task_struct(child);
	read_unlock(&tasklist_lock);
	if (!child)
		goto out;

	ret = -EPERM;
	if (pid == 1)		/* you may not mess with init */
		goto out_tsk;

	if (request == PTRACE_ATTACH) {
		ret = ptrace_attach(child);
		if (ret == 0)
			activate_bkpt();	
		goto out_tsk;
	}
	ret = ptrace_check_attach(child, request == PTRACE_KILL);
	if (ret == 0)
		ret = do_ptrace(request, child, addr, data);

out_tsk:
	put_task_struct(child);
out:
	unlock_kernel();
	return ret;
}
#endif

/*
 * handle tracing of system call entry
 * - return the revised system call number or ULONG_MAX to cause ENOSYS
 */
asmlinkage unsigned long syscall_trace_entry(struct pt_regs *regs)
{
	if (tracehook_report_syscall_entry(regs))
		/* tracing decided this syscall should not happen, so
		 * We'll return a bogus call number to get an ENOSYS
		 * error, but leave the original number in
		 * regs->orig_a4
		 */
		return ULONG_MAX;

	return regs->b0;
}

/*
 * handle tracing of system call exit
 */
asmlinkage void syscall_trace_exit(struct pt_regs *regs)
{
	tracehook_report_syscall_exit(regs, 0);
}
