/*
 *  linux/arch/c6x/kernel/traps.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Sets up all exception vectors
 */
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/a.out.h>
#include <linux/user.h>
#include <linux/string.h>
#include <linux/linkage.h>
#include <linux/linkage.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/ptrace.h>

#include <asm/setup.h>
#include <asm/system.h>
#include <asm/segment.h>
#include <asm/traps.h>
#include <asm/pgtable.h>
#include <asm/machdep.h>
#include <asm/bitops.h>
#include <asm/irq.h>

void die_if_kernel(char *str, struct pt_regs *fp, int nr);

#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)
void unmask_eexception(int evt)
{
	__dint();
	INTC_EVTCLR[evt >> 5] = (1 << (evt & 0x1f));
	INTC_EXPMASK[evt >> 5] &= ~(1 << (evt & 0x1f));
	__rint();
}

void mask_eexception(int evt)
{
	__dint();
	INTC_EXPMASK[evt >> 5] |= (1 << (evt & 0x1f));
	__rint();

}
#endif

void __init trap_init (void)
{
#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)
	ack_exception(EXCEPT_TYPE_NXF);
	ack_exception(EXCEPT_TYPE_EXC);
	ack_exception(EXCEPT_TYPE_IXF);
	ack_exception(EXCEPT_TYPE_SXF);

	enable_exception();

	/* 
	 * External exceptions work with NMI, so activate NMI interrupt to enable them.
	 * When GEE is enabled, NMI are treated as exceptions instead of interrupts.
	 */
	enable_irq_mask(INT1);
#endif
}

void dump_stack(void)
{
	unsigned long stack;

	show_stack(current, &stack);
}
EXPORT_SYMBOL(dump_stack);


void die(char *str, struct pt_regs *fp, int nr)
{
	console_verbose();
	printk("%s: %08x\n",str,nr);
	show_regs(fp);

	if (*((unsigned long *) (PAGE_SIZE + (unsigned long) current)) != STACK_MAGIC)
		printk("Corrupted stack page\n");
	printk("Process %s (pid: %d, stackpage=%08lx)\n",
	       current->comm, current->pid, (PAGE_SIZE + (unsigned long) current));

	dump_stack();
	while (1);
}

void die_if_kernel(char *str, struct pt_regs *fp, int nr)
{
	if (user_mode(fp))
		return;

	die(str, fp ,nr);
}

#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)

/* Internal exceptions */
static struct exception_info iexcept_table[10] = {
	{ "Oops - instruction fetch", SIGBUS, BUS_ADRERR },
	{ "Oops - fetch packet", SIGBUS, BUS_ADRERR },  
	{ "Oops - execute packet", SIGILL, ILL_ILLOPC },
	{ "Oops - undefined instruction", SIGILL, ILL_ILLOPC },
	{ "Oops - resource conflict", SIGILL, ILL_ILLOPC },
	{ "Oops - resource access", SIGILL, ILL_PRVREG },
	{ "Oops - privilege", SIGILL, ILL_PRVOPC },
	{ "Oops - loops buffer", SIGILL, ILL_ILLOPC },
	{ "Oops - software exception", SIGILL, ILL_ILLTRP },
	{ "Oops - unknown exception", SIGILL, ILL_ILLOPC }
};

/* External exceptions */
static struct exception_info eexcept_table[128] = {
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },

	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },

	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },

	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - external exception", SIGBUS, BUS_ADRERR },
	{ "Oops - CPU memory protection fault", SIGSEGV, SEGV_ACCERR },
	{ "Oops - CPU memory protection fault in L1P", SIGSEGV, SEGV_ACCERR },
	{ "Oops - DMA memory protection fault in L1P", SIGSEGV, SEGV_ACCERR },
	{ "Oops - CPU memory protection fault in L1D", SIGSEGV, SEGV_ACCERR },
	{ "Oops - DMA memory protection fault in L1D", SIGSEGV, SEGV_ACCERR },
	{ "Oops - CPU memory protection fault in L2", SIGSEGV, SEGV_ACCERR },
	{ "Oops - DMA memory protection fault in L2", SIGSEGV, SEGV_ACCERR },
	{ "Oops - EMC CPU memory protection fault", SIGSEGV, SEGV_ACCERR },
	{ "Oops - EMC bus error", SIGBUS, BUS_ADRERR }
};

void do_trap(struct exception_info *except_info, struct pt_regs *regs)
{
	unsigned long addr = instruction_pointer(regs);
	siginfo_t info;

	if (except_info->signo != SIGTRAP)
		printk(KERN_DEBUG "Exception: %s PC[0x%lx] signo[%d] code[0x%x]\n",
		       except_info->kernel_str, regs->pc,
		       except_info->signo, except_info->code);

	die_if_kernel(except_info->kernel_str, regs, addr);

	info.si_signo = except_info->signo;
	info.si_errno = 0;
	info.si_code  = except_info->code;
	info.si_addr  = (void *)addr;

	force_sig_info(except_info->signo, &info, current);
}

/*
 * Process an internal exception (non maskable)
 */
static int process_iexcept(struct pt_regs *regs)
{
	unsigned int iexcept_report = get_iexcept();
	unsigned int iexcept_num;

	ack_exception(EXCEPT_TYPE_IXF);

	while(iexcept_report) {
		iexcept_num = __ffs(iexcept_report);
		iexcept_report &= ~(1 << iexcept_num);
		set_iexcept(iexcept_report);
#ifdef CONFIG_TMS320C6X_SYSCALL_COMPAT
		if (((iexcept_num == 6) || (iexcept_num == 5)) && 
		    (*((unsigned int *)(regs->pc + 4))  == 0x0084c68a) &&
		    (*((unsigned int *)(regs->pc))      == 0x008803e2) &&
		    (*((unsigned int *)(regs->pc - 4))  == 0x008403a2) &&
		    (*((unsigned int *)(regs->pc - 8))  == 0x008800ca) &&
		    (*((unsigned int *)(regs->pc - 12)) == 0x010403e2) &&
		    (*((unsigned int *)(regs->pc + 8))  == 0x010403a2) &&
		    (*((unsigned int *)(regs->pc + 12)) == 0x008803a2) &&
		    (*((unsigned int *)(regs->pc + 16)) == 0x00000000)) {

			/* skip syscall user stub */
			regs->pc += 16;

			/* return with unmasked interrupts */
			regs->csr |= 0x2;
			regs->tsr |= 0x1;

			local_irq_enable();
			return 1;
		}
#endif
		if (*(unsigned int *)regs->pc == BKPT_OPCODE) {
			/* This is a breakpoint */
			struct exception_info bkpt_exception = { "Oops - undefined instruction", SIGTRAP, TRAP_BRKPT };
			do_trap(&bkpt_exception, regs);
			iexcept_report &= ~(0xFF);
			set_iexcept(iexcept_report);
			continue;
		}

		do_trap(&iexcept_table[iexcept_num], regs);		
	}
	return 0;
}

/*
 * Process an external exception (maskable)
 */
static void process_eexcept(struct pt_regs *regs)
{
	unsigned int eexcept_num;
	int i;

	ack_exception(EXCEPT_TYPE_EXC);

	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		while (INTC_MEXPMASK[i]) {

			__dint();
			eexcept_num = __ffs(INTC_MEXPMASK[i]);
			INTC_EVTCLR[i] = (1 << eexcept_num); /* ack the external exception */
			__rint();

			/* When IB or SPLX is set, exceptions cannot be resumed correctly */
			if (regs->tsr & 0xc000) {
				printk(KERN_CRIT "External exception not recoverable (NTSR=0x%x)!!!\n",
				       (unsigned int) regs->tsr);
			}

			do_trap(&eexcept_table[eexcept_num + (i << 5)], regs);
		}
	}
	ack_exception(EXCEPT_TYPE_EXC);
}

/*
 * Main exception processing
 */
asmlinkage int process_exception(struct pt_regs *regs)
{
	unsigned int type;
	unsigned int type_num;
	unsigned int ie_num = 9; /* default is unknown exception */

	while ((type = get_except_type()) != 0) {
		type_num = fls(type) - 1;

		switch(type_num) {
		case EXCEPT_TYPE_NXF:
			ack_exception(EXCEPT_TYPE_NXF);
			if (mach_nmi_handler)
				(*mach_nmi_handler)(regs);
			else
				die("Oops - NMI detected", regs, instruction_pointer(regs));
			break;

		case EXCEPT_TYPE_IXF:
			if (process_iexcept(regs))
				return 1;
			break;

		case EXCEPT_TYPE_EXC:
			process_eexcept(regs);
			break;

		case EXCEPT_TYPE_SXF:
			ie_num = 8;
		default:
			ack_exception(type_num);
			do_trap(&iexcept_table[ie_num], regs);
			break;
		}
	}
	return 0;
}

#endif /* CONFIG_TMS320C64XPLUS || CONFIG_TMS320C66X */

int kstack_depth_to_print = 48;

static void show_trace(unsigned long *stack, unsigned long *endstack)
{
	unsigned long addr;
	int i;

	printk("Call trace:");
	i = 0;
	while (stack + 1 <= endstack) {
		addr = *stack++;
		/*
		 * If the address is either in the text segment of the
		 * kernel, or in the region which contains vmalloc'ed
		 * memory, it *may* be the address of a calling
		 * routine; if so, print it so that someone tracing
		 * down the cause of the crash will be able to figure
		 * out the call path that was taken.
		 */
		if (__kernel_text_address(addr)) {
#ifndef CONFIG_KALLSYMS
			if (i % 5 == 0)
				printk("\n       ");
#endif
			printk(" [<%08lx>]", addr);
			print_symbol(" %s\n", addr);
			i++;
		}
	}
	printk("\n");
}

void show_stack(struct task_struct *task, unsigned long *stack)
{
	unsigned long *p, *endstack;
	int i;

	if (!stack) {
		if (task)
			/* We know this is a kernel stack, so this is the start/end */
			stack = (unsigned long *)task->thread.ksp;
		else
			stack = (unsigned long *)&stack;
	}
	endstack = (unsigned long *)(((unsigned long)stack + THREAD_SIZE - 1) & -THREAD_SIZE);

	printk("Stack from %08lx:", (unsigned long)stack);
	for (i = 0, p = stack; i < kstack_depth_to_print; i++) {
		if (p + 1 > endstack)
			break;
		if (i % 8 == 0)
			printk("\n       ");
		printk(" %08lx", *p++);
	}
	printk("\n");
	show_trace(stack, endstack);
}

int is_valid_bugaddr(unsigned long addr)
{
	return __kernel_text_address(addr);
}

