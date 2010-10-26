/*
 * This program is used to generate definitions needed by
 * assembly language modules.
 *
 * We use the technique used in the OSF Mach kernel code:
 * generate asm statements containing #defines,
 * compile this file to assembler, and then extract the
 * #defines from the assembly-language output.
 */

#include <linux/stddef.h>
#include <linux/sched.h>
#include <asm/thread_info.h>
#include <linux/kernel_stat.h>
#include <asm/page.h>
#include <asm/processor.h>

#define DEFINE(sym, val) unsigned long sym = val

int main(void)
{
	/* offsets into the task struct */
	DEFINE(TASK_STATE, offsetof(struct task_struct, state));

	/* offsets into the thread struct */
	DEFINE(THREAD_KSP, offsetof(struct thread_struct, ksp));

	/* offsets into the pt_regs struct */
	DEFINE(PT_REGS, offsetof(struct pt_regs, a4));

	/* offsets into the thread_info struct */
	DEFINE(PREEMPT_COUNT, offsetof(struct thread_info, preempt_count));

	return 0;
}
