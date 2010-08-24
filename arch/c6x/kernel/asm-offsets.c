/*
 * Generate definitions needed by assembly language modules.
 * This code generates raw asm output which is post-processed
 * to extract and format the required data.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <asm/thread_info.h>
#include <asm/procinfo.h>
#include <linux/kbuild.h>
#include <asm/unistd.h>

void foo(void)
{
#if defined(CONFIG_TMS320C64X) || defined(CONFIG_TMS320C64XPLUS) || defined(_TMS320C6400)
	OFFSET(REGS_A16,	pt_regs, a16);
	OFFSET(REGS_A17,	pt_regs, a17);
	OFFSET(REGS_A18,	pt_regs, a18);
	OFFSET(REGS_A19,	pt_regs, a19);
	OFFSET(REGS_A20,	pt_regs, a20);
	OFFSET(REGS_A21,	pt_regs, a21);
	OFFSET(REGS_A22,	pt_regs, a22);
	OFFSET(REGS_A23,	pt_regs, a23);
	OFFSET(REGS_A24,	pt_regs, a24);
	OFFSET(REGS_A25,	pt_regs, a25);
	OFFSET(REGS_A26,	pt_regs, a26);
	OFFSET(REGS_A27,	pt_regs, a27);
	OFFSET(REGS_A28,	pt_regs, a28);
	OFFSET(REGS_A29,	pt_regs, a29);
	OFFSET(REGS_A30,	pt_regs, a30);
	OFFSET(REGS_A31,	pt_regs, a31);

	OFFSET(REGS_B16,	pt_regs, b16);
	OFFSET(REGS_B17,	pt_regs, b17);
	OFFSET(REGS_B18,	pt_regs, b18);
	OFFSET(REGS_B19,	pt_regs, b19);
	OFFSET(REGS_B20,	pt_regs, b20);
	OFFSET(REGS_B21,	pt_regs, b21);
	OFFSET(REGS_B22,	pt_regs, b22);
	OFFSET(REGS_B23,	pt_regs, b23);
	OFFSET(REGS_B24,	pt_regs, b24);
	OFFSET(REGS_B25,	pt_regs, b25);
	OFFSET(REGS_B26,	pt_regs, b26);
	OFFSET(REGS_B27,	pt_regs, b27);
	OFFSET(REGS_B28,	pt_regs, b28);
	OFFSET(REGS_B29,	pt_regs, b29);
	OFFSET(REGS_B30,	pt_regs, b30);
	OFFSET(REGS_B31,	pt_regs, b31);
#endif
	OFFSET(REGS_A0,		pt_regs, a0);
	OFFSET(REGS_A1,		pt_regs, a1);
	OFFSET(REGS_A2,		pt_regs, a2);
	OFFSET(REGS_A3,		pt_regs, a3);
	OFFSET(REGS_A4,		pt_regs, a4);
	OFFSET(REGS_A5,		pt_regs, a5);
	OFFSET(REGS_A6,		pt_regs, a6);
	OFFSET(REGS_A7,		pt_regs, a7);
	OFFSET(REGS_A8,		pt_regs, a8);
	OFFSET(REGS_A9,		pt_regs, a9);
	OFFSET(REGS_B0,		pt_regs, b0);
	OFFSET(REGS_B1,		pt_regs, b1);
	OFFSET(REGS_B2,		pt_regs, b2);
	OFFSET(REGS_B3,		pt_regs, b3);
	OFFSET(REGS_B4,		pt_regs, b4);
	OFFSET(REGS_B5,		pt_regs, b5);
	OFFSET(REGS_B6,		pt_regs, b6);
	OFFSET(REGS_B7,		pt_regs, b7);
	OFFSET(REGS_B8,		pt_regs, b8);
	OFFSET(REGS_B9,		pt_regs, b9);

	OFFSET(REGS_TSR,	pt_regs, tsr);
	OFFSET(REGS_ORIG_A4,	pt_regs, orig_a4);

	DEFINE(REGS__END,	sizeof(struct pt_regs));
	BLANK();

	OFFSET(THREAD_KSP,	thread_struct, ksp);
	OFFSET(THREAD_CSR,	thread_struct, csr);
	BLANK();

	OFFSET(TASK_STATE,	task_struct, state);
	BLANK();

	OFFSET(THREAD_INFO_FLAGS,	thread_info, flags);
	BLANK();

	/* These would be unneccessary if we ran asm files
	 * through the preprocessor.
	 */
	DEFINE(KTHREAD_SIZE, THREAD_SIZE);
	DEFINE(KTHREAD_SHIFT, THREAD_SHIFT);
	DEFINE(KTHREAD_START_SP, THREAD_START_SP);
	DEFINE(ENOSYS_, ENOSYS);
	DEFINE(NR_SYSCALLS_, NR_SYSCALLS);

	DEFINE(_TIF_SYSCALL_TRACE, (1<<TIF_SYSCALL_TRACE));
	DEFINE(_TIF_NOTIFY_RESUME, (1<<TIF_NOTIFY_RESUME));
	DEFINE(_TIF_SIGPENDING, (1<<TIF_SIGPENDING));
	DEFINE(_TIF_NEED_RESCHED, (1<<TIF_NEED_RESCHED));
	DEFINE(_TIF_POLLING_NRFLAG, (1<<TIF_POLLING_NRFLAG));

	DEFINE(_TIF_ALLWORK_MASK, TIF_ALLWORK_MASK);
	DEFINE(_TIF_WORK_MASK, TIF_WORK_MASK);
}

