;
;  linux/arch/c6x/kernel/entry.s
;
;  Port on Texas Instruments TMS320C6x architecture
;
;  Copyright (C) 2004, 2005, 2006, 2009, 2010, 2011 Texas Instruments Incorporated
;  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
;  Updated for 2.6.34: Mark Salter <msalter@redhat.com>
;
;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License version 2 as
;  published by the Free Software Foundation.
;

#ifdef CONFIG_TI_C6X_COMPILER
#define _DEFINE_MACRO(name,args...) name .macro args

#define THREAD_SHIFT	  KTHREAD_SHIFT
#define THREAD_START_SP	  KTHREAD_START_SP

		.include	"c6x_defs.inc"
		.include	"c6x_config.inc"
#else
	.altmacro
#define _DEFINE_MACRO(name,args...) .macro name args

#include <linux/sys.h>
#include <linux/linkage.h>
#include <asm/thread_info.h>
#include <asm/asm-offsets.h>
#endif

; Registers naming
#define DP	B14
#define FP	A15
#define SP	B15

#if defined(CONFIG_TMS320C64X) || defined(CONFIG_TMS320C64XPLUS)
#define HAVE_32_REGS 1
#endif

#ifndef CONFIG_PREEMPT
#define resume_kernel restore_all
#endif

	;; Entry management functions
		.global	resume
		.global	ret_from_exception
		.global	ret_from_interrupt

	;; Interrupt handlers
		.global	_int4_handler
		.global	_int5_handler
		.global	_int7_handler
		.global	_int8_handler
		.global	_int9_handler
		.global	_int10_handler
		.global	_int11_handler
		.global	_int12_handler
		.global	_int13_handler
		.global	_int14_handler
		.global	_int15_handler
		.global	_bad_interrupt
		.global	_nmi_handler

#ifdef CONFIG_TI_C6X_COMPILER
	;; Syscalls
		.def 	sys_fork
		.def	sys_clone
		.def 	sys_execve

	;; External functions
		.ref	schedule
		.ref	schedule_tail
		.ref	syscall_trace_entry
		.ref	syscall_trace_exit
		.ref	send_sig
		.ref	do_notify_resume
		.ref	c6x_do_IRQ
		.ref 	c6x_fork
		.ref	c6x_clone
		.ref	c6x_vfork
		.ref	c6x_execve
		.ref 	do_sigsuspend
		.ref	do_sigreturn
		.ref	do_rt_sigreturn
		.ref	sys_rt_sigsuspend
		.ref	exit
		.ref	do_exit
		.ref	init_thread_union
		.ref	sys_pread64
		.ref	sys_pwrite64
		.ref	sys_truncate64
		.ref	sys_ftruncate64
		.ref	sys_advise64
		.ref	sys_advise64_64
		.ref	sys_fallocate
		.ref	process_exception
		.ref   sys_ni_syscall
#ifdef CONFIG_PREEMPT
		.ref	preempt_schedule_irq
#endif

	;; External variables
		.ref	irq_stat
		.ref	_edata

	;; Internal variables
		.def	_sys_call_table

#endif  /* CONFIG_TI_C6X_COMPILER */

	;;
	;;  This defines the normal kernel pt_regs layout.
	;;
_DEFINE_MACRO(SAVE_ALL,__rp,__tsr)
	STW	.D2T2	B0,*SP--[2]		; save original B0
	MVKL	.S2	current_ksp,B0
	MVKH	.S2	current_ksp,B0
	LDW	.D2T2	*B0,B1			; KSP

	NOP	3
	STW	.D2T2	B1,*+SP[1]		; save original B1
	XOR	.D2	SP,B1,B0		; (SP ^ KSP)
	LDW	.D2T2	*+SP[1],B1		; restore B0/B1
	LDW	.D2T2	*++SP[2],B0
	SHR	.S2	B0,THREAD_SHIFT,B0	; 0 if already using kernel stack

  [B0]	STDW	.D2T2	SP:DP,*--B1[1]		; user: save user sp/dp on kernel stack
  [B0]	MV	.S2	B1,SP			;    and switch to kernel stack
||[!B0] STDW	.D2T2	SP:DP,*--SP[1]		; kernel: save on current stack

	SUBAW	.D2	SP,2,SP

#ifdef HAVE_32_REGS
	ADD	.D1X	SP,-8,A15
 ||	STDW	.D2T1	A15:A14,*SP--[16]	; save A15:A14
#else
	ADD	.D1X	SP,-8,A15
 ||	STDW	.D2T1	A15:A14,*SP--[8]	; save A15:A14
#endif

	STDW	.D2T2	B13:B12,*SP--[1]
 ||	STDW	.D1T1	A13:A12,*A15--[1]
 ||	MVC	.S2	__rp,B13

	STDW	.D2T2	B11:B10,*SP--[1]
 ||	STDW	.D1T1	A11:A10,*A15--[1]
 ||	MVC	.S2	CSR,B12

#ifdef CONFIG_TMS320C64XPLUS
	STDW	.D2T2	B9:B8,*SP--[1]
 ||	STDW	.D1T1	A9:A8,*A15--[1]
 ||	MVC	.S2	RILC,B11
	STDW	.D2T2	B7:B6,*SP--[1]
 ||	STDW	.D1T1	A7:A6,*A15--[1]
 ||	MVC	.S2	ILC,B10
#else
	STDW	.D2T2	B9:B8,*SP--[1]
 ||	STDW	.D1T1	A9:A8,*A15--[1]
	STDW	.D2T2	B7:B6,*SP--[1]
 ||	STDW	.D1T1	A7:A6,*A15--[1]
#endif

	STDW	.D2T2	B5:B4,*SP--[1]
 ||	STDW	.D1T1	A5:A4,*A15--[1]

	STDW	.D2T2	B3:B2,*SP--[1]
 ||	STDW	.D1T1	A3:A2,*A15--[1]
 ||	MVC	.S2	__tsr,B5

	STDW	.D2T2	B1:B0,*SP--[1]
 ||	STDW	.D1T1	A1:A0,*A15--[1]
 ||	MV	.S1X	B5,A5

#ifdef HAVE_32_REGS
	STDW	.D2T2	B31:B30,*SP--[1]
 ||	STDW	.D1T1	A31:A30,*A15--[1]
	STDW	.D2T2	B29:B28,*SP--[1]
 ||	STDW	.D1T1	A29:A28,*A15--[1]
	STDW	.D2T2	B27:B26,*SP--[1]
 ||	STDW	.D1T1	A27:A26,*A15--[1]
	STDW	.D2T2	B25:B24,*SP--[1]
 ||	STDW	.D1T1	A25:A24,*A15--[1]
	STDW	.D2T2	B23:B22,*SP--[1]
 ||	STDW	.D1T1	A23:A22,*A15--[1]
	STDW	.D2T2	B21:B20,*SP--[1]
 ||	STDW	.D1T1	A21:A20,*A15--[1]
	STDW	.D2T2	B19:B18,*SP--[1]
 ||	STDW	.D1T1	A19:A18,*A15--[1]
	STDW	.D2T2	B17:B16,*SP--[1]
 ||	STDW	.D1T1	A17:A16,*A15--[1]
#endif

	STDW	.D2T2	B13:B12,*SP--[1]	; save PC and CSR

#ifndef CONFIG_TMS320C64XPLUS
	STW	.D2T2	A4,*SP--[2]		; orig A4
#else
	STDW	.D2T2	B11:B10,*SP--[1]	; save RILC and ILC
	STDW	.D2T1	A5:A4,*SP--[1]		; save TSR and orig A4
#endif

	;; We left an unused word on the stack just above pt_regs.
	;; It is used to save whether or not this frame is due to
	;; a syscall. It is cleared here, but the syscall handler
	;; sets it to a non-zero value.
	MVK	.L2	0,B1
	STW	.D2T2	B1,*+SP(REGS__END+8)	; clear syscall flag
	.endm

_DEFINE_MACRO(SAVE_ALL_EXCEPT)
#ifndef CONFIG_TMS320C64XPLUS
	SAVE_ALL IRP
#else
	SAVE_ALL NRP,NTSR
#endif
	.endm

_DEFINE_MACRO(SAVE_ALL_INT)
#ifndef CONFIG_TMS320C64XPLUS
	SAVE_ALL IRP
#else
	SAVE_ALL IRP,ITSR
#endif
	.endm

_DEFINE_MACRO(RESTORE_ALL,__rp,__tsr)

#ifdef CONFIG_TMS320C64XPLUS
	LDDW	.D2T2	*++SP[1],B9:B8		; get TSR (B9)
	LDDW	.D2T2	*++SP[1],B11:B10	; get RILC (B11) and ILC (B10)
	LDDW	.D2T2	*++SP[1],B13:B12	; get PC (B13) and CSR (B12)
#else
	LDDW	.D2T2	*++SP[2],B13:B12	; get PC (B13) and CSR (B12)
#endif

#ifdef HAVE_32_REGS
	ADDAW	.D1X	SP,30,A15
#else
	ADDAW	.D1X	SP,14,A15
#endif

#ifdef HAVE_32_REGS
	LDDW	.D1T1	*++A15[1],A17:A16
 ||	LDDW	.D2T2	*++SP[1],B17:B16
	LDDW	.D1T1	*++A15[1],A19:A18
 ||	LDDW	.D2T2	*++SP[1],B19:B18
	LDDW	.D1T1	*++A15[1],A21:A20
 ||	LDDW	.D2T2	*++SP[1],B21:B20
	LDDW	.D1T1	*++A15[1],A23:A22
 ||	LDDW	.D2T2	*++SP[1],B23:B22
	LDDW	.D1T1	*++A15[1],A25:A24
 ||	LDDW	.D2T2	*++SP[1],B25:B24
	LDDW	.D1T1	*++A15[1],A27:A26
 ||	LDDW	.D2T2	*++SP[1],B27:B26
	LDDW	.D1T1	*++A15[1],A29:A28
 ||	LDDW	.D2T2	*++SP[1],B29:B28
	LDDW	.D1T1	*++A15[1],A31:A30
 ||	LDDW	.D2T2	*++SP[1],B31:B30
#endif

	LDDW	.D1T1	*++A15[1],A1:A0
 ||	LDDW	.D2T2	*++SP[1],B1:B0

#ifdef HAVE_32_REGS
	LDDW	.D1T1	*++A15[1],A3:A2
 ||	LDDW	.D2T2	*++SP[1],B3:B2
 ||	MVC	.S2	B9,__tsr
	LDDW	.D1T1	*++A15[1],A5:A4
 ||	LDDW	.D2T2	*++SP[1],B5:B4
 ||	MVC	.S2	B11,RILC
	LDDW	.D1T1	*++A15[1],A7:A6
 ||	LDDW	.D2T2	*++SP[1],B7:B6
 ||	MVC	.S2	B10,ILC
#else
	LDDW	.D1T1	*++A15[1],A3:A2
 ||	LDDW	.D2T2	*++SP[1],B3:B2
	LDDW	.D1T1	*++A15[1],A5:A4
 ||	LDDW	.D2T2	*++SP[1],B5:B4
	LDDW	.D1T1	*++A15[1],A7:A6
 ||	LDDW	.D2T2	*++SP[1],B7:B6
#endif

	LDDW	.D1T1	*++A15[1],A9:A8
 ||	LDDW	.D2T2	*++SP[1],B9:B8
 ||	MVC	.S2	B13,__rp

	LDDW	.D1T1	*++A15[1],A11:A10
 ||	LDDW	.D2T2	*++SP[1],B11:B10
 ||	MVC	.S2	B12,CSR

	LDDW	.D1T1	*++A15[1],A13:A12
 ||	LDDW	.D2T2	*++SP[1],B13:B12

	MV	.D2X	A15,SP
 ||	MVKL	.S1	current_ksp,A15
	MVKH	.S1	current_ksp,A15
 ||	ADDAW	.D1X	SP,6,A14
	STW	.D1T1	A14,*A15	; save kernel stack pointer

	LDDW	.D2T1	*++SP[1],A15:A14

	B	.S2	__rp		; return from interruption
	LDDW	.D2T2	*+SP[1],SP:DP
	NOP	4
	.endm

_DEFINE_MACRO(RESTORE_ALL_EXCEPT)
#ifndef CONFIG_TMS320C64XPLUS
	RESTORE_ALL IRP
#else
	RESTORE_ALL NRP,NTSR
#endif
	.endm

_DEFINE_MACRO(RESTORE_ALL_INT)
#ifndef CONFIG_TMS320C64XPLUS
	RESTORE_ALL IRP
#else
	RESTORE_ALL IRP,ITSR
#endif
	.endm

_DEFINE_MACRO(MASK_SYSCALL)
#ifndef CONFIG_TMS320C64XPLUS
	MVC	.S2	IER,B1
	CLR	.S2	B1,INT_SYSCALL,INT_SYSCALL,B1
	MVC	.S2	B1,IER
#endif
	.endm

_DEFINE_MACRO(UNMASK_SYSCALL)
#ifndef CONFIG_TMS320C64XPLUS
	MVC	.S2	IER,B1
	SET	.S2	B1,INT_SYSCALL,INT_SYSCALL,B1
	MVC	.S2	B1,IER
#endif
	.endm

_DEFINE_MACRO(MASK_INT,reg)
	MVC	.S2	CSR,reg
	CLR	.S2	reg,0,0,reg
	MVC	.S2	reg,CSR
	.endm

	;; Enable interrupts
_DEFINE_MACRO(UNMASK_INT,reg)
	MVC	.S2	CSR,reg
	SET	.S2	reg,0,0,reg
	MVC	.S2	reg,CSR
	.endm

#ifndef CONFIG_TMS320C64XPLUS
#define SWITCH_STACK_SIZE (12*4+8)
#else
#define SWITCH_STACK_SIZE (14*4+8)
#endif

	;;
	;; Save remaining registers preserved by C
	;; B3 must contain the return addresse
	;;
_DEFINE_MACRO(SAVE_SWITCH_STACK)
#ifndef CONFIG_TMS320C64XPLUS
	STW	.D2T2	B3,*SP--[12]		; push return address
	STDW	.D2T2	B13:B12,*+SP[5]
	STDW	.D2T2	B11:B10,*+SP[4]
	STDW	.D2T1	A15:A14,*+SP[3]
	STDW	.D2T1	A13:A12,*+SP[2]
	STDW	.D2T1	A11:A10,*+SP[1]
#else
	STW	.D2T2	B3,*SP--[14]		; push return address
	STDW	.D2T2	B13:B12,*+SP[6]
	STDW	.D2T2	B11:B10,*+SP[5]
	STDW	.D2T1	A15:A14,*+SP[4]
	STDW	.D2T1	A13:A12,*+SP[3]
 ||	MVC	.S2	RILC,B11
	STDW	.D2T1	A11:A10,*+SP[2]
 ||	MVC	.S2	ILC,B10
	STDW	.D2T2	B11:B10,*+SP[1]
#endif
	.endm

_DEFINE_MACRO(RESTORE_SWITCH_STACK)
#ifndef CONFIG_TMS320C64XPLUS
	LDDW	.D2T1	*+SP[1],A11:A10
	LDDW	.D2T1	*+SP[2],A13:A12
	LDDW	.D2T1	*+SP[3],A15:A14
	LDDW	.D2T2	*+SP[4],B11:B10
	LDDW	.D2T2	*+SP[5],B13:B12
	LDW	.D2T2	*++SP[12],B3		; pop return address
#else
	LDDW	.D2T2	*+SP[1],B11:B10		; load B11 (RILC ) and B10 (ILC)

	LDDW	.D2T1	*+SP[2],A11:A10
	LDDW	.D2T1	*+SP[3],A13:A12
	LDDW	.D2T1	*+SP[4],A15:A14
	LDDW	.D2T2	*+SP[5],B11:B10

	LDDW	.D2T2	*+SP[6],B13:B12
 ||	MVC	.S2	B10,ILC

	LDW	.D2T2	*++SP[14],B3		; pop return address
 ||	MVC	.S2	B11,RILC
#endif
	NOP	4
	.endm

	;;
	;; Call c6x_do_IRQ with the corresponding int number and regs
	;;
_DEFINE_MACRO(CALL_INT,int)
	MVK	.S1	int,A4
 ||	ADDAW	.D2	SP,2,B4
	MVKL	.S1	c6x_do_IRQ,A0
	MVKH	.S1	c6x_do_IRQ,A0
	BNOP	.S2X	A0,3
	MVKL	.S2	ret_from_interrupt,B3
	MVKH	.S2	ret_from_interrupt,B3
	.endm

_DEFINE_MACRO(GET_THREAD_INFO,reg)
	SHR	.S1X	SP,THREAD_SHIFT,reg
	SHL	.S1	reg,THREAD_SHIFT,reg
	.endm

	.sect 	".data"

	.global	current_ksp
current_ksp:
	.word	init_thread_union + THREAD_START_SP

	.sect 	".text"

	;;
	;; Jump to schedule() then return to ret_from_exception
	;;
_reschedule:
	MVKL	.S1	schedule,A0
	MVKH	.S1	schedule,A0
	BNOP	.S2X	A0,3
	MVKL	.S2	ret_from_exception,B3
	MVKH	.S2	ret_from_exception,B3

	;;
	;; Called before syscall handler when process is being debugged
	;;
tracesys_on:

	MVKL	.S1	syscall_trace_entry,A0
	MVKH	.S1	syscall_trace_entry,A0
	B	.S2X	A0
	ADDKPC	.S2	ret_from_syscall_trace,B3,3
	ADD	.S1X	8,SP,A4

ret_from_syscall_trace:
	;; tracing returns (possibly new) syscall number
	MV	.D2X	A4,B0

 ||	MVKL	.S1	sys_ni_syscall,A0
 ||	MVK	.S2	NR_SYSCALLS_,B1
	CMPLTU	.L2	B0,B1,B1
 ||	MVKH	.S1	sys_ni_syscall,A0

 [!B1]	B	.S2X	A0
 [!B1]	ADDKPC	.S2	ret_from_syscall_trace,B3,4

	;; reload syscall args from (possibly modified) stack frame
	LDW	.D2T2	*+SP(REGS_B4+8),B4
	LDW	.D2T1	*+SP(REGS_A6+8),A6
	LDW	.D2T2	*+SP(REGS_B6+8),B6
	LDW	.D2T1	*+SP(REGS_A8+8),A8
	LDW	.D2T2	*+SP(REGS_B8+8),B8

	;; Get syscall handler addr from _sys_call_table:
	MVKL	.S2	_sys_call_table,B1
	MVKH	.S2	_sys_call_table,B1
	LDW	.D2T2	*+B1[B0],B0		; B0 = _sys_call_table[__NR_*]
	MVKL	.S2	ret_from_syscall_function,B3
	MVKH	.S2	ret_from_syscall_function,B3
	NOP	2
	BNOP	.S2	B0,5			; branch to syscall handler
 ||	LDW	.D2T1	*+SP(REGS_ORIG_A4+8),A4

syscall_exit_work:
	AND	.D1	_TIF_SYSCALL_TRACE,A2,A0
 [!A0]	BNOP	.S1	work_pending,5
 [A0]	B	.S2	syscall_trace_exit
	ADDKPC	.S2	resume_userspace,B3,1
	MVC	.S2	CSR,B1
	SET	.S2	B1,0,0,B1
	MVC	.S2	B1,CSR		; enable ints

work_pending:
	AND	.D1	_TIF_NEED_RESCHED,A2,A0
 [!A0]	BNOP	.S1	work_notifysig,5

work_resched:
	MVKL	.S1	schedule,A1
	MVKH	.S1	schedule,A1
	B	.S2X	A1
	ADDKPC	.S2	work_rescheduled,B3,4
work_rescheduled:
	;; make sure we don't miss an interrupt setting need_resched or
	;; sigpending between sampling and the rti
	MASK_INT B2
	GET_THREAD_INFO A12
	LDW	.D1T1	*+A12(THREAD_INFO_FLAGS),A2
	MVK	.S1	_TIF_WORK_MASK,A1
	MVK	.S1	_TIF_NEED_RESCHED,A3
	NOP	2
	AND	.D1	A1,A2,A0
 ||	AND	.S1	A3,A2,A1
 [!A0]	BNOP	.S1	restore_all,5
 [A1]	BNOP	.S1	work_resched,5

work_notifysig:
	B	.S2	do_notify_resume
	LDW	.D2T1	*+SP(REGS__END+8),A6 ; syscall flag
	ADDKPC	.S2	resume_userspace,B3,1
	ADD	.S1X	8,SP,A4		; pt_regs pointer is first arg
	MV	.D2X	A2,B4		; thread_info flags is second arg

ret_from_exception:
#ifdef CONFIG_PREEMPT
	MASK_INT B2
#endif
	;;
	;; On C64x+, the return way from exception and interrupt
	;; is a little bit different
	;;
ret_from_interrupt:
#ifndef CONFIG_TMS320C64XPLUS
	;;
	;; Check if we are comming from user mode.
	;; C6x doesn't have a user/supervisor mode
	;; so we need to compare context stack and
	;; current one (ksp).
	;;
	LDW	.D2T2	*+SP(REGS_SP+8),B0
	MVKL	.S1	resume_kernel,A0
	MVKH	.S1	resume_kernel,A0
	NOP	2
	XOR	.D2	B0,SP,B0
	SHR	.S2	B0,THREAD_SHIFT,B0	; (SP ^ KSP) >> THREAD_SHIFT => User or Sup

#else

	;;
	;; Check if we are comming from user mode.
	;;
	LDW	.D2T2	*+SP(REGS_TSR+8),B0
	MVK	.S2	0x40,B1
	MVKL	.S1	resume_kernel,A0
	MVKH	.S1	resume_kernel,A0
	NOP	1
	AND	.D2	B0,B1,B0

#endif

 [!B0]	BNOP	.S2X	A0,5

resume_userspace:
	;; make sure we don't miss an interrupt setting need_resched or
	;; sigpending between sampling and the rti
	MASK_INT B2
	GET_THREAD_INFO A12
	LDW	.D1T1	*+A12(THREAD_INFO_FLAGS),A2
	MVK	.S1	_TIF_WORK_MASK,A1
	MVK	.S1	_TIF_NEED_RESCHED,A3
	NOP	2
	AND	.D1	A1,A2,A0
 [A0]	BNOP	.S1	work_pending,5
	BNOP	.S1	restore_all,5

	;;
	;; System call function (called by syscall int)
	;; B0 = syscall number (in _sys_call_table)
	;; A4,B4,A6,B6,A8,B8 = arguments of the syscall function
	;; A4 is the return value register
	;;
#ifndef CONFIG_TMS320C64XPLUS
	.global _system_call
_system_call:
	SAVE_ALL_EXCEPT			; save A0-A9/B0-B9 registers in stack
	MASK_SYSCALL
#endif

system_call_saved:
	MVK	.L2	1,B2
	STW	.D2T2	B2,*+SP(REGS__END+8)	; set syscall flag
#ifdef CONFIG_TMS320C64XPLUS
	MVC	.S2	B2,ECR			; acknowledge the software exception
#endif

	UNMASK_INT B2			; re-enable global IT

system_call_saved_noack:
	;; Check system call number
	MVK	.S2	NR_SYSCALLS_,B1
 ||	MVKL	.S1	sys_ni_syscall,A0
	CMPLTU	.L2	B0,B1,B1
 ||	MVKH	.S1	sys_ni_syscall,A0

 [!B1]	B	.S2X	A0
 [!B1]	ADDKPC	.S2	ret_from_syscall_function,B3,4

	;; Get syscall handler addr from _sys_call_table:
	MVKL	.S2	_sys_call_table,B1
	MVKH	.S2	_sys_call_table,B1
	LDW	.D2T2	*+B1[B0],B0		; B0 = _sys_call_table[__NR_*]

	;; Check for ptrace
	GET_THREAD_INFO A12
	LDW	.D1T1	*+A12(THREAD_INFO_FLAGS),A2
	NOP	4
	AND	.D1	_TIF_SYSCALL_TRACE,A2,A2 ; check for TIF_SYSCALL_TRACE flag
 [A2]	BNOP	.S1	tracesys_on,5		 ; call syscall_trace_entry before syscall function

	;; Branch to the corresponding syscall
	B	.S2	B0
	ADDKPC	.S2	ret_from_syscall_function,B3,4

ret_from_syscall_function:
	STW	.D2T1	A4,*+SP(REGS_A4+8)	; save return value in A4
						; original A4 is in orig_A4
syscall_exit:
	;; make sure we don't miss an interrupt setting need_resched or
	;; sigpending between sampling and the rti
	MASK_INT B2
	LDW	.D1T1	*+A12(THREAD_INFO_FLAGS),A2
	MVKL	.S1	syscall_exit_work,A0
	MVKH	.S1	syscall_exit_work,A0
	MVK	.S1	_TIF_ALLWORK_MASK,A1
	NOP
	AND	.D1	A1,A2,A2 ; check for work to do
 [A2]	BNOP	.S1	syscall_exit_work,5

restore_all:
	RESTORE_ALL_EXCEPT

	;;
	;; After a fork we jump here directly from resume,
	;; so that A4 contains the previous task structure.
	;;
	.global	ret_from_fork
ret_from_fork:
	MVKL	.S1	schedule_tail,A0
	MVKH	.S1	schedule_tail,A0
	B	.S2X	A0
	ADDKPC	.S2	ret_from_fork_2,B3,4
ret_from_fork_2:
	;; return 0 for child process
	MVK	.L2	0,B0
 ||	MVKL	.S2	syscall_exit,B1
	STW	.D2T2	B0,*+SP(REGS_A4+8)
 ||	MVKH	.S2	syscall_exit,B1
	BNOP	.S2	B1,5

	;;
	;; These are the interrupt handlers, responsible for calling __do_IRQ()
	;; int6 is used for syscalls (see _system_call entry)
	;;
_int4_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 4

_int5_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 5

#ifdef CONFIG_TMS320C64XPLUS
	.global	_int6_handler
_int6_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 6
#endif

_int7_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 7

_int8_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 8

_int9_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 9

_int10_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 10

_int11_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 11

_int12_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 12

_int13_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 13

_int14_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 14

_int15_handler:
	SAVE_ALL_INT
	MASK_SYSCALL
	CALL_INT 15

	;;
	;; Handler for uninitialized and spurious interrupts
	;;
	.global _bad_interrupt
_bad_interrupt:
	B	.S2	IRP
	NOP	5

	;;
	;; Handler for NMI (C6x) and exceptions (C64x+)
	;;
	.global _nmi_handler
_nmi_handler:
#ifndef CONFIG_TMS320C64XPLUS

	B	.S2	NRP
	NOP	5

#else
	SAVE_ALL_EXCEPT

	MVC	.S2	EFR,B2
	CMPEQ	.L2	1,B2,B2
 ||	MVC	.S2	TSR,B1
	MV	.D1X	B2,A2
 ||	CLR	.S2	B1,10,10,B1
	MVC	.S2	B1,TSR
 [!A2]	MVKL	.S1	process_exception,A0
 ||[B2]	MVKL	.S2	system_call_saved,B1
 [!A2]	MVKH	.S1	process_exception,A0
 ||[B2]	MVKH	.S2	system_call_saved,B1
 [!B2]	B	.S2X	A0
 [B2]	B	.S2	B1
 [!B2]	ADDAW	.D2	SP,2,B1
 [!B2]	MV	.D1X	B1,A4
	ADDKPC	.S2	ret_from_trap,B3,2

ret_from_trap:
	MV	.D2X	A4,B0
 [!B0]	MVKL	.S2	ret_from_exception,B3
 [!B0]	MVKH	.S2	ret_from_exception,B3
 [!B0]	BNOP	.S2	B3,5

	MVKL	.S2	system_call_saved_noack,B3
	MVKH	.S2	system_call_saved_noack,B3

	LDW	.D2T2	*+SP(REGS_B0+8),B0
	LDW	.D2T1	*+SP(REGS_A4+8),A4
	LDW	.D2T2	*+SP(REGS_B4+8),B4
	LDW	.D2T1	*+SP(REGS_A6+8),A6
	LDW	.D2T2	*+SP(REGS_B6+8),B6
	LDW	.D2T1	*+SP(REGS_A8+8),A8
 ||	B	.S2	B3
	LDW	.D2T2	*+SP(REGS_B8+8),B8
	NOP	4
#endif

	;;
	;; Jump to schedule() then return to ret_from_isr
	;;
#ifdef	CONFIG_PREEMPT
irq_reschedule:
	MVKL	.S1	schedule,A0
	MVKH	.S1	schedule,A0
	BNOP	.S2X	A0,3
	MVKL	.S2	ret_from_interrupt,B3
	MVKH	.S2	ret_from_interrupt,B3
	NOP	3

resume_kernel:
	GET_THREAD_INFO A12
	LDW	.D1T1	*+A12(THREAD_INFO_PREEMPT_COUNT),A1
	MVKL	.S1	restore_all,A0
	MVKH	.S1	restore_all,A0
	NOP	2
 [A1]	BNOP	.S2X	A0,5

preempt_schedule:
	GET_THREAD_INFO A2
	LDW	.D1T1	*+A2(THREAD_INFO_FLAGS),A1
	MVKL	.S1	restore_all,A0
	MVKH	.S1	restore_all,A0
	MVKL	.S2	preempt_schedule_irq,B0
	MVKH	.S2	preempt_schedule_irq,B0
	AND	.D1	_TIF_NEED_RESCHED,A1,A1	; check for TIF_NEED_RESCHED flag
 [!A1]	BNOP	.S2X	A0,5
	B	.S2	B0
	ADDKPC	.S2	preempt_schedule,B3,4
#endif

#ifdef CONFIG_TMS320C64XPLUS
	.global enable_exception
enable_exception:
	DINT
	MVC	.S2	TSR,B0
	MVC	.S2	B3,NRP
	MVK	.L2	0xc,B1
	OR	.D2	B0,B1,B0
	MVC	.S2	B0,TSR			;  Set GEE and XEN in TSR
	B	.S2	NRP
	NOP	5
#endif

	;;
	;; Special system calls
	;; return address is in B3
	;;
sys_fork:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	c6x_fork,A0		; branch to _c6x_fork(struct pt_regs *regs)
	MVKH	.S1	c6x_fork,A0
	BNOP	.S2X	A0,5

sys_clone:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	c6x_clone,A0
	MVKH	.S1	c6x_clone,A0
	BNOP	.S2X	A0,5

sys_vfork:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	c6x_vfork,A0		; branch to _c6x_vfork(struct pt_regs *regs)
	MVKH	.S1	c6x_vfork,A0
	BNOP	.S2X	A0,5

sys_sigsuspend:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	do_sigsuspend,A0	; branch to _do_sigsuspend
	MVKH	.S1	do_sigsuspend,A0
	BNOP	.S2X	A0,5

sys_rt_sigsuspend_wrapper:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	sys_rt_sigsuspend,A0	; branch to sys_rt_sigsuspend
	MVKH	.S1	sys_rt_sigsuspend,A0
	BNOP	.S2X	A0,5

sys_sigreturn:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	do_sigreturn,A0	; branch to _do_sigreturn
	MVKH	.S1	do_sigreturn,A0
	BNOP	.S2X	A0,5

sys_rt_sigreturn:
	ADD	.D1X	SP,8,A4
 ||	MVKL	.S1	do_rt_sigreturn,A0	; branch to _do_rt_sigreturn
	MVKH	.S1	do_rt_sigreturn,A0
	BNOP	.S2X	A0,5

sys_execve:
	ADDAW	.D2	SP,2,B6			; put regs addr in 4th parameter
						; & adjust regs stack addr
	LDW	.D2T2	*+SP(REGS_B4+8),B4

	;; c6x_execve(char *name, char **argv, char **envp, struct pt_regs *regs)
 ||	MVKL	.S1	c6x_execve,A0
	MVKH	.S1	c6x_execve,A0
	B	.S2X	A0
	STW	.D2T2	B3,*SP--[2]
	ADDKPC	.S2	ret_from_c6x_execve,B3,3

ret_from_c6x_execve:
	LDW	.D2T2	*++SP[2],B3
	NOP	4
	BNOP	.S2	B3,5

sys_pread_c6x:
	MVKL	.S1	sys_pread64,A0
	MVKH	.S1	sys_pread64,A0
	BNOP	.S2X	A0,4
	MV	.D2X	A8,B7

sys_pwrite_c6x:
	MVKL	.S1	sys_pwrite64,A0
	MVKH	.S1	sys_pwrite64,A0
	BNOP	.S2X	A0,4
	MV	.D2X	A8,B7

sys_truncate64_c6x:
	MVKL	.S1	sys_truncate64,A0
	MVKH	.S1	sys_truncate64,A0
	BNOP	.S2X	A0,4
	MV	.D2X	A6,B5

sys_ftruncate64_c6x:
	MVKL	.S1	sys_ftruncate64,A0
	MVKH	.S1	sys_ftruncate64,A0
	BNOP	.S2X	A0,4
	MV	.D2X	A6,B5

;; On Entry
;;   A4 - fd
;;   B4 - offset_lo (LE), offset_hi (BE)
;;   A6 - offset_lo (BE), offset_hi (LE)
;;   B6 - len
;;   A8 - advice
sys_fadvise64_c6x:
	MVKL	.S1	sys_fadvise64,A0
	MVKH	.S1	sys_fadvise64,A0
	BNOP	.S2X	A0,2
#ifdef CONFIG_CPU_BIG_ENDIAN
	MV	.L2	B4,B5
 ||	MV	.D2X	A6,B4
#else
	MV	.D2X	A6,B5
#endif
	MV	.D1X	B6,A6
	MV	.D2X	A8,B6

;; On Entry
;;   A4 - fd
;;   B4 - offset_lo (LE), offset_hi (BE)
;;   A6 - offset_lo (BE), offset_hi (LE)
;;   B6 - len_lo (LE), len_hi (BE)
;;   A8 - len_lo (BE), len_hi (LE)
;;   B8 - advice
sys_fadvise64_64_c6x:
	MVKL	.S1	sys_fadvise64_64,A0
	MVKH	.S1	sys_fadvise64_64,A0
	BNOP	.S2X	A0,2
#ifdef CONFIG_CPU_BIG_ENDIAN
	MV	.L2	B4,B5
 ||	MV	.D2X	A6,B4
	MV	.L1	A8,A6
 ||	MV	.D1X	B6,A7
#else
	MV	.D2X	A6,B5
	MV	.L1	A8,A7
 ||	MV	.D1X	B6,A6
#endif
	MV	.L2	B8,B6

;; On Entry
;;   A4 - fd
;;   B4 - mode
;;   A6 - offset_hi
;;   B6 - offset_lo
;;   A8 - len_hi
;;   B8 - len_lo
sys_fallocate_c6x:
	MVKL	.S1	sys_fallocate,A0
	MVKH	.S1	sys_fallocate,A0
	BNOP	.S2X	A0,1
	MV	.D1	A6,A7
	MV	.D1X	B6,A6
	MV	.D2X	A8,B7
	MV	.D2	B8,B6

	;;
	;; resume() switchs to a new task
	;; A4 = prev task
	;; B4 = next task
	;; A6 = thread offset
	;; B6 = shared flags (mm structure is shared between tasks)
	;;
	.global resume
resume:
	ADD	.D1	A6,A4,A0		; ptr to thread struct
 ||	MVC	.S2	CSR,B2
	STW	.D1T2	B2,*+A0(THREAD_CSR)	; save CSR

	;; Save non-scratch registers on stack
	SAVE_SWITCH_STACK

	;; Save current kernel stack pointer
	MV	.D1X	SP,A2
	STW	.D1T1	A2,*+A0(THREAD_KSP)
	;; Get pointer to thread struct (B4 contains the new task)
 ||	ADD	.D2X	B4,A6,B4

	;; Restore the kernel stack pointer
	LDW	.D2T2	*+B4(THREAD_KSP),B2
	MVKL	.S2	current_ksp,B1
	MVKH	.S2	current_ksp,B1
	NOP	2
	MV	.S2	B2,SP
 ||	STW	.D2T2	B2,*B1

	;; Restore non-scratch registers
	RESTORE_SWITCH_STACK

	;; Restore status register
	LDW	.D2T2	*+B4(THREAD_CSR),B2
	BNOP	.S2	B3,4		; return in next E1
	MVC	.S2	B2,CSR

#ifdef CONFIG_TI_C6X_COMPILER
	;;
	;; void * current_text_addr(void)
	;; Return current PC
	;;
	.global current_text_addr
current_text_addr:
	BNOP	.S2	B3,4
	MV	.D1X	B3,A4
	MVC	.S2	PC,B4
#endif

	;;
	;; unsigned int get_current(void):
	;; Return current process
	;;
	.global	__current_thread_info
__current_thread_info:
	BNOP	.S2	B3,3
	CLR	.S2	SP,0,THREAD_SHIFT-1,B4
	MV	.D1X	B4,A4

	;;
	;; System calls table
	;;
_DEFINE_MACRO(SYSCALL_ENTRY,handler)
#ifdef CONFIG_TI_C6X_COMPILER
	.ref   handler
#endif
	.word  handler
	.endm

_DEFINE_MACRO(SYSCALL_NI)
	.word  sys_ni_syscall
	.endm

_DEFINE_MACRO(SYSCALL_LOCAL,handler)
	.word  handler
	.endm

	.sect 	".data"
#ifdef CONFIG_TI_C6X_COMPILER
	.align 32
#else
	.align 5
#endif
_sys_call_table:
	SYSCALL_NI				; 0
	SYSCALL_ENTRY	sys_exit
	SYSCALL_LOCAL	sys_fork
	SYSCALL_ENTRY	sys_read
	SYSCALL_ENTRY	sys_write
	SYSCALL_ENTRY	sys_open
	SYSCALL_ENTRY	sys_close
	SYSCALL_ENTRY	sys_waitpid
	SYSCALL_ENTRY	sys_creat
	SYSCALL_ENTRY	sys_link
	SYSCALL_ENTRY	sys_unlink		; 10
	SYSCALL_LOCAL	sys_execve
	SYSCALL_ENTRY	sys_chdir
	SYSCALL_ENTRY	sys_time
	SYSCALL_ENTRY	sys_mknod
	SYSCALL_ENTRY	sys_chmod
	SYSCALL_ENTRY	sys_chown16
	SYSCALL_NI
	SYSCALL_ENTRY	sys_stat
	SYSCALL_ENTRY	sys_lseek
	SYSCALL_ENTRY	sys_getpid		; 20
	SYSCALL_ENTRY	sys_mount
	SYSCALL_ENTRY	sys_oldumount
	SYSCALL_ENTRY	sys_setuid16
	SYSCALL_ENTRY	sys_getuid16
	SYSCALL_ENTRY	sys_stime
	SYSCALL_ENTRY	sys_ptrace
	SYSCALL_ENTRY	sys_alarm
	SYSCALL_ENTRY	sys_fstat
	SYSCALL_ENTRY	sys_pause
	SYSCALL_ENTRY	sys_utime		; 30
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_access
	SYSCALL_ENTRY	sys_nice
	SYSCALL_NI
	SYSCALL_ENTRY	sys_sync
	SYSCALL_ENTRY	sys_kill
	SYSCALL_ENTRY	sys_rename
	SYSCALL_ENTRY	sys_mkdir
	SYSCALL_ENTRY	sys_rmdir		; 40
	SYSCALL_ENTRY	sys_dup
	SYSCALL_ENTRY	sys_pipe
	SYSCALL_ENTRY	sys_times
	SYSCALL_NI
	SYSCALL_ENTRY	sys_brk
	SYSCALL_ENTRY	sys_setgid16
	SYSCALL_ENTRY	sys_getgid16
	SYSCALL_ENTRY	sys_signal
	SYSCALL_ENTRY	sys_geteuid16
	SYSCALL_ENTRY	sys_getegid16		; 50
	SYSCALL_ENTRY	sys_acct
	SYSCALL_ENTRY	sys_umount
	SYSCALL_NI
	SYSCALL_ENTRY	sys_ioctl
	SYSCALL_ENTRY	sys_fcntl
	SYSCALL_NI
	SYSCALL_ENTRY	sys_setpgid
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_umask		; 60
	SYSCALL_ENTRY	sys_chroot
	SYSCALL_ENTRY	sys_ustat
	SYSCALL_ENTRY	sys_dup2
	SYSCALL_ENTRY	sys_getppid
	SYSCALL_ENTRY	sys_getpgrp
	SYSCALL_ENTRY	sys_setsid
	SYSCALL_ENTRY	sys_sigaction
	SYSCALL_ENTRY	sys_sgetmask
	SYSCALL_ENTRY	sys_ssetmask
	SYSCALL_ENTRY	sys_setreuid16		; 70
	SYSCALL_ENTRY	sys_setregid16
	SYSCALL_LOCAL	sys_sigsuspend
	SYSCALL_ENTRY	sys_sigpending
	SYSCALL_ENTRY	sys_sethostname
	SYSCALL_ENTRY	sys_setrlimit
	SYSCALL_ENTRY	sys_old_getrlimit
	SYSCALL_ENTRY	sys_getrusage
	SYSCALL_ENTRY	sys_gettimeofday
	SYSCALL_ENTRY	sys_settimeofday
	SYSCALL_ENTRY	sys_getgroups16		; 80
	SYSCALL_ENTRY	sys_setgroups16
	SYSCALL_ENTRY	old_select
	SYSCALL_ENTRY	sys_symlink
	SYSCALL_ENTRY	sys_lstat
	SYSCALL_ENTRY	sys_readlink
	SYSCALL_ENTRY	sys_uselib
	SYSCALL_ENTRY	sys_swapon
	SYSCALL_ENTRY	sys_reboot
	SYSCALL_ENTRY	sys_old_readdir
	SYSCALL_ENTRY	old_mmap		; 90
	SYSCALL_ENTRY	sys_munmap
	SYSCALL_ENTRY	sys_truncate
	SYSCALL_ENTRY	sys_ftruncate
	SYSCALL_ENTRY	sys_fchmod
	SYSCALL_ENTRY	sys_fchown16
	SYSCALL_ENTRY	sys_getpriority
	SYSCALL_ENTRY	sys_setpriority
	SYSCALL_NI
	SYSCALL_ENTRY	sys_statfs
	SYSCALL_ENTRY	sys_fstatfs		; 100
	SYSCALL_ENTRY	sys_ioperm
	SYSCALL_ENTRY	sys_socketcall
	SYSCALL_ENTRY	sys_syslog
	SYSCALL_ENTRY	sys_setitimer
	SYSCALL_ENTRY	sys_getitimer
	SYSCALL_ENTRY	sys_newstat
	SYSCALL_ENTRY	sys_newlstat
	SYSCALL_ENTRY	sys_newfstat
	SYSCALL_NI
	SYSCALL_NI				; 110
	SYSCALL_ENTRY	sys_vhangup
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_wait4
	SYSCALL_ENTRY	sys_swapoff
	SYSCALL_ENTRY	sys_sysinfo
	SYSCALL_ENTRY	sys_ipc
	SYSCALL_ENTRY	sys_fsync
	SYSCALL_LOCAL	sys_sigreturn
	SYSCALL_LOCAL	sys_clone		; 120
	SYSCALL_ENTRY	sys_setdomainname
	SYSCALL_ENTRY	sys_newuname
	SYSCALL_ENTRY	sys_cacheflush
	SYSCALL_ENTRY	sys_adjtimex
	SYSCALL_NI
	SYSCALL_ENTRY	sys_sigprocmask
	SYSCALL_NI
	SYSCALL_ENTRY	sys_init_module
	SYSCALL_ENTRY	sys_delete_module
	SYSCALL_NI				; 130
	SYSCALL_ENTRY	sys_quotactl
	SYSCALL_ENTRY	sys_getpgid
	SYSCALL_ENTRY	sys_fchdir
	SYSCALL_ENTRY	sys_bdflush
	SYSCALL_ENTRY	sys_sysfs
	SYSCALL_ENTRY	sys_personality
	SYSCALL_NI
	SYSCALL_ENTRY	sys_setfsuid16
	SYSCALL_ENTRY	sys_setfsgid16
	SYSCALL_ENTRY	sys_llseek		; 140
	SYSCALL_ENTRY	sys_getdents
	SYSCALL_ENTRY	sys_select
	SYSCALL_ENTRY	sys_flock
	SYSCALL_NI
	SYSCALL_ENTRY	sys_readv
	SYSCALL_ENTRY	sys_writev
	SYSCALL_ENTRY	sys_getsid
	SYSCALL_ENTRY	sys_fdatasync
	SYSCALL_ENTRY	sys_sysctl
	SYSCALL_NI				; 150
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_sched_setparam
	SYSCALL_ENTRY	sys_sched_getparam
	SYSCALL_ENTRY	sys_sched_setscheduler
	SYSCALL_ENTRY	sys_sched_getscheduler
	SYSCALL_ENTRY	sys_sched_yield
	SYSCALL_ENTRY	sys_sched_get_priority_max
	SYSCALL_ENTRY	sys_sched_get_priority_min	; 160
	SYSCALL_ENTRY	sys_sched_rr_get_interval
	SYSCALL_ENTRY	sys_nanosleep
	SYSCALL_NI
	SYSCALL_ENTRY	sys_setresuid16
	SYSCALL_ENTRY	sys_getresuid16
	SYSCALL_ENTRY	sys_getpagesize
	SYSCALL_NI
	SYSCALL_ENTRY	sys_poll
	SYSCALL_ENTRY	sys_nfsservctl
	SYSCALL_ENTRY	sys_setresgid16		; 170
	SYSCALL_ENTRY	sys_getresgid16
	SYSCALL_ENTRY	sys_prctl
	SYSCALL_LOCAL	sys_rt_sigreturn
	SYSCALL_ENTRY	sys_rt_sigaction
	SYSCALL_ENTRY	sys_rt_sigprocmask
	SYSCALL_ENTRY	sys_rt_sigpending
	SYSCALL_ENTRY	sys_rt_sigtimedwait
	SYSCALL_ENTRY	sys_rt_sigqueueinfo
	SYSCALL_LOCAL	sys_rt_sigsuspend
	SYSCALL_LOCAL	sys_pread_c6x		; 180
	SYSCALL_LOCAL	sys_pwrite_c6x
	SYSCALL_ENTRY	sys_lchown16
	SYSCALL_ENTRY	sys_getcwd
	SYSCALL_ENTRY	sys_capget
	SYSCALL_ENTRY	sys_capset
	SYSCALL_ENTRY	sys_sigaltstack
	SYSCALL_ENTRY	sys_sendfile
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_LOCAL	sys_vfork		; 190
	SYSCALL_ENTRY	sys_getrlimit
	SYSCALL_ENTRY	sys_mmap_pgoff
	SYSCALL_LOCAL	sys_truncate64_c6x
	SYSCALL_LOCAL	sys_ftruncate64_c6x
	SYSCALL_ENTRY	sys_stat64
	SYSCALL_ENTRY	sys_lstat64
	SYSCALL_ENTRY	sys_fstat64
	SYSCALL_ENTRY	sys_chown
	SYSCALL_ENTRY	sys_getuid
	SYSCALL_ENTRY	sys_getgid		; 200
	SYSCALL_ENTRY	sys_geteuid
	SYSCALL_ENTRY	sys_getegid
	SYSCALL_ENTRY	sys_setreuid
	SYSCALL_ENTRY	sys_setregid
	SYSCALL_ENTRY	sys_getgroups
	SYSCALL_ENTRY	sys_setgroups
	SYSCALL_ENTRY	sys_fchown
	SYSCALL_ENTRY	sys_setresuid
	SYSCALL_ENTRY	sys_getresuid
	SYSCALL_ENTRY	sys_setresgid		; 210
	SYSCALL_ENTRY	sys_getresgid
	SYSCALL_ENTRY	sys_lchown
	SYSCALL_ENTRY	sys_setuid
	SYSCALL_ENTRY	sys_setgid
	SYSCALL_ENTRY	sys_setfsuid
	SYSCALL_ENTRY	sys_setfsgid
	SYSCALL_ENTRY	sys_pivot_root
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_getdents64		; 220
	SYSCALL_ENTRY	sys_gettid
	SYSCALL_ENTRY	sys_tkill
	SYSCALL_ENTRY	sys_setxattr
	SYSCALL_ENTRY	sys_lsetxattr
	SYSCALL_ENTRY	sys_fsetxattr
	SYSCALL_ENTRY	sys_getxattr
	SYSCALL_ENTRY	sys_lgetxattr
	SYSCALL_ENTRY	sys_fgetxattr
	SYSCALL_ENTRY	sys_listxattr
	SYSCALL_ENTRY	sys_llistxattr		; 230
	SYSCALL_ENTRY	sys_flistxattr
	SYSCALL_ENTRY	sys_removexattr
	SYSCALL_ENTRY	sys_lremovexattr
	SYSCALL_ENTRY	sys_fremovexattr
	SYSCALL_ENTRY	sys_futex
	SYSCALL_ENTRY	sys_sendfile64
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_fcntl64
	SYSCALL_ENTRY	sys_readahead		; 240
	SYSCALL_ENTRY	sys_io_setup
	SYSCALL_ENTRY	sys_io_destroy
	SYSCALL_ENTRY	sys_io_getevents
	SYSCALL_ENTRY	sys_io_submit
	SYSCALL_ENTRY	sys_io_cancel
	SYSCALL_LOCAL	sys_fadvise64_c6x
	SYSCALL_ENTRY	sys_exit_group
	SYSCALL_ENTRY	sys_lookup_dcookie
	SYSCALL_ENTRY	sys_epoll_create
	SYSCALL_ENTRY	sys_epoll_ctl		; 250
	SYSCALL_ENTRY	sys_epoll_wait
	SYSCALL_NI
	SYSCALL_ENTRY	sys_set_tid_address
	SYSCALL_ENTRY	sys_timer_create
	SYSCALL_ENTRY	sys_timer_settime
	SYSCALL_ENTRY	sys_timer_gettime
	SYSCALL_ENTRY	sys_timer_getoverrun
	SYSCALL_ENTRY	sys_timer_delete
	SYSCALL_ENTRY	sys_clock_settime
	SYSCALL_ENTRY	sys_clock_gettime	; 260
	SYSCALL_ENTRY	sys_clock_getres
	SYSCALL_ENTRY	sys_clock_nanosleep
	SYSCALL_ENTRY	sys_statfs64
	SYSCALL_ENTRY	sys_fstatfs64
	SYSCALL_ENTRY	sys_tgkill
	SYSCALL_ENTRY	sys_utimes
	SYSCALL_LOCAL	sys_fadvise64_64_c6x
	SYSCALL_ENTRY	sys_mbind
	SYSCALL_ENTRY	sys_get_mempolicy
	SYSCALL_ENTRY	sys_set_mempolicy	; 270
	SYSCALL_ENTRY	sys_mq_open
	SYSCALL_ENTRY	sys_mq_unlink
	SYSCALL_ENTRY	sys_mq_timedsend
	SYSCALL_ENTRY	sys_mq_timedreceive
	SYSCALL_ENTRY	sys_mq_notify
	SYSCALL_ENTRY	sys_mq_getsetattr
	SYSCALL_ENTRY	sys_waitid
	SYSCALL_NI
	SYSCALL_ENTRY	sys_add_key
	SYSCALL_ENTRY	sys_request_key		; 280
	SYSCALL_ENTRY	sys_keyctl
	SYSCALL_NI
	SYSCALL_NI
	SYSCALL_ENTRY	sys_sched_setaffinity
	SYSCALL_ENTRY	sys_sched_getaffinity
	SYSCALL_NI
	SYSCALL_ENTRY	sys_ioprio_set
	SYSCALL_ENTRY	sys_ioprio_get
	SYSCALL_ENTRY	sys_inotify_add_watch
	SYSCALL_ENTRY	sys_inotify_rm_watch    ; 290
	SYSCALL_ENTRY	sys_openat
	SYSCALL_ENTRY	sys_mkdirat
	SYSCALL_ENTRY	sys_mknodat
	SYSCALL_ENTRY	sys_fchownat
	SYSCALL_ENTRY	sys_fstatat64
	SYSCALL_ENTRY	sys_unlinkat
	SYSCALL_ENTRY	sys_renameat
	SYSCALL_ENTRY	sys_linkat
	SYSCALL_ENTRY	sys_symlinkat
	SYSCALL_ENTRY	sys_readlinkat		; 300
	SYSCALL_ENTRY	sys_fchmodat
	SYSCALL_ENTRY	sys_faccessat
	SYSCALL_ENTRY	sys_pselect6
	SYSCALL_ENTRY	sys_ppoll
	SYSCALL_ENTRY	sys_unshare
	SYSCALL_ENTRY	sys_set_robust_list
	SYSCALL_ENTRY	sys_get_robust_list
	SYSCALL_ENTRY	sys_splice
	SYSCALL_ENTRY	sys_sync_file_range
	SYSCALL_ENTRY	sys_tee			; 310
	SYSCALL_ENTRY	sys_vmsplice
	SYSCALL_ENTRY	sys_getcpu
	SYSCALL_ENTRY	sys_epoll_pwait
	SYSCALL_ENTRY	sys_utimensat
	SYSCALL_ENTRY	sys_timerfd_create
	SYSCALL_LOCAL	sys_fallocate_c6x
	SYSCALL_ENTRY	sys_timerfd_settime
	SYSCALL_ENTRY	sys_timerfd_gettime
	SYSCALL_ENTRY	sys_signalfd4
	SYSCALL_ENTRY	sys_eventfd2		; 320
	SYSCALL_ENTRY	sys_epoll_create1
	SYSCALL_ENTRY	sys_dup3
	SYSCALL_ENTRY	sys_pipe2
	SYSCALL_ENTRY	sys_inotify_init1
	SYSCALL_ENTRY	sys_preadv
	SYSCALL_ENTRY	sys_pwritev
	SYSCALL_ENTRY	sys_rt_tgsigqueueinfo
	SYSCALL_ENTRY	sys_perf_event_open
	SYSCALL_ENTRY	sys_recvmmsg
	SYSCALL_ENTRY	sys_cache_sync	; 330

