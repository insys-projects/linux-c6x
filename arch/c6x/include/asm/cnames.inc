

csym	.macro x
	.asg   _:x:, :x:
	.endm

	;; These are the symbols used in both C and asm code
	;; For coff builds, they need a '_' prefix in then asm code

	.if !  __TI_EABI__
	csym   irq_IER

	csym   __do_IRQ
	csym   resume
	csym   ret_from_exception
	csym   ret_from_interrupt

	csym   csum_partial
	csym   csum_partial_copy
	csym   ip_fast_csum

	csym   _c6x_delay
	csym   memcpy

	csym   _kernel_thread
	csym   jiffies_64
	csym   jiffies

	csym   __current_thread_info
	csym   current_test_addr
	csym   enable_exception
	csym   ret_from_fork
	csym   c6x_clone

	csym   c6x_exec_memobj
	csym   c6x_execve
	csym   c6x_fork
	csym   c6x_vfork
	csym   do_exit
	csym   do_rt_sigreturn
	csym   do_rt_sigsuspend
	csym   do_signal
	csym   do_sigreturn
	csym   do_sigsuspend
	csym   init_thread_union
	csym   process_exception
	csym   process_int
	csym   current_text_addr
	csym   schedule
	csym   schedule_tail
	csym   preempt_schedule_irq
	csym   start_kernel
	csym   syscall_trace

	.endif
