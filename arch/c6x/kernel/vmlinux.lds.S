/*
 * ld script for the c6x kernel
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Mark Salter <msalter@redhat.com>
 */
#define __VMLINUX_LDS__
#include <asm-generic/vmlinux.lds.h>
#include <asm/thread_info.h>
#include <asm/page.h>
#include <mach/board.h>

#ifdef CONFIG_TI_C6X_LINKER

#define AT(x)

/* The TI linker doesn't get the order right without explicitly naming files... */
#define TI_INITCALL(x) \
	"arch/c6x/kernel/built-in.o"(x) \
        "arch/c6x/mm/built-in.o"(x) \
        "arch/c6x/drivers/built-in.o"(x) \
        "arch/c6x/platforms/built-in.o"(x) \
        "kernel/built-in.o"(x) \
        "mm/built-in.o"(x) \
        "fs/built-in.o"(x) \
        "ipc/built-in.o"(x) \
        "security/built-in.o"(x) \
        "crypto/built-in.o"(x) \
        "block/built-in.o"(x) \
        "lib/built-in.o"(x) \
        "arch/c6x/lib/built-in.o"(x) \
        "drivers/built-in.o"(x) \
        "sound/built-in.o"(x) \
        "firmware/built-in.o"(x) \
        "net/built-in.o"(x) \
  	*(x)

MEMORY
{
	BMEMe0:	org = PAGE_OFFSET_RAW len = KERNEL_TEXT_LEN /* External DDR2 */
}

SECTIONS
{
  GROUP : PAGE_OFFSET_RAW {
	
    .vectors : ALIGN(1024)
    .cmdline : ALIGN(4096)

    /* read-only */
    .text {
        _stext = .;
        _text = .;
	HEAD_TEXT
	*(.text.hot)
	*(.text)
	*(.ref.text)
	DEV_KEEP(init.text)
	DEV_KEEP(exit.text)
	CPU_KEEP(init.text)
	CPU_KEEP(exit.text)
	MEM_KEEP(init.text)
	MEM_KEEP(exit.text)
	*(.text.unlikely)

	VMLINUX_SYMBOL(__sched_text_start) = .;
	*(.sched.text)
	VMLINUX_SYMBOL(__sched_text_end) = .;

	VMLINUX_SYMBOL(__lock_text_start) = .;
	*(.spinlock.text)
	VMLINUX_SYMBOL(__lock_text_end) = .;

	VMLINUX_SYMBOL(__kprobes_text_start) = .;
	*(.kprobes.text)
	VMLINUX_SYMBOL(__kprobes_text_end) = .;

	*(.fixup)
	*(.gnu.warning)
        _etext = .;
    }

    .const
    .switch
    .exitcall.exit

    NOTES

    __ex_table : ALIGN(16) {
	VMLINUX_SYMBOL(__start___ex_table) = .;
	*(__ex_table)
	VMLINUX_SYMBOL(__stop___ex_table) = .;
    }

    /* read-only */
    .rodata : ALIGN(4096) {
        __start_rodata = .;
	*(.rodata) *(.rodata.*)
	*(__vermagic)
	*(__markers_strings)
	*(__tracepoints_strings)
    }

    .rodata1 : ALIGN(4) {
        *(.rodata1)
    }

    __bug_table : ALIGN(8) {
	__start___bug_table = .;
       *(__bug_table)
       __stop___bug_table = .;
    }

    .builtin_fw  : ALIGN(4) {
	__start_builtin_fw = .;
	*(.builtin_fw)
	__end_builtin_fw = .;
    }

    .rio_rops  : ALIGN(4) {
	__start_rio_route_ops = .;
	*(.rio_route_ops)
	__end_rio_route_ops = .;
	__start_rio_em_ops = .;
	*(.rio_em_ops)
	__end_rio_em_ops = .;	
    }

    .tracedata : ALIGN(4) {
        __tracedata_start = .;
        *(.tracedata)
        __tracedata_end = .;
    }

    __ksymtab  : ALIGN(4) {
	__start___ksymtab = .;
	*(__ksymtab)
	__stop___ksymtab = .;
    }

    __ksymtab_gpl  : ALIGN(4) {
        __start___ksymtab_gpl = .;
	*(__ksymtab_gpl)
	__stop___ksymtab_gpl = .;
    }

    __ksymtab_unused  : ALIGN(4) {
        __start___ksymtab_unused = .;
        *(__ksymtab_unused)
        __stop___ksymtab_unused = .;
    }

    __ksymtab_unused_gpl  : ALIGN(4) {
        __start___ksymtab_unused_gpl = .;
        *(__ksymtab_unused_gpl)
        __stop___ksymtab_unused_gpl = .;
    }

    __ksymtab_gpl_future  : ALIGN(4) {
        __start___ksymtab_gpl_future = .;
        *(__ksymtab_gpl_future)
        __stop___ksymtab_gpl_future = .;
    }

    __kcrctab  : ALIGN(4) {
        __start___kcrctab = .;
        *(__kcrctab)
	__stop___kcrctab = .;
    }

    __kcrctab_gpl  : ALIGN(4) {
        __start___kcrctab_gpl = .;
	*(__kcrctab_gpl)
	__stop___kcrctab_gpl = .;
    }

    __kcrctab_unused  : ALIGN(4) {
        __start___kcrctab_unused = .;
        *(__kcrctab_unused)
        __stop___kcrctab_unused = .;
    }

    __kcrctab_unused_gpl  : ALIGN(4) {
        __start___kcrctab_unused_gpl = .;
        *(__kcrctab_unused_gpl)
        __stop___kcrctab_unused_gpl = .;
    }

    __kcrctab_gpl_future  : ALIGN(4) {
        __start___kcrctab_gpl_future = .;
	*(__kcrctab_gpl_future)
	__stop___kcrctab_gpl_future = .;
    }

    __ksymtab_strings  : ALIGN(4) {
        *(__ksymtab_strings)
    }

    __init_rodata : ALIGN(4) {
        *(.ref.rodata)
    }

    __param : ALIGN(4) {
        __start___param = .;
        *(__param)
        __stop___param = .;
        __end_rodata = .;
    }

    /* writeable */
    .data : ALIGN(8192) {
        _sdata = .;
	*(.data.init_task)

	/* NOSAVE */
	VMLINUX_SYMBOL(__nosave_begin) = .;
	*(.data.nosave)
	VMLINUX_SYMBOL(__nosave_end) = .;

        *(.data.page_aligned)
	*(.data.cacheline_aligned)
        *(.data.read_mostly)
	*(.data)
        *(.fardata)
	*(.ref.data)
	DEV_KEEP(init.data)
	DEV_KEEP(exit.data)
	CPU_KEEP(init.data)
	CPU_KEEP(exit.data)
	MEM_KEEP(init.data)
	MEM_KEEP(exit.data)
	VMLINUX_SYMBOL(__start___markers) = .;
	*(__markers)
	VMLINUX_SYMBOL(__stop___markers) = .;

	VMLINUX_SYMBOL(__start___tracepoints) = .;
	*(__tracepoints)
	VMLINUX_SYMBOL(__stop___tracepoints) = .;

	VMLINUX_SYMBOL(__start___verbose) = .;
	*(__verbose)
	VMLINUX_SYMBOL(__stop___verbose) = .;
	LIKELY_PROFILE()
	BRANCH_PROFILE()
	TRACE_PRINTKS()
	FTRACE_EVENTS()
	TRACE_SYSCALLS()
	*(.sysmem)
    }
    .cio
    .cinit

    .init.text : PALIGN(4096) {
        _edata = .;
        __init_begin = .;
	VMLINUX_SYMBOL(_sinittext) = .;
	INIT_TEXT
	VMLINUX_SYMBOL(_einittext) = .;

	INIT_DATA
    }
    .init.setup : PALIGN(4096) {
	VMLINUX_SYMBOL(__setup_start) = .;
	*(.init.setup)
	VMLINUX_SYMBOL(__setup_end) = .;

	VMLINUX_SYMBOL(__initcall_start) = .;
	*(.initcallearly.init)
	VMLINUX_SYMBOL(__early_initcall_end) = .;
  	TI_INITCALL(.initcall0.init)
  	TI_INITCALL(.initcall0s.init)
  	TI_INITCALL(.initcall1.init)
  	TI_INITCALL(.initcall1s.init)
  	TI_INITCALL(.initcall2.init)
  	TI_INITCALL(.initcall2s.init)
  	TI_INITCALL(.initcall3.init)
  	TI_INITCALL(.initcall3s.init)
  	TI_INITCALL(.initcall4.init)
  	TI_INITCALL(.initcall4s.init)
  	TI_INITCALL(.initcall5.init)
  	TI_INITCALL(.initcall5s.init)
	TI_INITCALL(.initcallrootfs.init)
  	TI_INITCALL(.initcall6.init)
  	TI_INITCALL(.initcall6s.init)
  	TI_INITCALL(.initcall7.init)
  	TI_INITCALL(.initcall7s.init)
	VMLINUX_SYMBOL(__initcall_end) = .;

	VMLINUX_SYMBOL(__con_initcall_start) = .;
	*(.con_initcall.init)
	VMLINUX_SYMBOL(__con_initcall_end) = .;

	VMLINUX_SYMBOL(__security_initcall_start) = .;
	*(.security_initcall.init)
	VMLINUX_SYMBOL(__security_initcall_end) = .;

	VMLINUX_SYMBOL(__initramfs_start) = .;
	*(.init.ramfs)
	VMLINUX_SYMBOL(__initramfs_end) = .;
    }

    /* .exit.text is discard at runtime, not link time, to deal with references
       from .altinstructions and .eh_frame */
    .exit.text {
        EXIT_TEXT
    }
    .exit.data {
        EXIT_DATA
    }

    .data.percpu : ALIGN(32) {
	    VMLINUX_SYMBOL(__per_cpu_load) = .;
	    VMLINUX_SYMBOL(__per_cpu_start) = .;
	    *(.data.percpu.first)
	    *(.data.percpu.page_aligned)
	    *(.data.percpu)
	    *(.data.percpu.shared_aligned)
	    VMLINUX_SYMBOL(__per_cpu_end) = .;
    }
    /* freed after init ends here */

    .tables : ALIGN(4096) {
        __init_end = .;
        *(.tables)
    }

    .bss : PALIGN(4096) {
        VMLINUX_SYMBOL(__bss_start) = .;
        VMLINUX_SYMBOL(_bss_start) = .;
	*(.sbss)
	*(.scommon)
#ifdef CONFIG_CPU_BIG_ENDIAN
	VMLINUX_SYMBOL(jiffies) = VMLINUX_SYMBOL(jiffies_64) + 4;
#else
	VMLINUX_SYMBOL(jiffies) = VMLINUX_SYMBOL(jiffies_64);
#endif
	*(.bss.page_aligned)
	*(.dynbss)
	*(.bss)
        *(.far)
	*(COMMON)
    }
    .bss_end_markers {
        VMLINUX_SYMBOL(__bss_stop) = .;
        VMLINUX_SYMBOL(_bss_end) = .;
        VMLINUX_SYMBOL(_ebss) = .;
        _end = . ;
    }
    .ram {
         __ramstart = . ;
    }
  }
}

#else /* CONFIG_TI_C6X_LINKER */

ENTRY(_c_int00)

#ifdef CONFIG_CPU_BIG_ENDIAN
jiffies = jiffies_64 + 4;
#else
jiffies = jiffies_64;
#endif

SECTIONS
{
  . = PAGE_OFFSET_RAW;
  .vectors : { *(.vectors) }

  . = PAGE_OFFSET_RAW + 0x400;
  . = ALIGN(0x1000);
  .cmdline : { *(.cmdline) }

  /* read-only */
  _stext = .;
  _text = .;
  .text : {
      HEAD_TEXT
      TEXT_TEXT
      SCHED_TEXT
      LOCK_TEXT
      KPROBES_TEXT
      *(.fixup)
      *(.gnu.warning)
  }

  _etext = .;

  EXCEPTION_TABLE(16)
  NOTES

  _sdata = .;
  RO_DATA_SECTION(32)
  .rodata_c6x : {
    *(.const)
    *(.switch)
  }
  RW_DATA_SECTION(32, PAGE_SIZE, THREAD_SIZE)
  _edata = .;

  /* will be freed after init */
  . = ALIGN(PAGE_SIZE);		/* Init code and data */
  __init_begin = .;
  INIT_TEXT_SECTION(PAGE_SIZE)
  INIT_DATA_SECTION(16)

  PERCPU(32)
  . = ALIGN(PAGE_SIZE);
  __init_end = .;

  VMLINUX_SYMBOL(_bss_start) = .;
  SBSS(32)
  BSS(PAGE_SIZE)
  . = ALIGN(32);
  .far : { *(.far) }
  . = ALIGN(32);
  VMLINUX_SYMBOL(__bss_stop) = .;
  VMLINUX_SYMBOL(_bss_end) = .;
  VMLINUX_SYMBOL(_ebss) = .;

  _end = . ;

  __ramstart = . ;

  DISCARDS
}

#endif /* CONFIG_TI_C6X_LINKER */
