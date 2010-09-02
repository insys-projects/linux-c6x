/*
 * TI linker script to build intermediate libraries
 */
SECTIONS
{
    .vectors
    .cmdline
    .head.text
    .text.hot
    .text
    .ref.text
    .text.unlikely
    .sched.text
    .spinlock.text
    .kprobes.text
    .irqentry.text
    .kstrtab
    __ex_table
    .rodata {
	*(.rodata)
	*(.rodata.*)
	*(__vermagic)
	*(__markers_strings)
	*(__tracepoints_strings)
    }
    .rodata1
    __bug_table

    .builtin_fw
    .rio_route_ops
    .tracedata

    __ksymtab
    __ksymtab_gpl
    __ksymtab_unused
    __ksymtab_unused_gpl
    __kcrctab
    __kcrctab_gpl
    __kcrctab_unused
    __kcrctab_unused_gpl
    __kcrctab_gpl_future
    __ksymtab_strings
    __param

    .data.init_task
    .data.nosave
    .data.page_aligned
    .data.cacheline_aligned
    .data.read_mostly
    .data
    .fardata
    .ref.data
    __markers
    __tracepoints
    __verbose

    .init.data
    .init.rodata
    .devinit.rodata
    .devexit.rodata
    .cpuinit.rodata
    .cpuexit.rodata
    .meminit.rodata
    .memexit.rodata

    .init.text
    .devinit.text
    .devinit.data
    .devinit.rodata
    .cpuinit.text
    .cpuinit.data
    .cpuinit.rodata
    .meminit.text
    .meminit.data
    .meminit.rodata
    .init.setup
    .initcallearly.init

    .initcall0.init
    .initcall1.init
    .initcall2.init
    .initcall3.init
    .initcall4.init
    .initcall5.init
    .initcall6.init
    .initcall7.init
    .initcall0s.init
    .initcall1s.init
    .initcall2s.init
    .initcall3s.init
    .initcall4s.init
    .initcall5s.init
    .initcall6s.init
    .initcall7s.init
    .con_initcall.init
    .security_initcall.init
    .init.ramfs

    .exit.text
    .devexit.text
    .cpuexit.text
    .memexit.text

    .exit.data
    .devexit.data
    .devexit.rodata
    .cpuexit.data
    .cpuexit.rodata
    .memexit.data
    .memexit.rodata

    .exitcall.exit

    .ctors

    .tables
    .switch
    .const
    .cio

    .bss

    .bss.page_aligned

    .far
    .neardata
    .sysmem
    .cinit
}
