;
;  linux/arch/c6x/kernel/head.s
;
;  Port on Texas Instruments TMS320C6x architecture
;
;  Copyright (C) 2004, 2009 Texas Instruments Incorporated
;  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
;
;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License version 2 as
;  published by the Free Software Foundation.
;

#ifdef CONFIG_TI_C6X_COMPILER
	.include	"c6x_defs.inc"

	.ref    current_ksp
	.ref	start_kernel
	.ref	_bss_start
	.ref	_bss_end
	.ref	c6x_tags_pointer
	.ref	c6x_mtd_early_init

	.include	"c6x_config.inc"
#else
#include "tags.h"
#endif

	.global	_c_int00
_c_int00:
	;; Save tag magic and pointer
	MV 	.S1	A4,A10
	MV 	.S2	B4,B10

	;; Set the stack pointer
	MVKL	.S2	current_ksp,B0
	MVKH	.S2	current_ksp,B0
	LDW	.D2T2	*B0,B15
	NOP	4
	AND	.D2	~7,B15,B15

#ifdef CONFIG_MTD_UCLINUX
	MVKL	.S2	c6x_mtd_early_init,B0
	MVKH	.S2	c6x_mtd_early_init,B0
	B	.S2	B0
	ADDKPC	.S2	_ret_from_mtd_early_init,B3,4

_ret_from_mtd_early_init:
#endif

	;; Set global page pointer in B14
	MVKL	.S2	_bss_start,B14
	MVKH	.S2	_bss_start,B14

	;; Clear bss area
	MVKL	.S1	_bss_start,A4
	MVKH	.S1	_bss_start,A4
	MVKL	.S1	_bss_end,A6
	MVKH	.S1	_bss_end,A6

        CMPLTU	.L1	A4,A6,A1
 [!A1]  B	.S1	__bss_done
        ZERO	.L2	B4
        SUB	.L1	A6,A4,A5
        SHR	.S1	A5,2,A3
        NOP     2

        SUB	.L2X	A3,1,B0
  ||    MV	.L1X	B4,A3

__bss_loop:
	STW	.D1T1	A3,*A4++
|| [B0]	B	.S1     __bss_loop
|| [B0]	SUB	.L2	B0,1,B0

__bss_done:

	;; Clear GIE and PGIE
	MVC	.S2	CSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,CSR
#ifdef CONFIG_TMS320C64XPLUS
	MVC	.S2	TSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,TSR
	MVC	.S2	ITSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,ITSR
	MVC	.S2	NTSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,NTSR
#endif

#ifdef CONFIG_NK
	.ref	_nkctx
	;; Save NK2 OS context pointer given in B4
	MVKL	.S1	_nkctx,A0
	MVKH	.S1	_nkctx,A0
	STW	.D1T2	B4,*A0
#endif

	MVKL	.S2	TAGS_MAGIC,B0
	MVKH	.S2	TAGS_MAGIC,B0
	CMPEQ   .L1	A10,A0,A0
	MVKL	.S2	c6x_tags_pointer,B1
	MVKH	.S2	c6x_tags_pointer,B1
  [!A0] MVK	.S2	0,B10
	STW	.D2T2	B10,*B1

	;; Jump to Linux init
	MVKL	.S1	start_kernel,A0
	MVKH	.S1	start_kernel,A0
	B	.S2X	A0
	MVKL	.S2	L1,B3
	MVKH	.S2	L1,B3
	NOP	3
L1:	B	.S2	B3
	NOP	5

