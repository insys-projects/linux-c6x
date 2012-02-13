/*
 *  linux/arch/c6x/boot/compressed/head.S
 *
 *  Copyright (C) 2012 Aurelien Jacquiot <a-jacquiot@ti.com>
 *	
 */

.text

#include <asm/page.h>
	
	.global	startup
startup:
	/* Save tag magic and pointer */
	MV 	.S1	A4,A10
	MV 	.S2	B4,B10

	/* Set the stack pointer */
	MVKL	.S2	stack_start,B0
	MVKH	.S2	stack_start,B0
	LDW	.D2T2	*B0,B15
	NOP	4
	AND	.D2	~7,B15,B15

	/* Set global page pointer in B14 */
	MVKL	.S2	_bss_start,B14
	MVKH	.S2	_bss_start,B14

	/* Clear bss area */
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

	/* Clear GIE and PGIE */
	MVC	.S2	CSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,CSR
	MVC	.S2	TSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,TSR
	MVC	.S2	ITSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,ITSR
	MVC	.S2	NTSR,B2
	CLR	.S2	B2,0,1,B2
	MVC	.S2	B2,NTSR

	/* Uncompress kernel */
	MVKL	.S2	decompress_kernel,B0
	MVKH	.S2	decompress_kernel,B0
	B	.S2	B0
	ADDKPC	.S2	_ret_from_decompress_kernel,B3,4
	
_ret_from_decompress_kernel:
	
	/* Jump to Linux init */
	MV 	.S1	A4,A0
	MV 	.S1	A10,A4
  ||	MV 	.S2	B10,B4
	B	.S2X	A0
	MVKL	.S2	L1,B3
	MVKH	.S2	L1,B3
	NOP	3
L1:	B	.S2	B3
	NOP	5
	
	.align	9
fake_headers_as_bzImage:
	.short	0
	.ascii	"HdrS"
	.short	0x0202
	.short	0
	.short	0
	.byte	0x00, 0x10
	.short	0
	.byte	0
	.byte	1
	.byte	0x00, 0x80
	.long	0
	.long	0
