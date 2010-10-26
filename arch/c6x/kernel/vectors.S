;
;  linux/arch/c6x/kernel/vector.s
;
;  Port on Texas Instruments TMS320C6x architecture
;
;  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
;  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
;
;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License version 2 as
;  published by the Free Software Foundation.
;
;  This section handles all the interrupt vector routines.
;  At RESET the processor sets up the DRAM timing parameters and
;  branches to the label _c_int00 which handles initialization for the C code.
;

#ifdef CONFIG_TI_C6X_COMPILER
	   .include	"c6x_config.inc"

           .ref  _c_int00			; Linux starting point

#define ALIGNMENT 32
#else
#define ALIGNMENT 5
#endif


#ifdef CONFIG_TI_C6X_COMPILER
IRQVEC	   .macro __name,__handler
	   .align ALIGNMENT
	   .ref __handler
	   .global __name
__name:    STW	.D2T1     A0,*B15--[2]
       ||  MVKL	.S1     __handler,A0
           MVKH	.S1     __handler,A0
           B	.S2X	A0
           LDW	.D2T1	*++B15[2],A0
           NOP         	4
           NOP
           NOP
	   .endm
#else
	.macro IRQVEC name, handler
	.align ALIGNMENT
	.global \name
\name:	STW	.D2T1	A0,*B15--[2]
 ||	MVKL	.S1	\handler,A0
	MVKH	.S1	\handler,A0
	B	.S2X	A0
	LDW	.D2T1	*++B15[2],A0
	NOP	4
	NOP
	NOP
	.endm
#endif

#ifdef CONFIG_TI_C6X_COMPILER
           .sect ".vectors"
#else
           .sect ".vectors","ax"
#endif

	   .align ALIGNMENT
RESET:     MVKL	.S1	_c_int00,A0		; branch to _c_int00
           MVKH	.S1	_c_int00,A0
           B	.S2X	A0
           NOP
           NOP
           NOP
           NOP
           NOP


           IRQVEC NMI,_nmi_handler		; NMI interrupt
           IRQVEC AINT,_bad_interrupt		; reserved
           IRQVEC MSGINT,_bad_interrupt		; reserved

           IRQVEC INT4,_int4_handler
           IRQVEC INT5,_int5_handler
#ifndef	CONFIG_TMS320C64XPLUS
           IRQVEC INT6,_system_call
#else
           IRQVEC INT6,_int6_handler
#endif
           IRQVEC INT7,_int7_handler
           IRQVEC INT8,_int8_handler
           IRQVEC INT9,_int9_handler
           IRQVEC INT10,_int10_handler
           IRQVEC INT11,_int11_handler
           IRQVEC INT12,_int12_handler
           IRQVEC INT13,_int13_handler
           IRQVEC INT14,_int14_handler
           IRQVEC INT15,_int15_handler
