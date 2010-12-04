;
;  linux/arch/c6x/lib/delay.s
;
;  Port on Texas Instruments TMS320C6x architecture
;
;  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
;  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
;
;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License version 2 as
;  published by the Free Software Foundation.
;
	.sect	".text"
	.global	_c6x_delay

_c6x_delay:
	bdec	.s1	_c6x_delay,A4
	nop	5
	b	.s2	B3
	nop	5

#ifdef CONFIG_TI_C6X_COMPILER
	;; this is inlined with gcc

	.global	_c6x_tickdelay
_c6x_tickdelay:
	mvc	.s2	TSCL,B1
	add	.s2x	B1,A4,B1
 ||	mvk	.l2	1,B0
$0:
 [B0]	b	.s2	$0
	mvc	.s2	TSCL,B0
	sub	.s2	B0,B1,B0
	cmpgt	.l2	0,B0,B0
	nop
	nop
	b	.s2	B3
	nop	5
#endif
