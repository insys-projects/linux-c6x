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
	BDEC .S1  _c6x_delay,A4
	NOP       5
	B    .S2  B3
	NOP       5
