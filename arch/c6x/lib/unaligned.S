;
;  arch/c6x/kernel/unaligned.S
;
;  Port on Texas Instruments TMS320C6x architecture
;
;  Copyright (C) 2010 Texas Instruments Incorporated
;  Author: Mark Salter (msalter@redhat.com)
;
;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License version 2 as
;  published by the Free Software Foundation.
;

	.text
	.global get_unaligned_le64
	.global get_unaligned_be64
	.global get_unaligned_le32
	.global get_unaligned_be32
	.global put_unaligned_le64
	.global put_unaligned_be64
	.global put_unaligned_le32
	.global put_unaligned_be32

; u64 get_unaligned_le64(void *ptr);
; u64 get_unaligned_be64(void *ptr);
;
#ifdef CONFIG_CPU_BIG_ENDIAN
get_unaligned_le64:
#else
get_unaligned_be64:
#endif
	LDNDW	.D1T1	*A4,A5:A4
	NOP	2
	BNOP	.S2	B3,1
	MV	.D2X	A4,B0
	SWAP2	.S2	B0,B1
 ||	SWAP2	.S1	A5,A0
	SWAP4	.L2	B1,B0
 ||	SWAP4	.L1	A0,A4
	MV	.D1X	B0,A5

#ifdef CONFIG_CPU_BIG_ENDIAN
get_unaligned_be64:
#else
get_unaligned_le64:
#endif
	BNOP	.S2	B3,5
 ||	LDNDW	.D1T1	*A4,A5:A4

; u32 get_unaligned_le32(void *ptr, int size);
; u32 get_unaligned_be32(void *ptr, int size);
;
#ifdef CONFIG_CPU_BIG_ENDIAN
get_unaligned_le32:
#else
get_unaligned_be32:
#endif
	LDNW	.D1T1	*A4,A4
	BNOP	.S2	B3,3
	SWAP2	.S1	A4,A0
	SWAP4	.L1	A0,A4

#ifdef CONFIG_CPU_BIG_ENDIAN
get_unaligned_be32:
#else
get_unaligned_le32:
#endif
	BNOP	.S2	B3,5
 ||	LDNW	.D1T1	*A4,A4


; void put_unaligned_le64(u64 val, void *ptr);
; void put_unaligned_be64(u64 val, void *ptr);
;
#ifdef CONFIG_CPU_BIG_ENDIAN
put_unaligned_le64:
#else
put_unaligned_be64:
#endif
	B	.S2	B3
	MV	.D2X	A4,B0
	SWAP2	.S2	B0,B1
 ||	SWAP2	.S1	A5,A0
	SWAP4	.L2	B1,B0
 ||	SWAP4	.L1	A0,A4
	MV	.D1X	B0,A5
	STNDW	.D2T1	A5:A4,*B4

#ifdef CONFIG_CPU_BIG_ENDIAN
put_unaligned_be64:
#else
put_unaligned_le64:
#endif
	BNOP	.S2	B3,5
 ||	STNDW	.D2T1	A5:A4,*B4

; void put_unaligned_le32(u32 val, void *ptr);
; void put_unaligned_be32(u32 val, void *ptr);
;
#ifdef CONFIG_CPU_BIG_ENDIAN
put_unaligned_le32:
#else
put_unaligned_be32:
#endif
	BNOP	.S2	B3,2
	SWAP2	.S1	A4,A5
	SWAP4	.L1	A5,A4
	STNW	.D2T1	A4,*B4

#ifdef CONFIG_CPU_BIG_ENDIAN
put_unaligned_be32:
#else
put_unaligned_le32:
#endif
	BNOP	.S2	B3,5
 ||	STNW	.D2T1	A4,*B4

