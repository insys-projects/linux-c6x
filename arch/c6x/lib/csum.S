;
;  linux/arch/c6x/lib/csum.s
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

;
;unsigned int csum_partial(const unsigned char * buff,int len, unsigned int sum)
;{
;	unsigned int checksum = 0;
;	unsigned short *tosum = (unsigned short *) buff;
;
;	if (len <= 0)
;		return 0;
;
;	while (len > 1)
;  	{
;  		len -= 2;
;  		checksum += *tosum++;
;  	}
;
;  	if (len & 1)
;  		checksum += *(unsigned char*) tosum;
;
;	checksum += sum;
;
;  	while(checksum >> 16)
;  		checksum = (checksum & 0xffff) + (checksum >> 16);
;
;  	return checksum;
;}
;
; A4:	buff
; B4:	len
; A6:	sum
; return checksum in A4
;

	.global  csum_partial
	.text

csum_partial:
	EXTU	.S1	A6,0,31,A0
  [ A0] EXTU	.S1	A6,16,16,A3
||[!A0]	MV	.D1	A6,A3

	CMPGT	.L2	B4,0,B0
  [!B0]	BNOP	.S1	L5,4
  [!B0]	ZERO	.D1	A5

	CMPLT	.L2	B4,2,B0
  [ B0] BNOP	.S1	L2,5

L1:	LDHU	.D1T1	*A4++,A5
	NOP	4
	ADD	.D1	A5,A3,A3
||	SUB	.D2	B4,2,B4
	CMPLT	.L2	B4,2,B0
  [!B0]	BNOP	.S1	L1,5

L2:
	AND	.D2	1,B4,B0
  [ B0]	LDBU	.D1T1	*A4,A4
	NOP	4

#ifdef CONFIG_CPU_BIG_ENDIAN
  [ B0] SHL	.S1	A4,8,A4
#endif

  [ B0]	ADD	.D1	A4,A3,A3

	SHRU	.S1	A3,16,A0
  [!A0]	BNOP	.S1	L4,5

L3:	SHRU	.S2X	A3,16,B4
	EXTU	.S1	A3,16,16,A3
	ADD	.D1X	B4,A3,A3
	SHRU	.S1	A3,16,A0
  [ A0]	BNOP	.S1	L3,5

L4:	MV	.D1	A3,A5

L5:	RETNOP	.S2	B3,4
	MV	.D1	A5,A4

;
;unsigned int csum_partial_copy(const char *src, char * dst,
;				int len, int sum)
;
; A4:	src
; B4:	dst
; A6:	len
; B6:	sum
; return csum in A4
;

	.global  csum_partial_copy
	.text

csum_partial_copy:
	STW	.D2T1   A10,*B15--[4]
	STW	.D2T1   A11,*+B15[3]
	STW	.D2T1   A12,*+B15[2]
	STW	.D2T1   A13,*+B15[1]

	MV	.D1X	B6,A8		; given csum
	ZERO	.D1	A9		; csum
	MV	.D1X	B4,A3		; dest
||	SHRU	.S2X	A6,3,B5		; len / 8
||	MVK	.S1	-1,A10
	EXTU    .S1     A10,16,16,A10	;  mask

	;; If (len & 1) => byte
	AND	.D1	1,A6,A1

	;; If (len & 2) => half-word
	AND	.D2	2,A6,B1

	;; If (len & 4) => non aligned word
	AND	.D1	4,A6,A0
  [ A0]	LDNW	.D1T1	*A4++,A5
	NOP		4
  [ A0]	EXTU	.S1	A5,16,16,A6
  [ A0]	ADD	.D1	A6,A9,A9
||[ A0]	EXTU	.S1	A5,0,16,A6
  [ A0]	ADD	.L1	A6,A9,A9
||[ A0]	STNW	.D1T1	A5,*A3++

	;; Start to copy double words
	CMPGT	.L2	B5,0,B0
  [!B0]	BNOP	.S1	L8,4
	ZERO	.D2	B4
||	ZERO	.D1	A5

	;; Main loop
L7:	LDNDW	.D1T1	*A4++,A7:A6
	ADD	.D2	1,B4,B4
	CMPLT	.L2	B4,B5,B0
  [ B0]	B	.S1	L7

	ADD	.L1	A5,A9,A9
	EXTU	.S1	A6,0,16,A11
||	AND	.L1	A6,A10,A12
	EXTU	.S1	A7,0,16,A13
||	AND	.L1	A7,A10,A2
	ADD	.S1	A11,A12,A12
||	ADD	.L1	A13,A2,A2
	ADD	.S1	A2,A12,A5
||	STNDW	.D1T1	A7:A6,*A3++

	ADD	.L1	A5,A9,A9
L8:
	;; Check if we need the last word
	OR	.L1	A1,B1,A0
  [!A0]	BNOP	.S1	_good_size,5

	LDNW	.D1T1	*A4++,A5	; source
	LDNW	.D1T1	*A3,A6		; destination
	ZERO	.D1	A11
	ZERO	.D1	A12
	NOP		2

#ifdef CONFIG_CPU_BIG_ENDIAN

  [ B1]	SHRU		A5,16,A12	; the 16 first bits
  [ B1] SHL		A12,16,A12
  [ B1]	EXTU		A6,16,16,A7

  [ B1]	SHRU		A5,8,A11	; the next 8 bits if needed
  [ B1] SHL		A11,24,A11
  [ B1]	EXTU		A6,24,24,A10

  [!B1]	SHRU		A5,24,A11	; otherwise get the 8 first bits
  [!B1]	SHL		A11,24,A11
  [!B1]	EXTU		A6,8,8,A10

  [ B1]	OR		A10,A12,A10
  [ B1]	SHRU		A11,16,A6
  [ B1]	OR		A10,A6,A10

  [!B1]	OR		A10,A11,A10

  [ B1]	OR		A7,A12,A7

	SHRU		A12,16,A12
	SHRU		A11,16,A11

	ADD	.L1	A12,A9,A9
  [ A1]	ADD	.L1	A11,A9,A9

  [ A1]	STNW	.D1T1	A10,*A3++
  [!A1]	STNW	.D1T1	A7,*A3++

#else

  [ B1]	EXTU		A5,16,16,A12	; the 16 first bits
  [ B1]	SHRU		A6,16,A7
  [ B1] SHL		A7,16,A7

  [ B1]	EXTU		A5,8,24,A11	; the next 8 bits if needed
  [ B1]	SHRU		A6,24,A10
  [ B1]	SHL		A10,24,A10

  [!B1]	EXTU		A5,24,24,A11	; otherwise get the 8 first bits
  [!B1]	SHRU		A6,8,A10
  [!B1]	SHL		A10,8,A10

  [ B1]	OR		A10,A12,A10
  [ B1]	SHL		A11,16,A6
  [ B1]	OR		A10,A6,A10

  [!B1]	OR		A10,A11,A10

  [ B1]	OR		A7,A12,A7

	ADD	.L1	A12,A9,A9
  [ A1]	ADD	.L1	A11,A9,A9

  [ A1]	STNW	.D1T1	A10,*A3++
  [!A1]	STNW	.D1T1	A7,*A3++

#endif

_good_size:
	;; Fold the csum
	SHRU	.S2X	A9,16,B0
  [!B0]	BNOP	.S1	L10,5

L9:	SHRU	.S2X	A9,16,B4
||	EXTU	.S1	A9,16,16,A3
	ADD	.D1X	B4,A3,A9

	SHRU	.S1	A9,16,A0
  [ A0]	BNOP	.S1	L9,5

	;; Return
L10:	ADD	.D1	A8,A9,A9
	MV	.D1	A9,A4

	LDW	.D2	*+B15[1],A13
	LDW	.D2	*+B15[2],A12
	LDW	.D2	*+B15[3],A11
	B	.S2	B3
	LDW	.D2	*++B15[4],A10
	NOP		4

;
;unsigned short
;ip_fast_csum(unsigned char *iph, unsigned int ihl)
;{
;   	unsigned int checksum = 0;
;	unsigned short *tosum = (unsigned short *) iph;
;	int len;
;
;	len = ihl*4;
;
;	if (len <= 0)
;		return 0;
;
;	while(len) {
;  		len -= 2;
;  		checksum += *tosum++;
;  	}
;  	if (len & 1)
;  		checksum += *(unsigned char*) tosum;
;
;  	while(checksum >> 16)
;  		checksum = (checksum & 0xffff) + (checksum >> 16);
;
;  	return ~checksum;
;}
;
; A4:	iph
; B4:	ihl
; return checksum in A4
;

	.global  ip_fast_csum
	.text

ip_fast_csum:
	ZERO    .D1     A5
	SHL     .S2     B4,2,B0
	CMPGT   .L2     B0,0,B1
  [!B1] BNOP	.S1	L15,4
  [!B1]	ZERO	.D1	A3

  [!B0]	BNOP	.S1	L12,5

L11:	LDHU	.D1T1	*A4++,A3
	NOP	4
	ADD	.D1	A3,A5,A5
	SUB	.D2	B0,2,B0
  [ B0]	BNOP	.S1	L11,5

L12:	AND	.D2	1,B0,B0
  [ B0]	LDBU	.D1T1	*A4,A3
	NOP	4
  [ B0]	ADD	.D1	A3,A5,A5
	SHRU	.S1	A5,16,A0
  [!A0]	BNOP	.S1	L14,5

L13:	SHRU	.S2X	A5,16,B4
	EXTU	.S1	A5,16,16,A3
	ADD	.D1X	B4,A3,A5
	SHRU	.S1	A5,16,A0
  [ A0]	BNOP	.S1	L13,5

L14:	NOT	.D1	A5,A3
	EXTU	.S1	A3,16,16,A3

L15:	RETNOP	.S2	B3,4
	MV	.D1	A3,A4
