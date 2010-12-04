/* Copyright (C) 1989, 1992, 1993, 1994, 1995, 1996, 1997, 1998, 1999,
   2000, 2001, 2002  Free Software Foundation, Inc.

This code was pulled from an old (GPLv2) libgcc.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2, or (at your option) any later version.

In addition to the permissions in the GNU General Public License, the
Free Software Foundation gives you unlimited permission to link the
compiled version of this file into combinations with other programs,
and to distribute those combinations without any restriction coming
from the use of this file.  (The General Public License restrictions
do apply in other respects; for example, they cover modification of
the file, and distribution when not linked into a combine
executable.)

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License
along with GCC; see the file COPYING.  If not, write to the Free
Software Foundation, 59 Temple Place - Suite 330, Boston, MA
02111-1307, USA.  */

#include <linux/types.h>
#include <linux/bitops.h>

#ifdef CONFIG_TI_C6X_COMPILER
#define count_leading_zeros(count, x) (count) = _lmbd(1, (x))
#else
static inline unsigned __clz(unsigned x)
{
	asm(" lmbd .l1 1,%0,%0\n" : "+a"(x));
	return x;
}
#define count_leading_zeros(count, x) (count) = __clz(x)
#endif

#define W_TYPE_SIZE	32

#define __BITS4 (W_TYPE_SIZE / 4)
#define __ll_B ((uint32_t) 1 << (W_TYPE_SIZE / 2))
#define __ll_lowpart(t) ((uint32_t) (t) & (__ll_B - 1))
#define __ll_highpart(t) ((uint32_t) (t) >> (W_TYPE_SIZE / 2))


#define sub_ddmmss(sh, sl, ah, al, bh, bl) \
	do {								\
		uint32_t __x;						\
		__x = (al) - (bl);					\
		(sh) = (ah) - (bh) - (__x > (al));			\
		(sl) = __x;						\
	} while (0)

#define umul_ppmm(w1, w0, u, v)						\
	do {								\
		uint32_t __x0, __x1, __x2, __x3;			\
		uint16_t __ul, __vl, __uh, __vh;			\
									\
		__ul = __ll_lowpart(u);					\
		__uh = __ll_highpart(u);				\
		__vl = __ll_lowpart(v);					\
		__vh = __ll_highpart(v);				\
									\
		__x0 = (uint32_t) __ul * __vl;				\
		__x1 = (uint32_t) __ul * __vh;				\
		__x2 = (uint32_t) __uh * __vl;				\
		__x3 = (uint32_t) __uh * __vh;				\
									\
		__x1 += __ll_highpart(__x0);/* this can't give carry */ \
		__x1 += __x2;		/* but this indeed can */	\
		if (__x1 < __x2)	/* did we get it? */	\
			__x3 += __ll_B;	/* yes, add it in the proper pos. */ \
									\
		(w1) = __x3 + __ll_highpart(__x1);			\
		(w0) = __ll_lowpart(__x1) * __ll_B + __ll_lowpart(__x0); \
	} while (0)

#define __udiv_qrnnd_c(q, r, n1, n0, d) \
	do {								\
		uint32_t __d1, __d0, __q1, __q0;			\
		uint32_t __r1, __r0, __m;				\
		__d1 = __ll_highpart(d);				\
		__d0 = __ll_lowpart(d);					\
									\
		__r1 = (n1) % __d1;					\
		__q1 = (n1) / __d1;					\
		__m = (uint32_t) __q1 * __d0;				\
		__r1 = __r1 * __ll_B | __ll_highpart(n0);		\
		if (__r1 < __m)	{					\
			__q1--, __r1 += (d);				\
			/* i.e. we didn't get carry when adding to __r1 */ \
			if (__r1 >= (d))				\
				if (__r1 < __m)				\
					__q1--, __r1 += (d);		\
		}							\
		__r1 -= __m;						\
									\
		__r0 = __r1 % __d1;					\
		__q0 = __r1 / __d1;					\
		__m = (uint32_t) __q0 * __d0;				\
		__r0 = __r0 * __ll_B | __ll_lowpart(n0);		\
		if (__r0 < __m) {					\
			__q0--, __r0 += (d);				\
			if (__r0 >= (d))				\
				if (__r0 < __m)				\
					__q0--, __r0 += (d);		\
		}							\
		__r0 -= __m;						\
									\
		(q) = (uint32_t) __q1 * __ll_B | __q0;			\
		(r) = __r0;						\
	} while (0)

#define UDIV_NEEDS_NORMALIZATION 1
#define udiv_qrnnd __udiv_qrnnd_c

struct llstruct {
#ifdef CONFIG_CPU_BIG_ENDIAN
	uint32_t high;
	uint32_t low;
#else
	uint32_t low;
	uint32_t high;
#endif
};

typedef union {
	struct llstruct s;
	int64_t ll;
} llunion_t;

static inline uint64_t __udivmoddi4(uint64_t n, uint64_t d, uint64_t *rp)
{
	llunion_t ww;
	llunion_t nn, dd;
	llunion_t rr;
	uint32_t d0, d1, n0, n1, n2;
	uint32_t q0, q1;
	uint32_t b, bm;

	nn.ll = n;
	dd.ll = d;

	d0 = dd.s.low;
	d1 = dd.s.high;
	n0 = nn.s.low;
	n1 = nn.s.high;

#if !UDIV_NEEDS_NORMALIZATION
	if (d1 == 0) {
		if (d0 > n1) {
			/* 0q = nn / 0D */

			udiv_qrnnd(q0, n0, n1, n0, d0);
			q1 = 0;

			/* Remainder in n0.  */
		} else {
			/* qq = NN / 0d */

			if (d0 == 0)
				d0 = 1 / d0; /* Divide intentionally by zero. */

			udiv_qrnnd(q1, n1, 0, n1, d0);
			udiv_qrnnd(q0, n0, n1, n0, d0);

			/* Remainder in n0.  */
		}

		if (rp != 0) {
			rr.s.low = n0;
			rr.s.high = 0;
			*rp = rr.ll;
		}
	}

#else /* UDIV_NEEDS_NORMALIZATION */

	if (d1 == 0) {
		if (d0 > n1) {
			/* 0q = nn / 0D */

			count_leading_zeros(bm, d0);

			if (bm != 0) {
				/* Normalize, i.e. make the most significant
				   bit of the denominator set.  */

				d0 = d0 << bm;
				n1 = (n1 << bm) | (n0 >> (W_TYPE_SIZE - bm));
				n0 = n0 << bm;
			}

			udiv_qrnnd(q0, n0, n1, n0, d0);
			q1 = 0;

			/* Remainder in n0 >> bm.  */
		} else {
			/* qq = NN / 0d */

			if (d0 == 0)
				d0 = 1 / d0; /* Divide intentionally by zero. */

			count_leading_zeros(bm, d0);

			if (bm == 0) {
				/* From (n1 >= d0) /\ (the most significant bit
				   of d0 is set), conclude (the most significant
				   bit of n1 is set) /\ (the leading quotient
				   digit q1 = 1).

				   This special case is necessary, not an
				   optimization. (Shifts counts of W_TYPE_SIZE
				   are undefined.)  */

				n1 -= d0;
				q1 = 1;
			} else {
				/* Normalize.  */

				b = W_TYPE_SIZE - bm;

				d0 = d0 << bm;
				n2 = n1 >> b;
				n1 = (n1 << bm) | (n0 >> b);
				n0 = n0 << bm;

				udiv_qrnnd(q1, n1, n2, n1, d0);
			}

			/* n1 != d0...  */

			udiv_qrnnd(q0, n0, n1, n0, d0);

			/* Remainder in n0 >> bm.  */
		}

		if (rp != 0) {
			rr.s.low = n0 >> bm;
			rr.s.high = 0;
			*rp = rr.ll;
		}
	}
#endif /* UDIV_NEEDS_NORMALIZATION */

	else {
		if (d1 > n1) {
			/* 00 = nn / DD */

			q0 = 0;
			q1 = 0;

			/* Remainder in n1n0.  */
			if (rp != 0) {
				rr.s.low = n0;
				rr.s.high = n1;
				*rp = rr.ll;
			}
		} else {
			/* 0q = NN / dd */

			count_leading_zeros(bm, d1);
			if (bm == 0) {
				/* From (n1 >= d1) /\ (the most significant bit
				   of d1 is set), conclude (the most significant
				   bit of n1 is set) /\ (the quotient digit
				   q0 = 0 or 1).

				   This special case is necessary, not an
				   optimization.  */

				/* The condition on the next line takes
				   advantage of that n1 >= d1 (true due to
				   program flow).  */
				if (n1 > d1 || n0 >= d0) {
					q0 = 1;
					sub_ddmmss(n1, n0, n1, n0, d1, d0);
				} else
					q0 = 0;

				q1 = 0;

				if (rp != 0) {
					rr.s.low = n0;
					rr.s.high = n1;
					*rp = rr.ll;
				}
			} else {
				uint32_t m1, m0;
				/* Normalize.  */

				b = W_TYPE_SIZE - bm;

				d1 = (d1 << bm) | (d0 >> b);
				d0 = d0 << bm;
				n2 = n1 >> b;
				n1 = (n1 << bm) | (n0 >> b);
				n0 = n0 << bm;

				udiv_qrnnd(q0, n1, n2, n1, d1);
				umul_ppmm(m1, m0, q0, d0);

				if (m1 > n1 || (m1 == n1 && m0 > n0)) {
					q0--;
					sub_ddmmss(m1, m0, m1, m0, d1, d0);
				}

				q1 = 0;

				/* Remainder in (n1n0 - m1m0) >> bm.  */
				if (rp != 0) {
					sub_ddmmss(n1, n0, n1, n0, m1, m0);
					rr.s.low = (n1 << b) | (n0 >> bm);
					rr.s.high = n1 >> bm;
					*rp = rr.ll;
				}
			}
		}
	}

	ww.s.low = q0;
	ww.s.high = q1;
	return ww.ll;
}

uint64_t
__c6xabi_divull(uint64_t n, uint64_t d)
{
	return __udivmoddi4(n, d, (uint64_t *)0);
}

