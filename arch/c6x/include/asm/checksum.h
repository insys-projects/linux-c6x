/*
 *  linux/include/asm-c6x/checksum.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_CHECKSUM_H
#define __ASM_C6X_CHECKSUM_H

#include <asm/byteorder.h>

/*
 * Computes the checksum of a memory block at buff, length len,
 * and adds in "sum" (32-bit)
 *
 * returns a 32-bit number suitable for feeding into itself
 * or csum_tcpudp_magic
 *
 * this function must be called with even lengths, except
 * for the last fragment, which may be odd
 *
 * it's best to have buff aligned on a 32-bit boundary
 */
extern __wsum csum_partial(const void * buff, int len, __wsum sum);

/*
 * The same as csum_partial, but copies from src while it checksums
 *
 * here even more important to align src and dst on a 32-bit (or even
 * better 64-bit) boundary
 */
extern __wsum csum_partial_copy(const void *src, void *dst, int len, __wsum sum);

/*
 * This is a new version of the above that records errors it finds in *errp,
 * but continues and zeros the rest of the buffer.
 */
#define csum_partial_copy_nocheck csum_partial_copy

/*
 * The same as csum_partial_copy, but copies from user space.
 *
 * here even more important to align src and dst on a 32-bit (or even
 * better 64-bit) boundary
 */
#define csum_partial_copy_from_user(src, dst, len, sum, err_ptr) \
    csum_partial_copy(src, dst, len, sum)

/*
 * This is a version of ip_compute_csum() optimized for IP headers,
 * which always checksum on 4 octet boundaries.
 *
 */
extern __sum16 ip_fast_csum(const void *iph, unsigned int ihl);

/*
 * Fold a partial checksum
 */
static inline __sum16 csum_fold(__wsum csum)
{
	u32 sum = (u32)csum;
	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);
	return (__sum16)~sum;
}

/*
 * Computes the checksum of the TCP/UDP pseudo-header
 */
static inline __wsum
csum_tcpudp_nofold(__be32 saddr, __be32 daddr, unsigned short len,
		  unsigned short proto, __wsum sum)
{
	unsigned long long s = (__force u32)sum;

	s += (__force u32)saddr;
	s += (__force u32)daddr;
#ifdef _BIG_ENDIAN
	s += proto + len;
#else
	s += (proto + len) << 8;
#endif
	s += (s >> 32);

	return (__force __wsum)s;
}

/*
 * Computes the checksum of the TCP/UDP pseudo-header
 * returns a 16-bit checksum, already complemented
 */
static inline __sum16
csum_tcpudp_magic(__be32 saddr, __be32 daddr, unsigned short len,
		  unsigned short proto, __wsum sum)
{
	return csum_fold(csum_tcpudp_nofold(saddr, daddr, len, proto, sum));
}

/*
 * This routine is used for miscellaneous IP-like checksums, mainly
 * in icmp.c
 */
static inline __sum16
ip_compute_csum(const void *buff, int len)
{
	return (__force __sum16)~do_csum(buff, len);
}

#endif /* __ASM_C6X_CHECKSUM_H */
