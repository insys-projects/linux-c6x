/*
 *  linux/include/asm-c6x/unaligned.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *  Rewritten for 2.6.3x: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_UNALIGNED_H
#define __ASM_C6X_UNALIGNED_H

/*
 * The C64x+ can do unaligned word and dword accesses in hardware
 * using special load/store instructions. This should be rewritten
 * to use inline assembly where appropriate when building with gcc.
 */

static inline u16 __get_unaligned_le16(const u8 *p)
{
	return p[0] | p[1] << 8;
}

static inline u16 __get_unaligned_be16(const u8 *p)
{
	return p[0] << 8 | p[1];
}

static inline void __put_unaligned_le16(u16 val, u8 *p)
{
	*p++ = val;
	*p++ = val >> 8;
}

static inline void __put_unaligned_be16(u16 val, u8 *p)
{
	*p++ = val >> 8;
	*p++ = val;
}

static inline u16 get_unaligned_le16(const void *p)
{
	return __get_unaligned_le16((const u8 *)p);
}

static inline u16 get_unaligned_be16(const void *p)
{
	return __get_unaligned_be16((const u8 *)p);
}

static inline void put_unaligned_le16(u16 val, void *p)
{
	__put_unaligned_le16(val, p);
}

static inline void put_unaligned_be16(u16 val, void *p)
{
	__put_unaligned_be16(val, p);
}

extern u32 get_unaligned_le32(const void *p);
extern u32 get_unaligned_be32(const void *p);
extern u64 get_unaligned_le64(const void *p);
extern u64 get_unaligned_be64(const void *p);
extern void put_unaligned_le32(u32 val, void *p);
extern void put_unaligned_be32(u32 val, void *p);
extern void put_unaligned_le64(u64 val, void *p);
extern void put_unaligned_be64(u64 val, void *p);

/*
 * Cause a link-time error if we try an unaligned access other than
 * 1,2,4 or 8 bytes long
 */
extern int __bad_unaligned_access_size(void);

#define __get_unaligned_le(ptr) (typeof(*(ptr)))({			\
	sizeof(*(ptr)) == 1 ? *(ptr) :					\
	  (sizeof(*(ptr)) == 2 ? get_unaligned_le16((ptr)) :		\
	     (sizeof(*(ptr)) == 4 ? get_unaligned_le32((ptr)) :		\
		(sizeof(*(ptr)) == 8 ? get_unaligned_le64((ptr)) :	\
		   __bad_unaligned_access_size())));			\
	})

#define __get_unaligned_be(ptr) (__force typeof(*(ptr)))({	\
	sizeof(*(ptr)) == 1 ? *(ptr) :					\
	  (sizeof(*(ptr)) == 2 ? get_unaligned_be16((ptr)) :		\
	     (sizeof(*(ptr)) == 4 ? get_unaligned_be32((ptr)) :		\
		(sizeof(*(ptr)) == 8 ? get_unaligned_be64((ptr)) :	\
		   __bad_unaligned_access_size())));			\
	})

#define __put_unaligned_le(val, ptr) ({					\
	void *__gu_p = (ptr);						\
	switch (sizeof(*(ptr))) {					\
	case 1:								\
		*(u8 *)__gu_p = (__force u8)(val);			\
		break;							\
	case 2:								\
		put_unaligned_le16((__force u16)(val), __gu_p);		\
		break;							\
	case 4:								\
		put_unaligned_le32((__force u32)(val), __gu_p);		\
		break;							\
	case 8:								\
		put_unaligned_le64((__force u64)(val), __gu_p);		\
		break;							\
	default:							\
		__bad_unaligned_access_size();				\
		break;							\
	}								\
	(void)0; })

#define __put_unaligned_be(val, ptr) ({					\
	void *__gu_p = (ptr);						\
	switch (sizeof(*(ptr))) {					\
	case 1:								\
		*(u8 *)__gu_p = (__force u8)(val);			\
		break;							\
	case 2:								\
		put_unaligned_be16((__force u16)(val), __gu_p);		\
		break;							\
	case 4:								\
		put_unaligned_be32((__force u32)(val), __gu_p);		\
		break;							\
	case 8:								\
		put_unaligned_be64((__force u64)(val), __gu_p);		\
		break;							\
	default:							\
		__bad_unaligned_access_size();				\
		break;							\
	}								\
	(void)0; })


#ifdef _BIG_ENDIAN
#define get_unaligned	__get_unaligned_be
#define put_unaligned	__put_unaligned_be
#define get_unaligned16	get_unaligned_be16
#define get_unaligned32	get_unaligned_be32
#define get_unaligned64	get_unaligned_be64
#else
#define get_unaligned	__get_unaligned_le
#define put_unaligned	__put_unaligned_le
#define get_unaligned16	get_unaligned_le16
#define get_unaligned32	get_unaligned_le32
#define get_unaligned64	get_unaligned_le64
#endif

#endif /* __ASM_C6X_UNALIGNED_H */
