/*
 *  arch/c6x/include/asm/swab.h
 *
 */
#ifndef __ASM_C6X_SWAB_H
#define __ASM_C6X_SWAB_H

#include <linux/types.h>

#if defined(__GNUC__) && !defined(__STRICT_ANSI__) || defined(__KERNEL__)
#  define __SWAB_64_THRU_32__
#endif

#endif
