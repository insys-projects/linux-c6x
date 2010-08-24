#ifndef __LINUX_KBUILD_H
#define __LINUX_KBUILD_H

#ifdef __GNU__
#define DEFINE(sym, val) \
        asm volatile("\n->" #sym " %0 " #val : : "i" (val))
#else

#define __CONCAT1(a, b) __CONCAT2(b, a, b)
#define __CONCAT2(a, b, c) a ## b ## c

#define DEFINE(sym, val) \
	({						\
		extern void bar(void *);		\
		static int __CONCAT1(sym,__xx__) = val;	\
		bar(&__CONCAT1(sym,__xx__));		\
	})
#endif

#define BLANK() asm ("\n->" : : )

#define OFFSET(sym, str, mem) \
	DEFINE(sym, offsetof(struct str, mem))

#define COMMENT(x) \
	asm ("\n->#" x)

#endif
