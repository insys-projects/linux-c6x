#ifndef __LINUX_COMPILER_H
#error "Please don't include <linux/compiler-ti.h> directly, include <linux/compiler.h> instead."
#endif

/* __GNUC__ is defined for TI compiler, but we need to undo some compiler-gcc.h things */
#undef  barrier
#undef  RELOC_HIDE
#undef  inline
#undef  __inline__
#undef  __inline
#undef  __deprecated
#undef  weak
#undef  __weak
#undef  __naked
#undef  __noreturn
#undef  __pure
#undef  __printf
#undef  noinline
#undef  __attribute_const__
#undef  __maybe_unused
#undef  __always_unused
#undef  __used
#undef  __must_check
#undef  __always_inline
#undef  uninitialized_var


#define barrier() asm("")

/* keep TI compiler quiet */
#undef notrace
#define notrace

#define __weak
#define __used
#define __pure
#define __section(S) __attribute__ ((section(#S)))

#define __attribute(x)

#define __same_type(a,b) (sizeof(a) == sizeof(b))
#define uninitialized_var(x) x
#define __builtin_extract_return_addr(x)	(x)

/* keep cilly quiet */
extern void *__builtin_return_address(unsigned int);
extern long __builtin_expect(long a, long b);

