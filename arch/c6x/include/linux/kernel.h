/* 
 * Wrapper for linux/kernel.h to avoid problems using non-GNU compiler.
 */
#ifndef _C6X_LINUX_KERNEL_H
#define _C6X_LINUX_KERNEL_H

#include "../../../../include/linux/kernel.h"

#ifndef __GNU__
/* 
 * TI compiler cannot handle MAYBE_BUILD_BUG_ON
 * without __builtin_constant_p support.
 */
#undef MAYBE_BUILD_BUG_ON
#define MAYBE_BUILD_BUG_ON(cond)

#undef BUILD_BUG_ON
#define BUILD_BUG_ON(condition)

#undef BUILD_BUG_ON_ZERO
#define BUILD_BUG_ON_ZERO(e) (sizeof(struct { }))

/* cilly gets the original (which had const before typeof) wrong */
#undef container_of
#define container_of(ptr, type, member) ({		\
	typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#endif /* !__GNU__ */

#endif
