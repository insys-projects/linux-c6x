#ifndef _ASM_C6X_IPC_CORE_H
#define _ASM_C6X_IPC_CORE_H

#include <linux/kernel.h>
#include <linux/types.h>

struct ipc_core_info {

	/* Inter-core interrupt methods */
	int (*ipc_request)(void (*handler)(u32, void*),
			   u32 iflags,
			   u32 ipc_num,
			   void *data);
	int (*ipc_free)(u32 ipc_num);
	int (*ipc_send)(int core_id, int ipc_num);
	int (*ipc_wait)(int ipc_num);

	/* Inter-core locks methods */
};

extern struct ipc_core_info ipc_core;

#endif /* _ASM_C6X_IPC_CORE_H */
