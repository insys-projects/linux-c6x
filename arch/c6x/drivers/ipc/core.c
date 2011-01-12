/*
 *  linux/arch/c6x/drivers/ipc/core.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/ipc-core.h>

static int dummy_ipc_request(void (*handler)(u32, void*),
			     u32 iflags,
			     u32 ipc_num,
			     void *data)
{
	return -ENOSYS;
}

static int dummy_ipc_free(u32 ipc_num)
{
	return -ENOSYS;
}

static int dummy_ipc_send(int core_id, int ipc_num)
{
	return -ENOSYS;
} 

static int dummy_ipc_wait(int ipc_num)
{
	return -ENOSYS;
} 

struct ipc_core_info ipc_core = {
	.ipc_request = dummy_ipc_request,
	.ipc_free    = dummy_ipc_free,
	.ipc_send    = dummy_ipc_send,
	.ipc_wait    = dummy_ipc_wait,
};

