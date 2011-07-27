/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
 *         Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef KEYSTONE_PA_H
#define KEYSTONE_PA_H

struct pa_config {
	/* 32 most significant bits of the mac address */
	u32  mac0_ms;
	/*
	 * 32 least significant bits of the mac address,
	 * in the 16msbs of this word
	 */
	u32  mac0_ls;
	/* 32 most significant bits of the mac address */
	u32  mac1_ms;
	/*
	 * 32 least significant bits of the mac address,
	 * in the 16 msbs of this word
	 */
	u32  mac1_ls;
	/* Receive packet queue number */
	u32  rx_qnum;
	/* Buffer used to create PA command */
	u8   *cmd_buf;
};

/* 
 * Prototypes
 */
int keystone_pa_enable(int pdsp);
int keystone_pa_disable(void);
int keystone_pa_reset(void);
int keystone_pa_config(u8 *mac_addr);

#endif /* KEYSTONE_PA_H */

