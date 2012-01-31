/*
 * Copyright (C) 2010, 2011 Texas Instruments Incorporated
 * Author: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#ifndef KEYSTONE_RIO_H
#define KEYSTONE_RIO_H

#include <mach/rio.h>

/*
 * Maximum message size fo RIONET 
 */
#define MACH_RIO_MAX_MSG_SIZE            1552

#ifdef __KERNEL__
/*
 * SerDes configurations
 */
struct keystone_serdes_config {
	u32 cfg_cntl;                              /* setting control register config */
	u16 serdes_cfg_pll;                        /* SerDes PLL configuration */
	u16 prescalar_srv_clk;                     /* prescalar fo ip_clk */
	u32 rx_chan_config[KEYSTONE_RIO_MAX_PORT]; /* SerDes receive channel configuration (per-port) */
	u32 tx_chan_config[KEYSTONE_RIO_MAX_PORT]; /* SerDes transmit channel configuration (per-port) */
	u32 path_mode[KEYSTONE_RIO_MAX_PORT];      /* path config for SerDes */
};

/*
 * Per board RIO devices controller configuration
 */
struct keystone_rio_board_controller_info {
        u16 ports; /* bitfield of port(s) to probe on this controller */
        u16 mode;  /* hw mode (default serdes config) */
        u16 id;    /* host id */
        u16 init;  /* initialisation method */
        u16 size;  /* RapidIO common transport system size.
		    * 0 - Small size. 256 devices.
		    * 1 - Large size, 65536 devices. */
	struct keystone_serdes_config* serdes_config;
	u16 serdes_config_num;
};

/*
 * LSU registers
 */
struct keystone_rio_lsu_reg {
        u32      reg[6];
};

extern struct rio_dbell *rio_retrieve_dbell(u32 dbell_num);

#endif /* __KERNEL__*/
#endif /* KEYSTONE_RIO_H */
