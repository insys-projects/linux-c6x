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

#ifndef __SGMII_H_
#define __SGMII_H_

struct sgmii_config {
	unsigned int loopback;
	unsigned int master;
	unsigned int autoneg;
	unsigned int txconfig;
	unsigned int rxconfig;
	unsigned int auxconfig;
};

extern int keystone_sgmii_reset(int port);
extern int keystone_sgmii_config(int port, struct sgmii_config *config);
extern int keystone_sgmii_init(void);

#endif /* __SGMII_H_ */
