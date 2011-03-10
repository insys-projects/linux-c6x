/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
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

#ifndef __MACH_C6X_NETCP_H
#define __MACH_C6X_NETCP_H

/*
 * SGMII registers
 */
#define SGMII_BASE_ADDRESS	{ 0x02090100, 0x02090200 }
#define SGMII_IDVER_REG(x)	(0x02090100 + (x * 0x100) + 0x000)
#define SGMII_SRESET_REG(x)	(0x02090100 + (x * 0x100) + 0x004)
#define SGMII_CTL_REG(x)	(0x02090100 + (x * 0x100) + 0x010)
#define SGMII_STATUS_REG(x)	(0x02090100 + (x * 0x100) + 0x014)
#define SGMII_MRADV_REG(x)	(0x02090100 + (x * 0x100) + 0x018)
#define SGMII_LPADV_REG(x)	(0x02090100 + (x * 0x100) + 0x020)
#define SGMII_TXCFG_REG(x)	(0x02090100 + (x * 0x100) + 0x030)
#define SGMII_RXCFG_REG(x)	(0x02090100 + (x * 0x100) + 0x034)
#define SGMII_AUXCFG_REG(x)	(0x02090100 + (x * 0x100) + 0x038)

#define SGMII_REG_STATUS_FIELD_LOCK	(1<<4)

#endif /* __MACH_C6X_NETCP_H */

