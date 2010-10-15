/*
 *  linux/arch/c6x/platforms/include/mach/dscr.h
 *
 *  DSCR definitions for Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __MACH_C6X_DSCR_H
#define __MACH_C6X_DSCR_H

/*
 * Device State Control Registers
 */
#if defined(CONFIG_SOC_TMS320C6455)
#define DSCR_DEVSTAT                 0x02a80000
#define DSCR_PERLOCK                 0x02ac0004
#define DSCR_PERCFG0                 0x02ac0008
#define DSCR_PERSTAT0                0x02ac0014
#define DSCR_PERSTAT1                0x02ac0018
#define DSCR_EMACCFG                 0x02ac0020
#define DSCR_PERCFG1                 0x02ac002c
#define DSCR_EMUBUFPD                0x02ac0054

#define DSCR_LOCKVAL                 0x0f0a0b00

#define DSCR_B_PERCFG0_TCP           0x00000001
#define DSCR_B_PERCFG0_VCP           0x00000004
#define DSCR_B_PERCFG0_EMAC          0x00000010
#define DSCR_B_PERCFG0_TIMER0        0x00000040
#define DSCR_B_PERCFG0_TIMER1        0x00000100
#define DSCR_B_PERCFG0_GPIO          0x00000400
#define DSCR_B_PERCFG0_I2C           0x00001000
#define DSCR_B_PERCFG0_BSP0          0x00004000
#define DSCR_B_PERCFG0_BSP1          0x00010000
#define DSCR_B_PERCFG0_HPI           0x00040000
#define DSCR_B_PERCFG0_PCI           0x00100000
#define DSCR_B_PERCFG0_UTOPIA        0x00400000
#define DSCR_B_PERCFG1_EMIFA         0xc0000001
#define DSCR_B_PERCFG1_DDR2          0x00000002

#define DSCR_B_EMACCFG_RMIIRST       (1<<19)
#endif  /* CONFIG_SOC_TMS320C6455 */

#if defined(CONFIG_SOC_TMS320C6474)
#define DSCR_DEVSTAT                 0x02880804
#define DSCR_JTAGID                  0x02880814
#define DSCR_PERLOCK                 0x02ac0004
#define DSCR_PERCFG0                 0x02ac0008
#define DSCR_PERSTAT0                0x02ac0014
#define DSCR_PERSTAT1                0x02ac0018
#define DSCR_EMACCFG                 0x02ac0020
#define DSCR_PERCFG1                 0x02ac002c
#define DSCR_EMUBUFPD                0x02ac0054

#define DSCR_LOCKVAL                 0x0f0a0b00

#define DSCR_B_PERCFG0_TCP           0x00000001
#define DSCR_B_PERCFG0_VCP           0x00000004
#define DSCR_B_PERCFG0_EMAC          0x00000010
#define DSCR_B_PERCFG0_TIMER0        0x00000040
#define DSCR_B_PERCFG0_TIMER1        0x00000100
#define DSCR_B_PERCFG0_GPIO          0x00000400
#define DSCR_B_PERCFG0_I2C           0x00001000
#define DSCR_B_PERCFG0_BSP0          0x00004000
#define DSCR_B_PERCFG0_BSP1          0x00010000
#define DSCR_B_PERCFG0_HPI           0x00040000
#define DSCR_B_PERCFG0_PCI           0x00100000
#define DSCR_B_PERCFG0_UTOPIA        0x00400000
#define DSCR_B_PERCFG1_EMIFA         0xc0000001
#define DSCR_B_PERCFG1_DDR2          0x00000002

#define DSCR_B_EMACCFG_RMIIRST       (1<<19)
#endif  /* CONFIG_SOC_TMS320C6474 */

#if defined(CONFIG_SOC_TMS320C6472)
#define DSCR_DEVSTAT                 0x2a80000
#define DSCR_PRIALLOC                0x2a80004
#define DSCR_DEVID                   0x2a80008
#define DSCR_DEVCTL                  0x2a80200
#define DSCR_PERLOCK                 0x2a80204
#define DSCR_RMIIRESET0              0x2a80208
#define DSCR_RMIIRESET1              0x2a8020c
#define DSCR_HOSTPRIV                0x2a8040c
#define DSCR_PRIVPERM                0x2a8041c
#define DSCR_PRIVKEY                 0x2a80420
#define DSCR_NMIGR0                  0x2a80500
#define DSCR_NMIGR1                  0x2a80504
#define DSCR_NMIGR2                  0x2a80508
#define DSCR_NMIGR3                  0x2a8050c
#define DSCR_NMIGR4                  0x2a80510
#define DSCR_NMIGR5                  0x2a80514
#define DSCR_IPCGR0                  0x2a80540
#define DSCR_IPCGR1                  0x2a80544
#define DSCR_IPCGR2                  0x2a80548
#define DSCR_IPCGR3                  0x2a8054c
#define DSCR_IPCGR4                  0x2a80550
#define DSCR_IPCGR5                  0x2a80554
#define DSCR_IPCGRH                  0x2a8057c
#define DSCR_IPCAR0                  0x2a80580
#define DSCR_IPCAR1                  0x2a80584
#define DSCR_IPCAR2                  0x2a80588
#define DSCR_IPCAR3                  0x2a8058c
#define DSCR_IPCAR4                  0x2a80590
#define DSCR_IPCAR5                  0x2a80594
#define DSCR_IPCARH                  0x2a805bc
#define DSCR_TPMGR                   0x2a80714
#define DSCR_RSTMUX0                 0x2a80718
#define DSCR_RSTMUX1                 0x2a8071c
#define DSCR_RSTMUX2                 0x2a80720
#define DSCR_RSTMUX3                 0x2a80724
#define DSCR_RSTMUX4                 0x2a80728
#define DSCR_RSTMUX5                 0x2a8072c

#define DSCR_LOCKVAL                 0xa1e183a

#define DSCR_B_DEVCTL_EMAC1          (1<<12)

#define DEVSTAT_B_EMAC0_MACSEL       0x00000700
#define DEVSTAT_B_EMAC0_OFFSET       8
#define DEVSTAT_B_EMAC1_MACSEL       0x00C00000
#define DEVSTAT_B_EMAC1_OFFSET       22
#endif   /* CONFIG_SOC_TMS320C6474 */


/*
 * MDIO interfaces
 */
#define DEVSTAT_MACSEL_MII     0
#define DEVSTAT_MACSEL_RMII    1
#define DEVSTAT_MACSEL_GMII    2
#define DEVSTAT_MACSEL_RGMII   3
#define DEVSTAT_MACSEL_OFFSET  9
#define DEVSTAT_MACSEL_MASK    3

#endif /* __MACH_C6X_DSCR_H */

