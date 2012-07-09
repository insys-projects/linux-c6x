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

#include <linux/bitops.h>

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

#define DSCR_B_PERCFG0_TCP           BIT(0)
#define DSCR_B_PERCFG0_VCP           BIT(2)
#define DSCR_B_PERCFG0_EMAC          BIT(4)
#define DSCR_B_PERCFG0_TIMER0        BIT(6)
#define DSCR_B_PERCFG0_TIMER1        BIT(8)
#define DSCR_B_PERCFG0_GPIO          BIT(10)
#define DSCR_B_PERCFG0_I2C           BIT(12)
#define DSCR_B_PERCFG0_BSP0          BIT(14)
#define DSCR_B_PERCFG0_BSP1          BIT(16)
#define DSCR_B_PERCFG0_HPI           BIT(18)
#define DSCR_B_PERCFG0_PCI           BIT(20)
#define DSCR_B_PERCFG0_UTOPIA        BIT(22)

#define DSCR_B_PERCFG1_EMIFA         BIT(0)
#define DSCR_B_PERCFG1_DDR2          BIT(1)

#define DSCR_B_EMACCFG_RMIIRST       (1<<19)

#define DEVSTAT_MACSEL_OFFSET        9
#define DEVSTAT_MACSEL_MASK          3

#endif  /* CONFIG_SOC_TMS320C6455 */

#if defined(CONFIG_SOC_TMS320C6457)
#define DSCR_JTAGID                  0x02880818
#define DSCR_DEVSTAT                 0x02880820
#define DSCR_KICK0                   0x02880838
#define DSCR_KICK1                   0x0288083c
#define DSCR_BOOTADDR                0x02880840
#define DSCR_DEVCFG                  0x02880910
#define DSCR_MACID1                  0x02880914
#define DSCR_MACID2                  0x02880918
#define DSCR_PRI_ALLOC               0x0288091c
#define DSCR_WDRSTSEL                0x02880920

#define DSCR_KICK0_KEY               0x83E70B13
#define DSCR_KICK1_KEY               0x95A4F1E0

#define DSCR_B_DEVSTAT_LENDIAN       BIT(0)
#define DSCR_B_DEVSTAT_HPIWIDTH      BIT(14)
#define DSCR_B_DEVSTAT_ECLKINSEL     BIT(15)

/* Using SGMII */
#define DEVSTAT_MACSEL_OFFSET        0
#define DEVSTAT_MACSEL_MASK          0

#endif  /* CONFIG_SOC_TMS320C6457 */

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

#define DSCR_B_PERCFG0_TCP           BIT(0)
#define DSCR_B_PERCFG0_VCP           BIT(2)
#define DSCR_B_PERCFG0_EMAC          BIT(4)
#define DSCR_B_PERCFG0_TIMER0        BIT(6)
#define DSCR_B_PERCFG0_TIMER1        BIT(8)
#define DSCR_B_PERCFG0_GPIO          BIT(10)
#define DSCR_B_PERCFG0_I2C           BIT(12)
#define DSCR_B_PERCFG0_BSP0          BIT(14)
#define DSCR_B_PERCFG0_BSP1          BIT(16)
#define DSCR_B_PERCFG0_HPI           BIT(18)
#define DSCR_B_PERCFG0_PCI           BIT(20)
#define DSCR_B_PERCFG0_UTOPIA        BIT(22)

#define DSCR_B_PERCFG1_EMIFA         BIT(0)
#define DSCR_B_PERCFG1_DDR2          BIT(1)

#define DSCR_B_EMACCFG_RMIIRST       (1<<19)

/* Using SGMII */
#define DEVSTAT_MACSEL_OFFSET        0
#define DEVSTAT_MACSEL_MASK          0

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

#define DSCR_B_DEVCTL_EMAC1          BIT(12)

#define DEVSTAT_B_EMAC0_MACSEL       0x00000700
#define DEVSTAT_B_EMAC0_OFFSET       8
#define DEVSTAT_B_EMAC1_MACSEL       0x00C00000
#define DEVSTAT_B_EMAC1_OFFSET       22

#define DEVSTAT_MACSEL_OFFSET        8
#define DEVSTAT_MACSEL_MASK          7

#endif   /* CONFIG_SOC_TMS320C6472 */

#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
#define DSCR_JTAGID                  0x02620018
#define DSCR_DEVSTAT                 0x02620020
#define DSCR_KICK0                   0x02620038
#define DSCR_KICK1                   0x0262003c
#define DSCR_BOOTADDR                0x02620040
#define DSCR_BOOTCOMPLETE            0x0262013C
#define DSCR_DEVCFG                  0x0262014c
#define DSCR_MACID                   0x02620110
#define DSCR_TINPSEL                 0x02620300
#define DSCR_TOUTPSEL                0x02620304
#define DSCR_RSTMUX0                 0x02620308
#define DSCR_MAINPLLCTL0             0x02620328
#define DSCR_MAINPLLCTL1             0x0262032c
#define DSCR_DDR3PLLCTL0	     0x02620330
#define DSCR_DDR3PLLCTL1	     0x02620334
#define DSCR_PAPLLCTL0	             0x02620338
#define DSCR_SGMII_SERDES_CFGPLL     0x02620340
#define DSCR_SGMII_SERDES_CFGRX0     0x02620344
#define DSCR_SGMII_SERDES_CFGTX0     0x02620348
#define DSCR_SGMII_SERDES_CFGRX1     0x0262034C
#define DSCR_SGMII_SERDES_CFGTX1     0x02620350
#define DSCR_OBSCLKCTL               0x026203ac
#define DSCR_PRI_ALLOC               0x0288091c

#define DSCR_KICK0_KEY               0x83E70B13
#define DSCR_KICK1_KEY               0x95A4F1E0

#define DSCR_B_DEVSTAT_LENDIAN       BIT(0)
#define DSCR_B_DEVSTAT_PCIESSEN      BIT(16)

#define DSP_BOOT_ADDR(x)             (DSCR_BOOTADDR + (x * 4))

/* Using SGMII */
#define DEVSTAT_MACSEL_OFFSET        0
#define DEVSTAT_MACSEL_MASK          0

#endif  /* CONFIG_SOC_TMS320C6670 */

/*
 * MDIO interfaces
 */
#define DEVSTAT_MACSEL_MII           0
#define DEVSTAT_MACSEL_RMII          1
#define DEVSTAT_MACSEL_GMII          2
#define DEVSTAT_MACSEL_RGMII         3
#define DEVSTAT_MACSEL_S3MII         5
#define DEVSTAT_MACSEL_DISABLE       7

#endif /* __MACH_C6X_DSCR_H */

