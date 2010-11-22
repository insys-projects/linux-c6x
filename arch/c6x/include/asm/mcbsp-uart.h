/*
 *  arch/c6x/include/asm/mcbsp-uart.h
 *
 *  UART bit-bang driver using McBSP
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <aurelien.jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MCBSP_UART_H
#define __ASM_ARCH_MCBSP_UART_H

#include <asm/mcbsp.h>

#define MCBSP_TTY_MINORS         128

#define MIN_BAUDRATE             38400  /* minimal allowed baudrate */
#define MAX_BAUDRATE             115200

#define MMI_OVERSAMPLING         32
#define MCBSP_TX_BITS_PER_CHAR   11
#define MCBSP_RX_BITS_PER_CHAR   10

#define MCBSP_INST               0      /* map this McMBSP number to the first UART port for console */

#define MCBSP_MAX_TX_CHARS       64     /* max number of char for McBSP transmit */
#define MCBSP_MAX_RX_CHARS       1      /* max number of char for McBSP receive */
#define MCBSP_MAX_TX_BAUD_BITS   (MCBSP_MAX_TX_CHARS * MCBSP_TX_BITS_PER_CHAR)
#define MCBSP_MAX_RX_BAUD_BITS   (MCBSP_MAX_RX_CHARS * MCBSP_RX_BITS_PER_CHAR)

#define MAX_PORT                 2      /* max number of emulated UART ports */

struct mcbsp_uart_info {
    unsigned int mcbsp_id;  /* first McBSP used for UART */
    unsigned int mcbsp_num; /* number of McBSP used as UART ports */
};

#endif /* __ASM_ARCH_MCBSP_UART_H */
