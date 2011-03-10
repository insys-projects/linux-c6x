/*
 *  linux/arch/c6x/platforms/include/mach/edma-c6670.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _MACH_EDMA3_C6670_H
#define _MACH_EDMA3_C6670_H

/* 
 * EDMA3 controller 0 channels
 */
#define DMA0_INTC2_OUT40             6  /* INTC outputs */
#define DMA0_INTC2_OUT41             7
#define DMA0_INTC2_OUT0              8
#define DMA0_INTC2_OUT1              9
#define DMA0_INTC2_OUT2              10
#define DMA0_INTC2_OUT3              11
#define DMA0_INTC2_OUT4              12
#define DMA0_INTC2_OUT5              13
#define DMA0_INTC2_OUT6              14
#define DMA0_INTC2_OUT7              15

/* 
 * EDMA3 controller 1 channels
 */
#define DMA1_SPI_INT0                0  /* SPI */
#define DMA1_SPI_INT1                1
#define DMA1_SPI_TX                  2
#define DMA1_SPI_RX                  3
#define DMA1_I2C_RX                  4
#define DMA1_I2C_TX                  5
#define DMA1_GPIO_EVT0               6  /* GPIO */
#define DMA1_GPIO_EVT1               7
#define DMA1_GPIO_EVT2               8
#define DMA1_GPIO_EVT3               9
#define DMA1_AIF_EVT0                10 /* AIF sync event: 10 to 17 */
#define DMA1_SEM_EVT0                18 /* semaphores */
#define DMA1_SEM_EVT1                19
#define DMA1_SEM_EVT2                20
#define DMA1_SEM_EVT3                21
#define DMA1_TINT4                   22 /* timer interrupts */
#define DMA1_TINTLO4                 22
#define DMA1_TINTHI4                 23
#define DMA1_TINT5                   24
#define DMA1_TINTLO5                 24
#define DMA1_TINTHI5                 25
#define DMA1_TINT6                   26
#define DMA1_TINTLO6                 26
#define DMA1_TINTHI6                 27
#define DMA1_TINT7                   28
#define DMA1_TINTLO7                 28
#define DMA1_TINTHI7                 29
#define DMA1_INTC1_OUT2              45 /* INTC output 2 to 20 -> 45 to 63 */

/* 
 * EDMA3 controller 2 channels
 */
#define DMA2_TCP3DA_RX0              0  /* TCP3 */
#define DMA2_TCP3DA_RX1              1
#define DMA2_TCP3E_REVT              2
#define DMA2_TCP3E_WEVT              3
#define DMA2_UART_RX                 4  /* UART */
#define DMA2_UART_TX                 5
#define DMA2_GPIO_EVT0               6  /* GPIO */
#define DMA2_GPIO_EVT1               7
#define DMA2_GPIO_EVT2               8
#define DMA2_GPIO_EVT3               9
#define DMA2_VCP0_REVT               10 /* VCP */
#define DMA2_VCP0_XEVT               11
#define DMA2_VCP1_REVT               12
#define DMA2_VCP1_XEVT               13
#define DMA2_VCP2_REVT               14
#define DMA2_VCP2_XEVT               15
#define DMA2_VCP3_REVT               16
#define DMA2_VCP3_XEVT               17
#define DMA2_SEM_EVT0                18 /* semaphores */
#define DMA2_SEM_EVT1                19
#define DMA2_SEM_EVT2                20
#define DMA2_SEM_EVT3                21
#define DMA2_TINT4                   22 /* timer interrupts */
#define DMA2_TINTLO4                 22
#define DMA2_TINTHI4                 23
#define DMA2_TINT5                   24
#define DMA2_TINTLO5                 24
#define DMA2_TINTHI5                 25
#define DMA2_TINT6                   26
#define DMA2_TINTLO6                 26
#define DMA2_TINTHI6                 27
#define DMA2_TINT7                   28
#define DMA2_TINTLO7                 28
#define DMA2_TINTHI7                 29
#define DMA2_SPI_INT0                30 /* SPI */
#define DMA2_SPI_INT1                31
#define DMA2_SPI_RX                  32
#define DMA2_SPI_TX                  33
#define DMA2_TCP3DB_RX0              34 /* TCP3 */
#define DMA2_TCP3DB_RX1              35
#define DMA2_INTC1_OUT23             36 /* INTC output 23 to 44 -> 36 to 57 */

/* EDMA controllers identifier */
#define EDMA0_CTLR                   0
#define EDMA1_CTLR                   1
#define EDMA2_CTLR                   2

/* 
 * C6670 uses region indexed on core id
 */
#define MACH_EDMA_NUM_REGIONS        8
#define MACH_EDMA_REGION             (7 - get_coreid())

#define MACH_EDMA_NUM_QDMACH         8

/* 
 * TPCC0 has only 2 TC, 2 event queues, 128 params entries and 16 channels
 */
#define EDMA0_NUM_EVQUE	             2
#define EDMA0_NUM_TC	             2
#define EDMA0_NUM_PARAMENTRY         128
#define EDMA0_NUM_DMACH              16

/*
 * TPCC1 and TPCC2 have 4 TC, 4 event queues and 512 params entries
 */
#define EDMA1_NUM_EVQUE	             4
#define EDMA1_NUM_TC	             4
#define EDMA1_NUM_PARAMENTRY         512
#define EDMA2_NUM_EVQUE	             4
#define EDMA2_NUM_TC	             4
#define EDMA2_NUM_PARAMENTRY         512

/*
 * IRQ mapping
 */
#define EDMA0_IRQ_CCINT              (IRQ_TPCC0INT0 + MACH_EDMA_REGION)
#define EDMA1_IRQ_CCINT              (IRQ_TPCC1INT0 + MACH_EDMA_REGION)
#define EDMA2_IRQ_CCINT              (IRQ_TPCC2INT0 + MACH_EDMA_REGION)

#define EDMA0_IRQ_CCERRINT           IRQ_TPCC0ERRINT
#define EDMA1_IRQ_CCERRINT           IRQ_TPCC1ERRINT
#define EDMA2_IRQ_CCERRINT           IRQ_TPCC2ERRINT

/* 
 * By default we use TPCC1
 */
#define MACH_EDMA_IRQ_CCINT	     (IRQ_TPCC1INT0 + MACH_EDMA_REGION)
#define MACH_EDMA_IRQ_CCERRINT	     IRQ_TPCC1ERRINT

#define MACH_EDMA_NUM_EVQUE	     4
#define MACH_EDMA_NUM_TC	     4

#define EDMA_REGISTER_BASE           EDMA1_REGISTER_BASE	
#define EDMA_TC0_BASE		     EDMA1_TC0_BASE
#define EDMA_TC1_BASE		     EDMA1_TC1_BASE
#define EDMA_TC2_BASE		     EDMA1_TC2_BASE
#define EDMA_TC3_BASE		     EDMA1_TC3_BASE

#define IRQ_TCERRINT0	             IRQ_TPTC1ERRINT0
#define IRQ_TCERRINT1	             IRQ_TPTC1ERRINT1
#define IRQ_TCERRINT2	             IRQ_TPTC1ERRINT2
#define IRQ_TCERRINT3	             IRQ_TPTC1ERRINT3

#endif /* _MACH_EDMA3_C6670_H */
