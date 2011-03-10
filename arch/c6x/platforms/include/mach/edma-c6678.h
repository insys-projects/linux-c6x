/*
 *  linux/arch/c6x/platforms/include/mach/edma-c6678.h
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

#ifndef _MACH_EDMA3_C6678_H
#define _MACH_EDMA3_C6678_H

/* 
 * EDMA3 controller 0 channels
 */
#define DMA0_TINT8                   0  /* timer interrupts */
#define DMA0_TINTLO8                 0
#define DMA0_TINTHI8                 1
#define DMA0_TINT9                   2
#define DMA0_TINTLO9                 2
#define DMA0_TINTHI9                 3
#define DMA0_TINT10                  4
#define DMA0_TINTLO10                4
#define DMA0_TINTHI10                5
#define DMA0_TINT11                  6
#define DMA0_TINTLO11                6
#define DMA0_TINTHI11                7
#define DMA0_INTC3_OUT0              8  /* INTC outputs */
#define DMA0_INTC3_OUT1              9
#define DMA0_INTC3_OUT2              10
#define DMA0_INTC3_OUT3              11
#define DMA0_INTC3_OUT4              12
#define DMA0_INTC3_OUT5              13
#define DMA0_INTC3_OUT6              14
#define DMA0_INTC3_OUT7              15

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
#define DMA1_GPIO_EVT4               10
#define DMA1_GPIO_EVT5               11
#define DMA1_GPIO_EVT6               12
#define DMA1_GPIO_EVT7               13
#define DMA1_SEM_EVT0                14 /* semaphores */
#define DMA1_SEM_EVT1                15
#define DMA1_SEM_EVT2                16
#define DMA1_SEM_EVT3                17
#define DMA1_SEM_EVT4                18
#define DMA1_SEM_EVT5                19
#define DMA1_SEM_EVT6                20
#define DMA1_SEM_EVT7                21
#define DMA1_TINT8                   22 /* timer interrupts */
#define DMA1_TINTLO8                 22
#define DMA1_TINTHI8                 23
#define DMA1_TINT9                   24
#define DMA1_TINTLO9                 24
#define DMA1_TINTHI9                 25
#define DMA1_TINT10                  26
#define DMA1_TINTLO10                26
#define DMA1_TINTHI10                27
#define DMA1_TINT11                  28
#define DMA1_TINTLO11                28
#define DMA1_TINTHI11                29
#define DMA1_TINT12                  30
#define DMA1_TINTLO12                30
#define DMA1_TINTHI12                31
#define DMA1_TINT13                  32
#define DMA1_TINTLO13                32
#define DMA1_TINTHI13                33
#define DMA1_TINT14                  34
#define DMA1_TINTLO14                34
#define DMA1_TINTHI14                35
#define DMA1_TINT15                  36
#define DMA1_TINTLO15                36
#define DMA1_TINTHI15                37
#define DMA1_INTC2_OUT44             38 /* INTC outputs */
#define DMA1_INTC2_OUT45             39
#define DMA1_INTC2_OUT46             40
#define DMA1_INTC2_OUT47             41
#define DMA1_INTC2_OUT0              42 /* 0 to 21 -> 42 to 63 */

/* 
 * EDMA3 controller 2 channels
 */
#define DMA2_SPI_INT0                0  /* SPI */
#define DMA2_SPI_INT1                1
#define DMA2_SPI_TX                  2
#define DMA2_SPI_RX                  3
#define DMA2_I2C_RX                  4
#define DMA2_I2C_TX                  5
#define DMA2_GPIO_EVT0               6  /* GPIO */
#define DMA2_GPIO_EVT1               7
#define DMA2_GPIO_EVT2               8
#define DMA2_GPIO_EVT3               9
#define DMA2_GPIO_EVT4               10
#define DMA2_GPIO_EVT5               11
#define DMA2_GPIO_EVT6               12
#define DMA2_GPIO_EVT7               13
#define DMA2_SEM_EVT0                14 /* semaphores */
#define DMA2_SEM_EVT1                15
#define DMA2_SEM_EVT2                16
#define DMA2_SEM_EVT3                17
#define DMA2_SEM_EVT4                18
#define DMA2_SEM_EVT5                19
#define DMA2_SEM_EVT6                20
#define DMA2_SEM_EVT7                21
#define DMA2_TINT8                   22 /* timer interrupts */
#define DMA2_TINTLO8                 22
#define DMA2_TINTHI8                 23
#define DMA2_TINT9                   24
#define DMA2_TINTLO9                 24
#define DMA2_TINTHI9                 25
#define DMA2_TINT10                  26
#define DMA2_TINTLO10                26
#define DMA2_TINTHI10                27
#define DMA2_TINT11                  28
#define DMA2_TINTLO11                28
#define DMA2_TINTHI11                29
#define DMA2_TINT12                  30
#define DMA2_TINTLO12                30
#define DMA2_TINTHI12                31
#define DMA2_TINT13                  32
#define DMA2_TINTLO13                32
#define DMA2_TINTHI13                33
#define DMA2_TINT14                  34
#define DMA2_TINTLO14                34
#define DMA2_TINTHI14                35
#define DMA2_TINT15                  36
#define DMA2_TINTLO15                36
#define DMA2_TINTHI15                37
#define DMA2_INTC2_OUT48             38 /* INTC outputs */
#define DMA2_INTC2_OUT49             39
#define DMA2_UART_RX                 40 /* UART */
#define DMA2_UART_TX                 41
#define DMA2_INTC2_OUT22             38 /* INTC output 22 to 43 -> 38 to 63 */

/* EDMA controllers identifier */
#define EDMA0_CTLR                   0
#define EDMA1_CTLR                   1
#define EDMA2_CTLR                   2

/* 
 * C6678 uses region indexed on core id
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

#endif /* _MACH_EDMA3_C6678_H */
