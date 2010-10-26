/*
 *  linux/include/asm-c6x/gpio.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_GPIO_H
#define __ASM_C6X_GPIO_H

#include <asm/hardware.h>

#if defined(CONFIG_SOC_TMS320C6472) ||  defined(CONFIG_SOC_TMS320C6474) || \
    defined(CONFIG_SOC_TMS320C6455) ||  defined(CONFIG_SOC_TMS320C6457)
#define GPIO_BINTEN          0x08 /* GPIO interrupt per bank enable register */
#define GPIO_GPDIR           0x10 /* GPIO direction register */
#define GPIO_GPOUTDATA       0x14 /* GPIO output data register */
#define GPIO_GPSETDATA       0x18 /* GPIO set data register */
#define GPIO_GPCLRDATA       0x1c /* GPIO clear data register */
#define GPIO_GPINDATA        0x20 /* GPIO input data register */
#define GPIO_GPSETRIS        0x24 /* GPIO set rising edge interrupt register */
#define GPIO_GPCLRRIS        0x28 /* GPIO clear rising edge interrupt register */
#define GPIO_GPSETFAL        0x2c /* GPIO set falling edge interrupt register */
#define GPIO_GPCLRFAL        0x30 /* GPIO clear falling edge interrupt register */

#define GPIO_CLEAR_EDGE      0x00
#define GPIO_RISING_EDGE     0x01
#define GPIO_FALLING_EDGE    0x02

#endif /* CONFIG_SOC_TMS320C64{55,57,72,74} */

/* Pin id definitions */
#define GPIO_PIN0            0x00000000u
#define GPIO_PIN1            0x00000001u
#define GPIO_PIN2            0x00000002u
#define GPIO_PIN3            0x00000003u
#define GPIO_PIN4            0x00000004u
#define GPIO_PIN5            0x00000005u
#define GPIO_PIN6            0x00000006u
#define GPIO_PIN7            0x00000007u
#define GPIO_PIN8            0x00000008u
#define GPIO_PIN9            0x00000009u
#define GPIO_PIN10           0x0000000au
#define GPIO_PIN11           0x0000000bu
#define GPIO_PIN12           0x0000000cu
#define GPIO_PIN13           0x0000000du
#define GPIO_PIN14           0x0000000eu
#define GPIO_PIN15           0x0000000fu

/* CPU interrupt pins */
#define GPIO_GPINT0          0
#define GPIO_GPINT4          1
#define GPIO_GPINT5          2
#define GPIO_GPINT6          3
#define GPIO_GPINT7          4

/* Interrupt polarity */
#define GPIO_RISING          0
#define GPIO_FALLING         1

/* Pin direction */
#define GPIO_LOGICAL_INPUT   0

#if defined(CONFIG_SOC_TMS320C6455) || defined(CONFIG_SOC_TMS320C6457) || \
    defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6474)
#define GPIO_INPUT           1
#define GPIO_OUTPUT          0
#endif

/* GPIO masks */
#define GPIO_MASK_00         0x00000000u
#define GPIO_MASK_01         0x00000001u
#define GPIO_MASK_02         0x00000002u
#define GPIO_MASK_03         0x00000003u
#define GPIO_MASK_04         0x00000004u
#define GPIO_MASK_05         0x00000005u
#define GPIO_MASK_06         0x00000006u
#define GPIO_MASK_07         0x00000007u
#define GPIO_MASK_08         0x00000008u
#define GPIO_MASK_09         0x00000009u
#define GPIO_MASK_10         0x0000000au
#define GPIO_MASK_11         0x0000000bu
#define GPIO_MASK_12         0x0000000cu
#define GPIO_MASK_13         0x0000000du
#define GPIO_MASK_14         0x0000000eu
#define GPIO_MASK_15         0x0000000fu

#define gpio_setbit_reg(reg, val) \
        *((volatile unsigned int *) (GPIO_REG_BASE + (reg))) |= (unsigned int) (val)
	    
#define gpio_clearbit_reg(reg, val) \
        *((volatile unsigned int *) (GPIO_REG_BASE + (reg))) &= ~((unsigned int) (val))
        
#define gpio_set_reg(reg, val) \
        *((volatile unsigned int *) (GPIO_REG_BASE + (reg))) = (unsigned int) (val)
        
#define gpio_get_reg(reg) \
        *((volatile unsigned int *) (GPIO_REG_BASE + (reg)))

#ifndef __ASSEMBLY__



static inline void gpio_bank_int_enable(void) {
	gpio_setbit_reg(GPIO_BINTEN, 1);
}

static inline void gpio_bank_int_disable(void) {
	gpio_clearbit_reg(GPIO_BINTEN, 1);
}

static inline void gpio_int_edge_detection_set(unsigned int id, unsigned int trigger) {
	if (trigger & GPIO_RISING_EDGE)
		gpio_setbit_reg(GPIO_GPSETRIS, 1 << (id % 32));
	else
		gpio_setbit_reg(GPIO_GPCLRRIS, 1 << (id % 32));
	
	if (trigger & GPIO_FALLING_EDGE)
		gpio_setbit_reg(GPIO_GPSETFAL, 1 << (id % 32));
	else
		gpio_setbit_reg(GPIO_GPCLRFAL, 1 << (id % 32));
}

static inline void gpio_pin_direction(unsigned int id, unsigned int direction) {
	if (direction == GPIO_INPUT)
		gpio_setbit_reg(GPIO_GPDIR, 1 << (id % 32));
	else
		gpio_clearbit_reg(GPIO_GPDIR, 1 << (id % 32));
}

static inline void gpio_direction(unsigned short direction_mask) {
	gpio_set_reg(GPIO_GPDIR, direction_mask);
}

static inline void gpio_direction_set(unsigned short direction_mask, unsigned short mask)
{
        unsigned int dir = gpio_get_reg(GPIO_GPDIR) & ~mask;
	gpio_set_reg(GPIO_GPDIR, dir | (direction_mask & mask));
}

static inline void gpio_pin_set(unsigned int id) {
	gpio_set_reg(GPIO_GPSETDATA, 1 << (id % 32));
}

static inline void gpio_set(unsigned short val) {
	gpio_set_reg(GPIO_GPSETDATA, val);
}

static inline void gpio_pin_clear(unsigned int id) {
	gpio_set_reg(GPIO_GPCLRDATA, 1 << (id % 32));
}

static inline void gpio_clear(unsigned short val) {
	gpio_set_reg(GPIO_GPCLRDATA, val);
}

static inline void gpio_pin_output(unsigned int id) {
	gpio_setbit_reg(GPIO_GPOUTDATA, 1 << (id % 32));
}

static inline void gpio_output(unsigned short val) {
	gpio_set_reg(GPIO_GPOUTDATA, val);
}

static inline unsigned short gpio_input(void) {
	return gpio_get_reg(GPIO_GPINDATA);
}

#endif /* __ASSEMBLY__ */

#endif /* __ASM_C6X_GPIO_H */
