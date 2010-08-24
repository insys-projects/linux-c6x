/*
 *  linux/include/asm-c6x/gpio.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_GPIO_H
#define __ASM_C6X_GPIO_H

#include <asm/hardware.h>

#if (defined(CONFIG_TMS320DM64X) || defined(CONFIG_TMS320C641X))
#define GPIO_BANK_REG_OFFSET 0x00
#define GPIO_BANK_REG_SIZE   0x00
#define GPIO_GPEN            0x00 /* GPIO enable register */
#define GPIO_GPDIR           0x04 /* GPIO direction register */
#define GPIO_GPVALIN         0x08 /* GPIO value in register */
#define GPIO_GPVALOUT        0x08 /* GPIO value out register */
#define GPIO_GPDH            0x10 /* GPIO delta high register */
#define GPIO_GPHM            0x14 /* GPIO high mask register */
#define GPIO_GPDL            0x18 /* GPIO delta low register */
#define GPIO_GPLM            0x1c /* GPIO low mask register */
#define GPIO_GPGC            0x20 /* GPIO global control register */
#define GPIO_GPPOL           0x24 /* GPIO interrupt polarity register */

#define GPIO_B_GPGC_GP0M     (1 << 5)
#define GPIO_B_GPGC_GPINT0M  (1 << 4)
#define GPIO_B_GPGC_GPINTPOL (1 << 2)
#define GPIO_B_GPGC_LOGIC    (1 << 1)
#define GPIO_B_GPGC_GPINTDV  (1 << 0)

#define GPIO_INT_CNT         5
#define GPIO_PIN_CNT         32
#define GPIO_OPEN_RESET      1
#endif /* CONFIG_TMS320DM64X || CONFIG_TMS320C641X */

#if (defined(CONFIG_TMS320DM644X) || defined(CONFIG_TMS320DM643X) || defined(CONFIG_TMS320DM648))
#define GPIO_BINTEN          0x08 /* GPIO interrupt per bank enable register */
#define GPIO_BANK_REG_OFFSET 0x0c
#define GPIO_BANK_REG_SIZE   0x28
#define GPIO_GPDIR           0x04 /* GPIO direction register */
#define GPIO_GPVALOUT        0x08 /* GPIO value out register */
#define GPIO_GPDHOUT         0x0c /* GPIO delta high out register */
#define GPIO_GPDLOUT         0x10 /* GPIO delta low out register */
#define GPIO_GPVALIN         0x14 /* GPIO value in register */
#define GPIO_GPDHIN          0x18 /* GPIO delta high in register */
#define GPIO_GPDHINC         0x1c /* GPIO delta high clear in register */
#define GPIO_GPDLIN          0x20 /* GPIO delta low in register */
#define GPIO_GPDLINC         0x24 /* GPIO delta low clear in register */

#define GPIO_CLEAR_EDGE      0x00
#define GPIO_RISING_EDGE     0x01
#define GPIO_FALLING_EDGE    0x02
#endif /* CONFIG_TMS320DM644X || CONFIG_TMS320DM643X || CONFIG_TMS320DM648 */

#if defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6455)
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

#endif /* CONFIG_SOC_TMS320C6472 || CONFIG_SOC_TMS320C6455 */

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

#if !(defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6455)) || defined(CONFIG_TMS320DM648)
#define GPIO_PIN16           0x00000010u
#define GPIO_PIN17           0x00000011u
#define GPIO_PIN18           0x00000012u
#define GPIO_PIN19           0x00000013u
#define GPIO_PIN20           0x00000014u
#define GPIO_PIN21           0x00000015u
#define GPIO_PIN22           0x00000016u
#define GPIO_PIN23           0x00000017u
#define GPIO_PIN24           0x00000018u
#define GPIO_PIN25           0x00000019u
#define GPIO_PIN26           0x0000001au
#define GPIO_PIN27           0x0000001bu
#define GPIO_PIN28           0x0000001cu
#define GPIO_PIN29           0x0000001du
#define GPIO_PIN30           0x0000001eu
#define GPIO_PIN31           0x0000001fu

#if (defined(CONFIG_TMS320DM644X) || defined(CONFIG_TMS320DM643X))
#define GPIO_PIN32           0x00000020u
#define GPIO_PIN33           0x00000021u
#define GPIO_PIN34           0x00000022u
#define GPIO_PIN35           0x00000023u
#define GPIO_PIN36           0x00000024u
#define GPIO_PIN37           0x00000025u
#define GPIO_PIN38           0x00000026u
#define GPIO_PIN39           0x00000027u
#define GPIO_PIN40           0x00000028u
#define GPIO_PIN41           0x00000029u
#define GPIO_PIN42           0x0000002au
#define GPIO_PIN43           0x0000002bu
#define GPIO_PIN44           0x0000002cu
#define GPIO_PIN45           0x0000002du
#define GPIO_PIN46           0x0000002eu
#define GPIO_PIN47           0x0000002fu
#define GPIO_PIN48           0x00000030u
#define GPIO_PIN49           0x00000031u
#define GPIO_PIN50           0x00000032u
#define GPIO_PIN51           0x00000033u
#define GPIO_PIN52           0x00000034u
#define GPIO_PIN53           0x00000035u
#define GPIO_PIN54           0x00000036u
#define GPIO_PIN55           0x00000037u
#define GPIO_PIN56           0x00000038u
#define GPIO_PIN57           0x00000039u
#define GPIO_PIN58           0x0000003au
#define GPIO_PIN59           0x0000003bu
#define GPIO_PIN60           0x0000003cu
#define GPIO_PIN61           0x0000003du
#define GPIO_PIN62           0x0000003eu
#define GPIO_PIN63           0x0000003fu
#define GPIO_PIN64           0x00000040u
#define GPIO_PIN65           0x00000041u
#define GPIO_PIN66           0x00000042u
#define GPIO_PIN67           0x00000043u
#define GPIO_PIN68           0x00000044u
#define GPIO_PIN69           0x00000045u
#define GPIO_PIN70           0x00000046u
#endif /* CONFIG_TMS320DM644X || CONFIG_TMS320DM643X */
#endif /* CONFIG_SOC_TMS320C6472 || CONFIG_TMS320C645X */

#if defined(CONFIG_TMS320DM643X)
#define GPIO_PIN71           0x00000047u
#define GPIO_PIN72           0x00000048u
#define GPIO_PIN73           0x00000049u
#define GPIO_PIN74           0x0000004au
#define GPIO_PIN75           0x0000004bu
#define GPIO_PIN76           0x0000004cu
#define GPIO_PIN77           0x0000004du
#define GPIO_PIN78           0x0000004eu
#define GPIO_PIN79           0x0000004fu
#define GPIO_PIN80           0x00000050u
#define GPIO_PIN81           0x00000051u
#define GPIO_PIN82           0x00000052u
#define GPIO_PIN83           0x00000053u
#define GPIO_PIN84           0x00000054u
#define GPIO_PIN85           0x00000055u
#define GPIO_PIN86           0x00000056u
#define GPIO_PIN87           0x00000057u
#define GPIO_PIN88           0x00000058u
#define GPIO_PIN89           0x00000059u
#define GPIO_PIN90           0x0000005au
#define GPIO_PIN91           0x0000005bu
#define GPIO_PIN92           0x0000005cu
#define GPIO_PIN93           0x0000005du
#define GPIO_PIN94           0x0000005eu
#define GPIO_PIN95           0x0000005fu
#define GPIO_PIN96           0x00000060u
#define GPIO_PIN97           0x00000061u
#define GPIO_PIN98           0x00000062u
#define GPIO_PIN99           0x00000063u
#define GPIO_PIN100          0x00000064u
#define GPIO_PIN101          0x00000065u
#define GPIO_PIN102          0x00000066u
#define GPIO_PIN103          0x00000067u
#define GPIO_PIN104          0x00000068u
#define GPIO_PIN105          0x00000069u
#define GPIO_PIN106          0x0000006au
#define GPIO_PIN107          0x0000006bu
#define GPIO_PIN108          0x0000006cu
#define GPIO_PIN109          0x0000006du
#define GPIO_PIN110          0x0000006eu
#endif /* CONFIG_TMS320DM643X */

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

#if (defined(CONFIG_TMS320DM64X) || defined(CONFIG_TMS320C641X))
#define GPIO_INPUT           0
#define GPIO_OUTPUT          1
#endif /* CONFIG_TMS320DM64X || CONFIG_TMS320C641X */

#if (defined(CONFIG_TMS320DM644X) || defined(CONFIG_SOC_TMS320C6455) \
	|| defined(CONFIG_TMS320DM643X) || defined(CONFIG_SOC_TMS320C6472) \
	|| defined(CONFIG_TMS320DM648))
#define GPIO_INPUT           1
#define GPIO_OUTPUT          0
#endif /* CONFIG_TMS320DM644X || CONFIG_TMS320C645X || CONFIG_TMS320DM643X || CONFIG_SOC_TMS320C6472 */

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

#if !(defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6455))
static inline void gpio_pin_enable(unsigned int id)
{
#if (defined(CONFIG_TMS320DM64X) || defined(CONFIG_TMS320C641X))
	volatile unsigned int gpen = gpio_get_reg(GPIO_GPEN);
	gpio_set_reg(GPIO_GPEN, (gpen | (1 << id)) & 0xffff);
#endif
}

static inline void gpio_pin_disable(unsigned int id)
{
#if (defined(CONFIG_TMS320DM64X) || defined(CONFIG_TMS320C641X))
	volatile unsigned int gpen = gpio_get_reg(GPIO_GPEN);
	gpio_set_reg(GPIO_GPEN, (gpen & ~(1 << id)) & 0xffff);
#endif
}

static inline unsigned int gpio_pin_direction(unsigned int id, unsigned int direction)
{
	volatile unsigned int gpreg = GPIO_BANK_REG_OFFSET +  GPIO_GPDIR +
		                      (id / 32 ) * GPIO_BANK_REG_SIZE;

	if (direction == GPIO_LOGICAL_INPUT)
                gpio_clearbit_reg(gpreg, 1 << (id % 32));
	else
                gpio_setbit_reg(gpreg, 1 << (id % 32));

	return gpio_get_reg(gpreg);
}

static inline unsigned int gpio_pin_read(unsigned int id)
{
	volatile unsigned int gpreg = GPIO_BANK_REG_OFFSET + GPIO_GPVALIN +
		                      (id / 32 ) * GPIO_BANK_REG_SIZE;

	if ((gpio_get_reg(gpreg) & (1 << (id % 32))) != 0)
		return (unsigned int)1;
	else
		return (unsigned int)0;
}

static inline void gpio_pin_write(unsigned int id, unsigned int val)
{
	volatile unsigned int gpreg = GPIO_BANK_REG_OFFSET + GPIO_GPVALOUT
		                      + (id / 32 ) * GPIO_BANK_REG_SIZE ;
	
	if (val == 0)
		gpio_clearbit_reg(gpreg, 1 << (id % 32));
	else
		gpio_setbit_reg(gpreg, 1 << (id % 32));
}

static inline unsigned int gpio_read(unsigned int bank, unsigned int mask)
{
	return (gpio_get_reg(GPIO_BANK_REG_OFFSET + GPIO_GPVALIN +
			     bank * GPIO_BANK_REG_SIZE) & mask);
}

static inline void gpio_write(unsigned int bank, unsigned int mask, unsigned int val)
{
	volatile unsigned int gpreg = GPIO_BANK_REG_OFFSET + GPIO_GPVALOUT +
		                      bank * GPIO_BANK_REG_SIZE;
	volatile unsigned int val = gpio_get_reg(gpreg);
	gpio_set_reg(gpreg, (val & ~mask) | (mask & val));
}

#if (defined(CONFIG_TMS320DM64X) || defined(CONFIG_TMS320C641X))
static inline unsigned int gpio_delta_high_get(unsigned int id)
{
	return ((gpio_get_reg(GPIO_GPDH) & 0xffff) & id);
}

static inline void gpio_delta_high_clear(unsigned int id)
{
	volatile unsigned int gpdh = gpio_get_reg(GPIO_GPDH);
	gpio_set_reg(GPIO_GPDH, gpdh & id);
}

static inline unsigned int gpio_delta_low_get(unsigned int id)
{
	return ((gpio_get_reg(GPIO_GPDL) & 0xffff) & id);
}

static inline void gpio_delta_low_clear(unsigned int id)
{
	volatile unsigned int gpdl = gpio_get_reg(GPIO_GPDL);
	gpio_set_reg(GPIO_GPDL, gpdl & id);
}

static inline void gpio_mask_high_set(unsigned int id)
{
	volatile unsigned int gphm = gpio_get_reg(GPIO_GPHM);
	gpio_set_reg(GPIO_GPHM, gphm | id);
}

static inline void gpio_mask_high_clear(unsigned int id)
{
	volatile unsigned int gphm = gpio_get_reg(GPIO_GPHM);
	gpio_set_reg(GPIO_GPHM, gphm & ~id);
}
static inline void gpio_mask_low_set(unsigned int id)
{
	volatile unsigned int gplm = gpio_get_reg(GPIO_GPLM);
	gpio_set_reg(GPIO_GPLM, gplm | id);
}

static inline void gpio_mask_low_clear(unsigned int id)
{
	volatile unsigned int gplm = gpio_get_reg(GPIO_GPLM);
	gpio_set_reg(GPIO_GPLM, gplm & ~id);
}

static inline unsigned int gpio_int_polarity(unsigned int sig, unsigned int polarity)
{
	volatile unsigned int gppol = gpio_get_reg(GPIO_GPPOL);
	unsigned int _int_table[GPIO_INT_CNT] = {
		1 << GPIO_MASK_00,
		1 << GPIO_MASK_04,
		1 << GPIO_MASK_05,
		1 << GPIO_MASK_06,
		1 << GPIO_MASK_07
	};

	if (polarity == 0)
		gpio_set_reg(GPIO_GPPOL, gppol & ~_int_table[sig]);
	else
		gpio_set_reg(GPIO_GPPOL, gppol | _int_table[sig]);

	return gpio_get_reg(GPIO_GPPOL);
}
#endif /* CONFIG_TMS320DM64X || TMS320C641X */

#if (defined(CONFIG_TMS320DM644X) || defined(CONFIG_TMS320DM643X) || defined(CONFIG_TMS320DM648))
static inline void gpio_out_delta_high_set(unsigned int id)
{
	gpio_setbit_reg(GPIO_BANK_REG_OFFSET + GPIO_GPDHOUT +
			(id / 32 ) * GPIO_BANK_REG_SIZE, 1 << (id % 32));	
}

static inline void gpio_out_delta_high_clear(unsigned int id)
{
	gpio_setbit_reg(GPIO_BANK_REG_OFFSET + GPIO_GPDLOUT +
			(id / 32 ) * GPIO_BANK_REG_SIZE, 1 << (id % 32));	
}

static inline void gpio_bank_int_enable(unsigned int id) {
	gpio_setbit_reg(GPIO_BINTEN, 1 << id);
}

static inline void gpio_bank_int_disable(unsigned int id) {
	gpio_clearbit_reg(GPIO_BINTEN, 1 << id);
}

static inline void gpio_int_edge_detection_set(unsigned int id, unsigned int trigger) {
	if (trigger & GPIO_RISING_EDGE)
		gpio_setbit_reg(GPIO_BANK_REG_OFFSET + GPIO_GPDHIN +
			(id / 32 ) * GPIO_BANK_REG_SIZE, 1 << (id % 32));
	else
		gpio_setbit_reg(GPIO_BANK_REG_OFFSET + GPIO_GPDHINC +
			(id / 32 ) * GPIO_BANK_REG_SIZE, 1 << (id % 32));
	
	if (trigger & GPIO_FALLING_EDGE)
		gpio_setbit_reg(GPIO_BANK_REG_OFFSET + GPIO_GPDLIN +
			(id / 32 ) * GPIO_BANK_REG_SIZE, 1 << (id % 32));
	else
		gpio_setbit_reg(GPIO_BANK_REG_OFFSET + GPIO_GPDLINC +
			(id / 32 ) * GPIO_BANK_REG_SIZE, 1 << (id % 32));
}
#endif /* CONFIG_TMS320DM644X  || CONFIG_TMS320DM643X */
#else  /* ! CONFIG_SOC_TMS320C6472 || CONFIG_TMS320C645X */

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
#endif /* !__ASSEMBLY__ */

#endif /* ! CONFIG_SOC_TMS320C6472 || CONFIG_TMS320C645X */
#endif /* __ASM_C6X_GPIO_H */
