/*
 * TI C6X GPIO Support
 *
 * Copyright (c) 2010 Texas Instruments Incorporated
 * Mark Salter <msalter@redhat.com>
 *
 * Based on arm/mach-davinci/gpio.c
 *
 * Copyright (c) 2006-2007 David Brownell
 * Copyright (c) 2007, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>

static struct tci648x_gpio chips[DIV_ROUND_UP(NR_SOC_GPIO, 32)];

/* create a non-inlined version */
static struct gpio_controller __iomem * __init gpio2controller(unsigned gpio)
{
	return __gpio_to_controller(gpio);
}

static int __init c6x_gpio_irq_setup(void);

static int c6x_direction_in(struct gpio_chip *chip, unsigned offset)
{
	struct tci648x_gpio *d = container_of(chip, struct tci648x_gpio, chip);
	struct gpio_controller *__iomem g = d->regs;
	u32 temp;

	__dint();
	temp = __raw_readl(&g->dir);
	temp |= (1 << offset);
	__raw_writel(temp, &g->dir);
	__rint();

	return 0;
}

/*
 * Read the pin's value (works even if it's set up as output);
 * returns zero/nonzero.
 *
 * Note that changes are synched to the GPIO clock, so reading values back
 * right after you've set them may give old values.
 */
static int c6x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct tci648x_gpio *d = container_of(chip, struct tci648x_gpio, chip);
	struct gpio_controller *__iomem g = d->regs;

	return (1 << offset) & __raw_readl(&g->in_data);
}

static int
c6x_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tci648x_gpio *d = container_of(chip, struct tci648x_gpio, chip);
	struct gpio_controller *__iomem g = d->regs;
	u32 temp;
	u32 mask = 1 << offset;

	__dint();
	temp = __raw_readl(&g->dir);
	temp &= ~mask;
	__raw_writel(mask, value ? &g->set_data : &g->clr_data);
	__raw_writel(temp, &g->dir);
	__rint();
	return 0;
}

/*
 * Assuming the pin is muxed as a gpio output, set its output value.
 */
static void
c6x_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tci648x_gpio *d = container_of(chip, struct tci648x_gpio, chip);
	struct gpio_controller *__iomem g = d->regs;

	__raw_writel((1 << offset), value ? &g->set_data : &g->clr_data);
}

static int __init c6x_gpio_setup(void)
{
	int i, base;

	for (i = 0, base = 0; base < NR_SOC_GPIO; i++, base += 32) {
		chips[i].chip.label = "C64X+";

		chips[i].chip.direction_input = c6x_direction_in;
		chips[i].chip.get = c6x_gpio_get;
		chips[i].chip.direction_output = c6x_direction_out;
		chips[i].chip.set = c6x_gpio_set;

		chips[i].chip.base = base;
		chips[i].chip.ngpio = NR_SOC_GPIO - base;
		if (chips[i].chip.ngpio > 32)
			chips[i].chip.ngpio = 32;

		chips[i].regs = gpio2controller(base);

		gpiochip_add(&chips[i].chip);
	}

	c6x_gpio_irq_setup();
	return 0;
}
pure_initcall(c6x_gpio_setup);

static unsigned __irq_to_mask(unsigned irq)
{
#ifdef CONFIG_SOC_TMS320C6472
	if (irq == IRQ_GPIO_START)
		return 1 << get_coreid();

	return 1 << (CORE_NUM + irq - IRQ_GPIO_START - 1);
#else
	return 1 << (irq - IRQ_GPIO_START);
#endif
}

static int c6x_gpio_irq_type(unsigned irq, unsigned trigger)
{
	struct gpio_controller *__iomem g = get_irq_chip_data(irq);
	struct irq_desc *desc = irq_to_desc(irq);
	u32 mask = __irq_to_mask(irq);

	if (trigger & ~(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	desc->status &= ~IRQ_TYPE_SENSE_MASK;
	desc->status |= trigger;

	__raw_writel(mask, (trigger & IRQ_TYPE_EDGE_FALLING)
		     ? &g->set_falling : &g->clr_falling);
	__raw_writel(mask, (trigger & IRQ_TYPE_EDGE_RISING)
		     ? &g->set_rising : &g->clr_rising);

	return 0;
}

static int c6x_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	if (offset >= NR_SOC_GPIO)
		return -ENODEV;

#ifdef CONFIG_SOC_TMS320C6472
	if (offset == get_coreid())
		return IRQ_GPIOINT;
	else if (offset < CORE_NUM)
		return -ENODEV;
	offset -= (CORE_NUM - 1);
#endif
	return IRQ_GPIO_START + offset;
}

static int __init c6x_gpio_irq_setup(void)
{
	unsigned	irq;
	struct gpio_controller	*__iomem g;
	struct irq_desc *desc;
	struct irq_chip *chip;

	/* default trigger: both edges */
	g = gpio2controller(0);
	__raw_writel(0xffff, &g->set_falling);
	__raw_writel(0xffff, &g->set_rising);

	chips[0].chip.to_irq = c6x_gpio_to_irq;
	chips[0].irq_base = IRQ_GPIO_START;

	/* set the direct IRQs up to use that irqchip */
	for (irq = IRQ_GPIO_START; irq <= IRQ_GPIO15; irq++) {
		desc = irq_to_desc(irq);
		chip = get_irq_desc_chip(desc);

		chip->set_type = c6x_gpio_irq_type;
		desc->status |= IRQ_TYPE_EDGE_BOTH;
		desc->chip_data = g;
	}

	/* allow GPIO line interrupts */
	__raw_writel(BIT(0), SOC_GPIO_BASE + 0x08);

	printk(KERN_INFO "C64x: %d gpio irqs\n", irq - IRQ_GPIO_START);

	return 0;
}
