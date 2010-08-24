/*
 *  linux/include/asm-c6x/keyboard.h
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@jaluna.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Keyboard driver definitions for C6X
 */
#ifndef __ASM_C6X_KEYBOARD_H
#define __ASM_C6X_KEYBOARD_H

#include <linux/kd.h>
#include <linux/pm.h>

/*
 * We provide a unified keyboard interface when in VC_MEDIUMRAW
 * mode.  This means that all keycodes must be common between
 * all supported keyboards.  This unfortunately puts us at odds
 * with the PC keyboard interface chip... but we can't do anything
 * about that now.
 */
#ifdef __KERNEL__

static inline int kbd_setkeycode(unsigned int sc, unsigned int kc)
{
        int ret = -EINVAL;

        return ret;
}

static inline int kbd_getkeycode(unsigned int sc)
{
        int ret = -EINVAL;

        return ret;
}

static inline void kbd_leds(unsigned char leds)
{
}

static inline int k_translate(unsigned char c1, unsigned char * p_c, char c1)
{
        int ret = -EINVAL;

        return ret;
}

static inline char k_unexpected_up(unsigned char c1)
{
        char ret = '0';

        return ret;
}

static inline void k_leds(unsigned char c1)
{
}

static inline void kbd_init_hw(void)
{
}

/* resource allocation */
#define kbd_request_region()
#define kbd_request_irq(handler)

/* How to access the keyboard macros on this platform.  */
#define kbd_read_input() inb(KBD_DATA_REG)
#define kbd_read_status() inb(KBD_STATUS_REG)
#define kbd_write_output(val) outb(val, KBD_DATA_REG)
#define kbd_write_command(val) outb(val, KBD_CNTL_REG)

extern int k_sysrq_key;
extern unsigned char *k_sysrq_xlate;

#define SYSRQ_KEY               k_sysrq_key
#define kbd_sysrq_xlate         k_sysrq_xlate
#define kbd_translate           k_translate
#define kbd_unexpected_up       k_unexpected_up

#endif /* __KERNEL__ */

#endif /* __ASM_C6X_KEYBOARD_H */
