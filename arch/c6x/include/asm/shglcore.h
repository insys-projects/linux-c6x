/*
 *  linux/include/asm-c6x/shglcore.h
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
#ifndef __ASM_C6X_SHGLCORE_H
#define __ASM_C6X_SHGLCORE_H


#ifdef CONFIG_SHGLCORE

extern int comm_status_led, comm_error_led, alarm_led;

static inline void SET_COMM_STATUS_LED(int value) {
	*(volatile char *)(0x400100+0) = comm_status_led = value;
}
static inline int  GET_COMM_STATUS_LED(void) {
	return comm_status_led;
}

static inline void SET_COMM_ERROR_LED(int value) {
	*(volatile char *)(0x400100+1) = comm_error_led = value;
}
static inline int  GET_COMM_ERROR_LED(void) {
	return comm_error_led;
}

static inline void SET_ALARM_LED(int value) {
	*(volatile char *)(0x400100+2) = alarm_led = value;
}
static inline int  GET_ALARM_LED(void) {
	return alarm_led;
}
#endif /* CONFIG_SHGLCORE */
#endif /* __ASM_C6X_SHGLCORE_H */
