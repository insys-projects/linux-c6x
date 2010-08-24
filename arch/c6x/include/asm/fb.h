/*
 *  linux/include/asm-c6x/fb.h
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@jaluna.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Frame buffer driver definitions for C6X
 */
#ifndef __ASM_C6X_FB_H
#define __ASM_C6X_FB_H


#define FBIOGET_SCREEN_DISPLAY  0x46A0
#define FBIOPUT_SCREEN_DISPLAY  0x46A1
#define FBIOGET_SCREEN_WINDOW   0x46A2
#define FBIOPUT_SCREEN_WINDOW   0x46A3
#define FBIOGET_SCREEN_ADDR     0x46A4
#define FBIOGET_SCREEN_UPDATE   0x46A5

#define FB_SCREEN_INVALID  0
#define FB_SCREEN_FULL     1
#define FB_SCREEN_WINDOW   2
#define FB_SCREEN_AROUND   3
#define FB_SCREEN_MISC     4
#define FB_SCREEN_MAX      5

#endif  /* __ASM_C6X_FB_H */
