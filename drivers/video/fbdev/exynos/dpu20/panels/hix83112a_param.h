/* linux/drivers/video/decon_display/hix83112a_param.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __HIX83112A_PARAM_H__
#define __HIX83112A_PARAM_H__

/*
R51 00 00  //SEQ_CMD_0
R53 24     //SEQ_CMD_1
R55 00     //SEQ_CMD_2
R11 00     //SEQ_SLEEP_OUT
R29 00     //SEQ_DISPLAY_ON
*/

/* MIPI commands list */
static const unsigned char SEQ_SLEEP_OUT[] = {
	0x11,
};

static const unsigned char SEQ_DISPLAY_ON[] = {
	0x29,
};

static const unsigned char SEQ_DISPLAY_OFF[] = {
	0x28,
};

static const unsigned char SEQ_SLEEP_IN[] = {
	0x10,
};

static const unsigned char SEQ_CMD_0[] = {
	0x51, 0x00, 0x00
};

static const unsigned char SEQ_CMD_1[] = {
	0x53, 0x24
};

static const unsigned char SEQ_CMD_2[] = {
	0x55, 0x01
};
static const unsigned char SEQ_PASSWORD[] = {
	0xB9, 0x83, 0x11, 0x2A
};
static const unsigned char SEQ_CMD_3[] = {
	0xB2,0x00,0x02,0x00,0x90,0xD8,0x00,0x08,0x19,0xEE,0x11,0x01,0x00,0x15,0xA3,0x07
};
static const unsigned char SEQ_CMD_4[] = {
	0xE7,0x0E,0x0E,0x1E,0x69,0x1C,0x69,0x00,0x50,0x02,0x02,0x00,0x00,0x02,0x02,0x02,0x05,0x14,0x14,0x32,0xB9,0x23,0xB9,0x08
};
static const unsigned char SEQ_CMD_5[] = {
	0xE9,0xC3
};
static const unsigned char SEQ_CMD_6[] = {
	0xCB,0xD1,0xDD
};
static const unsigned char SEQ_CMD_7[] = {
	0xE9,0x3F
};
static const unsigned char SEQ_OTP_DISABLE[] = {
	0xCF, 0x00, 0x14, 0x00, 0xC0
};
#endif /* __HIX83112A_PARAM_H__ */
