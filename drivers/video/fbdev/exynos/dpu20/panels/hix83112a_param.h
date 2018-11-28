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

#endif /* __HIX83112A_PARAM_H__ */
