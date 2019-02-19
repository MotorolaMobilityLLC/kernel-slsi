/* linux/drivers/video/decon_display/nt36672a_param.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * Hwangjae Lee <hj-yo.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __NOV36672A_PARAM_H__
#define __NOV36672A_PARAM_H__

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
	0x51, 0x00
};

static const unsigned char SEQ_CMD_1[] = {
	0x53, 0x24
};

static const unsigned char SEQ_CMD_2[] = {
	0x55, 0x01
};
static const unsigned char SEQ_CMD_3[] = {
	0xFF, 0x20
};
static const unsigned char SEQ_CMD_4[] = {
	0xFB, 0x01
};

static const unsigned char SEQ_CMD_5[] = {
	0x62, 0xB8
};
static const unsigned char SEQ_CMD_6[] = {
	0xFF, 0x24
};
static const unsigned char SEQ_CMD_7[] = {
	0xFB, 0x01
};

static const unsigned char SEQ_CMD_8[] = {
	0x92, 0x79
};
static const unsigned char SEQ_CMD_9[] = {
	0xFF, 0x25
};
static const unsigned char SEQ_CMD_10[] = {
	0xFB, 0x01
};
static const unsigned char SEQ_CMD_11[] = {
	0x24, 0x79
};

static const unsigned char SEQ_CMD_12[] = {
	0x25, 0x79
};

static const unsigned char SEQ_CMD_13[] = {
	0x30, 0x30
};

static const unsigned char SEQ_CMD_14[] = {
	0x38, 0x30
};

static const unsigned char SEQ_CMD_15[] = {
	0x40, 0x63
};

static const unsigned char SEQ_CMD_16[] = {
	0x4C, 0x63
};
static const unsigned char SEQ_CMD_17[] = {
	0xFF, 0x26
};
static const unsigned char SEQ_CMD_18[] = {
	0xFB, 0x01
};
static const unsigned char SEQ_CMD_19[] = {
	0x19, 0x0B
};

static const unsigned char SEQ_CMD_20[] = {
	0x1A, 0x16
};

static const unsigned char SEQ_CMD_21[] = {
	0x1E, 0x99
};
static const unsigned char SEQ_CMD_22[] = {
	0xFF, 0x10
};
static const unsigned char SEQ_CMD_23[] = {
	0xFB, 0x01
};


#endif /* __NOV36672A_GAMMA_H__ */
