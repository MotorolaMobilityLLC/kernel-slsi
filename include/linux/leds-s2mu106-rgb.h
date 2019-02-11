/*
 * leds-s2mu106-rgb.h - RGB-led driver for Samsung S2MU106
 *
 * Copyright (C) 2018 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_S2MU106_RGB_H__
#define __LEDS_S2MU106_RGB_H__

#define MASK(width, shift)	(((0x1 << (width)) - 1) << shift)
#define ABS(x) (((x)<0) ? -(x) : (x))

#define S2MU106_RGB_BRIGHTNESS_LED 1
#define S2MU106_RGB_LED_MAX	4

#define S2MU106_RGB_CURR_MAX	255 // 0.1mA unit
#define S2MU106_RGB_RAMP_MAX	2200 // 1ms unit
#define S2MU106_RGB_ON_MAX	3250 // 1ms unit
#define S2MU106_RGB_OFF_MAX0	12000 // 1ms unit
#define S2MU106_RGB_OFF_MAX1	3250 // 1ms unit

#define S2MU106_RGB_LED_EN	0x43
#define S2MU106_RGB_LED1_CURR	0x44
#define S2MU106_RGB_LED2_CURR	0x45
#define S2MU106_RGB_LED3_CURR	0x46
#define S2MU106_RGB_LED4_CURR	0x47
#define S2MU106_RGB_LED1_RAMP	0x48
#define S2MU106_RGB_LED1_DUR	0x49
#define S2MU106_RGB_LED2_RAMP	0x4A
#define S2MU106_RGB_LED2_DUR	0x4B
#define S2MU106_RGB_LED3_RAMP	0x4C
#define S2MU106_RGB_LED3_DUR	0x4D
#define S2MU106_RGB_LED4_RAMP	0x4E
#define S2MU106_RGB_LED4_DUR	0x4F
#define S2MU106_RGB_LED_CTRL0	0x51
#define S2MU106_RGB_LED_CTRL1	0x52

#define S2MU106_RGB_LEDX_RAMP_UP_MASK	MASK(4,4)
#define S2MU106_RGB_LEDX_RAMP_DOWN_MASK	MASK(4,0)

#define S2MU106_RGB_LEDX_ON_DUR_MASK	MASK(4,4)
#define S2MU106_RGB_LEDX_OFF_DUR_MASK	MASK(4,0)

#define S2MU106_RGB_OFF_DUR_MODE_MASK	MASK(4,4)
#define S2MU106_RGB_OFF_DUR_MODE_NUM	2
#define S2MU106_RGB_ONOFF_TIME_NUM	16

int s2mu106_off_time[S2MU106_RGB_OFF_DUR_MODE_NUM]
					[S2MU106_RGB_ONOFF_TIME_NUM] = {
	{0, 500, 1000, 1500, 2000,
	2500, 3000, 3500, 4000, 4500,
	5000, 6000, 7000, 8000, 10000,
	12000},
	{100, 200, 300, 400, 500,
	750, 1000, 1250, 1500, 1750,
	2000, 2250, 2500, 2750, 3000,
	3250},
};

u8 s2mu106_rgb_off_dur_mode_mask[] = {
	0,	// not used
	MASK(1,7),
	MASK(1,6),
	MASK(1,5),
	MASK(1,4),
};

u8 s2mu106_rgb_dur_reg[] = {
	S2MU106_RGB_LED1_DUR,	// not used
	S2MU106_RGB_LED1_DUR,
	S2MU106_RGB_LED2_DUR,
	S2MU106_RGB_LED3_DUR,
	S2MU106_RGB_LED4_DUR,
};

u8 s2mu106_rgb_ramp_reg[] = {
	S2MU106_RGB_LED1_RAMP,	// not used
	S2MU106_RGB_LED1_RAMP,
	S2MU106_RGB_LED2_RAMP,
	S2MU106_RGB_LED3_RAMP,
	S2MU106_RGB_LED4_RAMP,
};

u8 s2mu106_rgb_curr_reg[] = {
	S2MU106_RGB_LED1_CURR,	// not used
	S2MU106_RGB_LED1_CURR,
	S2MU106_RGB_LED2_CURR,
	S2MU106_RGB_LED3_CURR,
	S2MU106_RGB_LED4_CURR,
};

u8 s2mu106_rgb_led_en_mask[] = {
	0,	//	not used
	MASK(2,6),
	MASK(2,4),
	MASK(2,2),
	MASK(2,0),
};

u8 s2mu106_rgb_led_en_shift[] = {
	0,	//	not used
	6,
	4,
	2,
	0,
};

#define S2MU106_RGB_MODE_OFF	0x00
#define S2MU106_RGB_MODE_CONST	0x01
#define S2MU106_RGB_MODE_TOGGLE	0x02
#define S2MU106_RGB_MODE_MAX	0x02

char *s2mu106_rgb_mode_string[] = {
	"OFF",
	"CONST",
	"TOGGLE",
};

struct s2mu106_rgb_led {
	int id;
	u32 curr;
	u32 timer;
	u8 mode;
};

struct s2mu106_rgb_platform_data {
	struct s2mu106_rgb_led *led;
	int led_num;

	u32 default_current;
	u32 max_current;
	int def_off_dur_mode;
};

struct s2mu106_rgb_data {
	struct s2mu106_rgb_platform_data *pdata;
	struct s2mu106_rgb_led led[S2MU106_RGB_LED_MAX];
	struct led_classdev cdev;
	struct device *dev;

	int off_dur_mode;

	struct i2c_client *i2c;
};

#endif
