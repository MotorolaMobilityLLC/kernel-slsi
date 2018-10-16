/* drivers/video/fbdev/exynos/decon_8890/panels/default_mipi_lcd.c
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2018 Samsung Electronics
 *
 * Hwangjae Lee, <hj-yo.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/delay.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>

#include "../dsim.h"
#include "lcd_ctrl.h"
#include "decon_lcd.h"
#include "default_param.h"

#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0
#define DEFAULT_BRIGHTNESS 0

static struct dsim_device *dsim_base;
static struct backlight_device *bd;


static int default_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int default_get_backlight_level(int brightness)
{
	int backlightlevel;

	switch (brightness) {
	case 0:
		backlightlevel = 0;
		break;
	case 1 ... 29:
		backlightlevel = 0;
		break;
	case 30 ... 34:
		backlightlevel = 1;
		break;
	case 35 ... 39:
		backlightlevel = 2;
		break;
	case 40 ... 44:
		backlightlevel = 3;
		break;
	case 45 ... 49:
		backlightlevel = 4;
		break;
	case 50 ... 54:
		backlightlevel = 5;
		break;
	case 55 ... 64:
		backlightlevel = 6;
		break;
	case 65 ... 74:
		backlightlevel = 7;
		break;
	case 75 ... 83:
		backlightlevel = 8;
		break;
	case 84 ... 93:
		backlightlevel = 9;
		break;
	case 94 ... 103:
		backlightlevel = 10;
		break;
	case 104 ... 113:
		backlightlevel = 11;
		break;
	case 114 ... 122:
		backlightlevel = 12;
		break;
	case 123 ... 132:
		backlightlevel = 13;
		break;
	case 133 ... 142:
		backlightlevel = 14;
		break;
	case 143 ... 152:
		backlightlevel = 15;
		break;
	case 153 ... 162:
		backlightlevel = 16;
		break;
	case 163 ... 171:
		backlightlevel = 17;
		break;
	case 172 ... 181:
		backlightlevel = 18;
		break;
	case 182 ... 191:
		backlightlevel = 19;
		break;
	case 192 ... 201:
		backlightlevel = 20;
		break;
	case 202 ... 210:
		backlightlevel = 21;
		break;
	case 211 ... 220:
		backlightlevel = 22;
		break;
	case 221 ... 230:
		backlightlevel = 23;
		break;
	case 231 ... 240:
		backlightlevel = 24;
		break;
	case 241 ... 250:
		backlightlevel = 25;
		break;
	case 251 ... 255:
		backlightlevel = 26;
		break;
	default:
		backlightlevel = 12;
		break;
	}

	return backlightlevel;
}

static int default_update_brightness(int brightness)
{
	int backlightlevel;

	backlightlevel = default_get_backlight_level(brightness);

	pr_info("brightness [%d]\n", brightness);
	if (dsim_wr_data(0, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_0[0],
				brightness) < 0)
		dsim_err("fail to send SEQ_CMD_0 command.\n");

	mdelay(12);
	return 0;
}

static int default_set_brightness(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
		pr_err("Brightness should be in the range of 0 ~ 255\n");
		return -EINVAL;
	}
	default_update_brightness(brightness);

	return 0;
}

static const struct backlight_ops default_backlight_ops = {
	.get_brightness = default_get_brightness,
	.update_status = default_set_brightness,
};

static int default_probe(struct dsim_device *dsim)
{
	int ret = 1;

	dsim_base = dsim;

	bd = backlight_device_register("backlight_0", NULL,
			NULL, &default_backlight_ops, NULL);
	if (IS_ERR(bd))
		pr_err("failed to register backlight device!\n");

	bd->props.max_brightness = MAX_BRIGHTNESS;
	bd->props.brightness = DEFAULT_BRIGHTNESS;

	return ret;
}

static int default_displayon(struct dsim_device *dsim)
{
	dsim_info("%s \n", __func__);
	return 1;
}

static int default_suspend(struct dsim_device *dsim)
{
	return 0;
}

static int default_resume(struct dsim_device *dsim)
{
	return 0;
}

struct dsim_lcd_driver default_mipi_lcd_driver = {
	.probe		= default_probe,
	.displayon	= default_displayon,
	.suspend	= default_suspend,
	.resume		= default_resume,
};
