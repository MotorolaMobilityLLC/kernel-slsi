/* drivers/video/fbdev/exynos/decon_8890/panels/nt36672a_mipi_lcd.c
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
#include "nt36672a_param.h"

#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0
#define DEFAULT_BRIGHTNESS 0

static struct dsim_device *dsim_base;
static struct backlight_device *bd;

static int nt36672a_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int nt36672a_get_backlight_level(int brightness)
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

static int nt36672a_update_brightness(int brightness)
{
	int backlightlevel;

	backlightlevel = nt36672a_get_backlight_level(brightness);

	pr_info("brightness [%d]\n", brightness);
	if (dsim_wr_data(0, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_0[0],
				brightness) < 0)
		dsim_err("fail to send SEQ_CMD_0 command.\n");

	mdelay(12);
	return 0;
}

static int nt36672a_set_brightness(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
		pr_err("Brightness should be in the range of 0 ~ 255\n");
		return -EINVAL;
	}
	nt36672a_update_brightness(brightness);

	return 0;
}

static const struct backlight_ops nt36672a_backlight_ops = {
	.get_brightness = nt36672a_get_brightness,
	.update_status = nt36672a_set_brightness,
};

static int nt36672a_probe(struct dsim_device *dsim)
{
	dsim_base = dsim;

	bd = backlight_device_register("backlight_0", NULL,
		NULL, &nt36672a_backlight_ops, NULL);
	if (IS_ERR(bd))
		pr_err("failed to register backlight device!\n");

	bd->props.max_brightness = MAX_BRIGHTNESS;
	bd->props.brightness = DEFAULT_BRIGHTNESS;

	return 0;
}

static int nt36672a_displayon(struct dsim_device *dsim)
{
	dsim_info("%s +\n", __func__);
	nt36672a_lcd_init(dsim->id, &dsim->lcd_info);
	nt36672a_lcd_enable(dsim->id);
	dsim_info("%s -\n", __func__);
	return 1;
}

static int nt36672a_suspend(struct dsim_device *dsim)
{
	return 0;
}

static int nt36672a_resume(struct dsim_device *dsim)
{
	return 0;
}

#if defined (CONFIG_EXYNOS_PANEL_CABC)
static int nt36672a_cabc_mode(struct dsim_device *dsim, int mode)
{
	int ret = 0;
	int count;
	unsigned char buf[] = {0x0, 0x0};
	unsigned char SEQ_CABC_CMD[] = {0x55, 0x00, 0x00};
	unsigned char cmd = MIPI_DCS_WRITE_POWER_SAVE; /* 0x55 */

	dsim_info("%s: CABC mode[%d] write/read\n", __func__, mode);

	switch (mode) {
	/* read */
	case CABC_READ_MODE:
		cmd = MIPI_DCS_GET_POWER_SAVE; /* 0x56 */
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, cmd, 0x1, buf);
		if (ret < 0) {
			dsim_err("CABC REG(0x%02x) read failure!\n", cmd);
			count = 0;
		} else {
			dsim_info("CABC REG(0x%02x) read success: 0x%02x\n",
					cmd, *(unsigned int *)buf & 0xFF);
			count = 1;
		}
		return count;

	/*write */
	case POWER_SAVE_OFF:
		SEQ_CABC_CMD[1] = CABC_OFF;
		break;
	case POWER_SAVE_LOW:
		SEQ_CABC_CMD[1] = CABC_USER_IMAGE;
		break;
	case POWER_SAVE_MEDIUM:
		SEQ_CABC_CMD[1] = CABC_STILL_PICTURE;
		break;
	case POWER_SAVE_HIGH:
		SEQ_CABC_CMD[1] = CABC_MOVING_IMAGE;
		break;
	default:
		dsim_err("Unavailable CABC mode(%d)!\n", mode);
		return -EINVAL;
	}
	ret = dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long)SEQ_CABC_CMD /* cmd */,
			ARRAY_SIZE(SEQ_CABC_CMD));
	if (ret < 0) {
		dsim_err("CABC write command failure!\n");
		count = 0;
	} else {
		dsim_dbg("CABC write command success!\n");
		count = ARRAY_SIZE(SEQ_CABC_CMD);
	}

	return count;
}
#endif

struct dsim_lcd_driver nt36672a_mipi_lcd_driver = {
	.probe		= nt36672a_probe,
	.displayon	= nt36672a_displayon,
	.suspend	= nt36672a_suspend,
	.resume		= nt36672a_resume,
#if defined (CONFIG_EXYNOS_PANEL_CABC)
	.cabc		= nt36672a_cabc_mode,
#endif
};
