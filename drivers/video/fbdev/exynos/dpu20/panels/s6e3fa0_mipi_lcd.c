/* drivers/video/exynos/panels/s6e3fa0_mipi_lcd.c
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2014 Samsung Electronics
 *
 * Haowei Li, <haowei.li@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>
#include <linux/platform_device.h>

#include "s6e3fa0_param.h"
#include "lcd_ctrl.h"
#include "decon_lcd.h"
#include "../dsim.h"

#define MAX_BRIGHTNESS		255
/* set the minimum brightness value to see the screen */
#define MIN_BRIGHTNESS		0
#define DEFAULT_BRIGHTNESS	170
#define CMD_SIZE		34

unsigned char set_brightness[2] = {0x51, 0x7F};
int backlightlevel_log;

#if defined(CONFIG_EXYNOS_PANEL_CABC)
struct panel_device {
	struct device *dev;
	struct dsim_device *dsim;
	struct mutex lock;
	int cabc_mode;
};

struct panel_device *panel_drvdata;
struct class *panel_class;
#endif

static int s6e3fa0_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int s6e3fa0_get_backlight_level(int brightness)
{
	int backlightlevel;

	switch (brightness) {
	case 0:
		backlightlevel = 0;
		break;
	case 1 ... 10:
		backlightlevel = 10;
		break;
	case 11 ... 15:
		backlightlevel = 55;
		break;
	case 16 ... 20:
		backlightlevel = 95;
		break;
	case 21 ... 25:
		backlightlevel = 125;
		break;
	case 26 ... 30:
		backlightlevel = 160;
		break;
	case 31 ... 35:
		backlightlevel = 165;
		break;
	case 36 ... 40:
		backlightlevel = 165;
		break;
	case 41 ... 45:
		backlightlevel = 170;
		break;
	case 46 ... 50:
		backlightlevel = 170;
		break;
	case 51 ... 55:
		backlightlevel = 175;
		break;
	case 56 ... 60:
		backlightlevel = 175;
		break;
	case 61 ... 65:
		backlightlevel = 180;
		break;
	case 66 ... 70:
		backlightlevel = 180;
		break;
	case 71 ... 75:
		backlightlevel = 185;
		break;
	case 76 ... 80:
		backlightlevel = 185;
		break;
	case 81 ... 85:
		backlightlevel = 190;
		break;
	case 86 ... 90:
		backlightlevel = 190;
		break;
	case 91 ... 95:
		backlightlevel = 195;
		break;
	case 96 ... 100:
		backlightlevel = 195;
		break;
	case 101 ... 105:
		backlightlevel = 200;
		break;
	case 106 ... 110:
		backlightlevel = 210;
		break;
	case 111 ... 115:
		backlightlevel = 225;
		break;
	case 116 ... 120:
		backlightlevel = 230;
		break;
	case 121 ... 125:
		backlightlevel = 230;
		break;
	case 126 ... 130:
		backlightlevel = 235;
		break;
	case 131 ... 135:
		backlightlevel = 235;
		break;
	case 136 ... 140:
		backlightlevel = 240;
		break;
	case 141 ... 145:
		backlightlevel = 240;
		break;
	case 146 ... 150:
		backlightlevel = 245;
		break;
	case 151 ... 155:
		backlightlevel = 245;
		break;
	case 156 ... 160:
		backlightlevel = 250;
		break;
	case 161 ... 165:
		backlightlevel = 253;
		break;
	case 166 ... 170:
		backlightlevel = 253;
		break;
	case 171 ... 175:
		backlightlevel = 254;
		break;
	case 176 ... 180:
		backlightlevel = 254;
		break;
	case 181 ... 255:
		backlightlevel = 255;
		break;
	default:
		backlightlevel = 127;
		break;
	}

	return backlightlevel;
}

static int s6e3fa0_update_brightness(struct dsim_device *dsim, int brightness)
{
	int backlightlevel;

	dsim_dbg("%s +\n", __func__);

	backlightlevel = s6e3fa0_get_backlight_level(brightness);

	set_brightness[1] = backlightlevel;

	if (backlightlevel_log != backlightlevel)
		pr_info("brightness: %d -> %d\n", brightness, backlightlevel);

	backlightlevel_log = backlightlevel;

	if (brightness >= 0) {
		/* DO update brightness using dsim_wr_data */
		dsim_wr_data(0, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long)set_brightness, 2);
	} else {
		/* DO update brightness using dsim_wr_data */
		/* backlight_off ??? */
		return -EINVAL;
	}

	dsim_dbg("%s -(brightness:%d)\n", __func__, brightness);

	return 0;
}

static int s6e3fa0_set_brightness(struct backlight_device *bd)
{
	struct dsim_device *dsim;
	int brightness = bd->props.brightness;

	dsim = get_dsim_drvdata(0);

	if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
		printk(KERN_ALERT "Brightness should be in the range of 0 ~ 255\n");
		return -EINVAL;
	}

	s6e3fa0_update_brightness(dsim, brightness);
	return 1;
}

static const struct backlight_ops s6e3fa0_backlight_ops = {
	.get_brightness = s6e3fa0_get_brightness,
	.update_status = s6e3fa0_set_brightness,
};

#if defined(CONFIG_EXYNOS_PANEL_CABC)
static int s6e3fa0_cabc_mode(struct dsim_device *dsim, int mode)
{
	int ret = 0;
	int count;
	unsigned char buf[] = {0x0, 0x0};
	unsigned char SEQ_CABC_CMD[] = {0x55, 0x00, 0x00};
	unsigned char cmd = MIPI_DCS_WRITE_POWER_SAVE; /* 0x55 */

	dsim_dbg("%s: CABC mode[%d] write/read\n", __func__, mode);

	switch (mode) {
	/* read */
	case CABC_READ_MODE:
		cmd = MIPI_DCS_GET_POWER_SAVE; /* 0x56 */
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, cmd, 0x1, buf);
		if (ret < 0) {
			pr_err("CABC REG(0x%02X) read failure!\n", cmd);
			count = 0;
		} else {
			pr_info("CABC REG(0x%02X) read success: 0x%02x\n",
				cmd, *(unsigned int *)buf & 0xFF);
			count = 1;
		}
		return count;

	/* write */
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
		pr_err("Unavailable CABC mode(%d)!\n", mode);
		return -EINVAL;
	}

	ret = dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long)SEQ_CABC_CMD /*cmd*/,
			ARRAY_SIZE(SEQ_CABC_CMD));
	if (ret < 0) {
		pr_err("CABC write command failure!\n");
		count = 0;
	} else {
		dsim_dbg("CABC write command success!\n");
		count = ARRAY_SIZE(SEQ_CABC_CMD);
	}

	return count;
}

static ssize_t s6e3fa0_panel_cabc_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int ret = 0;
	struct panel_device *panel = dev_get_drvdata(dev);

	mutex_lock(&panel->lock);

	ret = s6e3fa0_cabc_mode(panel->dsim, CABC_READ_MODE);

	mutex_unlock(&panel->lock);

	count = snprintf(buf, PAGE_SIZE, "cabc_mode = %d, ret = %d\n",
			panel->cabc_mode, ret);

	return count;
}

static ssize_t s6e3fa0_panel_cabc_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int value = 0;
	struct panel_device *panel = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&panel->lock);
	panel->cabc_mode = value;
	mutex_unlock(&panel->lock);

	pr_info("%s: %d\n", __func__, value);

	s6e3fa0_cabc_mode(panel->dsim, panel->cabc_mode);

	return count;
}

static DEVICE_ATTR(cabc_mode, 0660, s6e3fa0_panel_cabc_mode_show,
			s6e3fa0_panel_cabc_mode_store);

static struct attribute *panel_attrs[] = {
	&dev_attr_cabc_mode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(panel);
#endif

static int s6e3fa0_probe(struct dsim_device *dsim)
{
	int ret = 1;
	struct panel_device *panel;
	static unsigned int panel_no;

	dsim->bd = backlight_device_register("backlight_0", dsim->dev,
		NULL, &s6e3fa0_backlight_ops, NULL);
	if (IS_ERR(dsim->bd))
		printk(KERN_ALERT "failed to register backlight device!\n");

	dsim->bd->props.max_brightness = MAX_BRIGHTNESS;
	dsim->bd->props.brightness = DEFAULT_BRIGHTNESS;

#if defined(CONFIG_EXYNOS_PANEL_CABC)
	panel = kzalloc(sizeof(struct panel_device), GFP_KERNEL);
	if (!panel) {
		pr_err("failed to allocate panel\n");
		ret = -ENOMEM;
		goto exit0;
	}

	panel_drvdata = panel;

	panel->dsim = dsim;
	panel->cabc_mode = 0;

	if (IS_ERR_OR_NULL(panel_class)) {
		panel_class = class_create(THIS_MODULE, "panel");
		if (IS_ERR_OR_NULL(panel_class)) {
			pr_err("failed to create panel class\n");
			ret = -EINVAL;
			goto exit1;
		}

		panel_class->dev_groups = panel_groups;
	}

	panel->dev = device_create(panel_class, dsim->dev, 0,
			&panel, !panel_no ? "panel" : "panel%d", panel_no);
	if (IS_ERR_OR_NULL(panel->dev)) {
		pr_err("failed to create panel device\n");
		ret = -EINVAL;
		goto exit2;
	}

	mutex_init(&panel->lock);
	dev_set_drvdata(panel->dev, panel);

	panel_no++;

	return ret;

exit2:
	class_destroy(panel_class);
exit1:
	kfree(panel);
exit0:
#endif
	return ret;
}

static int s6e3fa0_displayon(struct dsim_device *dsim)
{
#if defined(CONFIG_EXYNOS_PANEL_CABC)
	struct panel_device *panel = panel_drvdata;
#endif

	s6e3fa0_lcd_init(dsim->id, &dsim->lcd_info);
	s6e3fa0_lcd_enable(dsim->id);

#if defined(CONFIG_EXYNOS_PANEL_CABC)
	if (panel)
		s6e3fa0_cabc_mode(dsim, panel->cabc_mode);
#endif

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
	if (dsim->esd_recovering)
		s6e3fa0_set_brightness(dsim->bd);
#endif
	return 1;
}

static int s6e3fa0_suspend(struct dsim_device *dsim)
{
	s6e3fa0_lcd_disable(dsim->id);
	s6e3fa0_lcd_sleepin(dsim->id);

	return 1;
}

static int s6e3fa0_resume(struct dsim_device *dsim)
{
	return 1;
}

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
#if defined(READ_ESD_SOLUTION_TEST)
static int s6e3fa0_read_state(struct dsim_device *dsim)
{
	int ret = 0;
	u8 buf[2] = {0x0};
	int i = 0;
	int RETRY = 5; /* several retries are allowed */
	bool esd_detect = false;

	dsim_info("%s, read DDI for checking ESD\n", __func__);

	switch (dsim->esd_test) {
	case 0: /* same as original operation */
		for (i = 0; i < RETRY; i++) {
			ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
					MIPI_DCS_GET_POWER_MODE, 0x1, buf);
			if (ret < 0) {
				dsim_err("Failed to read panel REG 0x%02X!: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
				if (dsim->state != DSIM_STATE_ON)
					return DSIM_ESD_OK;
				usleep_range(1000, 1100);
				continue;
			}

			if ((buf[0] & 0x7c) == 0x1c) {
				dsim_info("s6e3fa0 panel REG 0x%02X=0x%02x\n",
					MIPI_DCS_GET_POWER_MODE, buf[0]);
				break;
			}

			dsim_err("s6e3fa0 panel REG 0x%02X Not match: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
			esd_detect = true;
		}

		if (i < RETRY)
			return DSIM_ESD_OK;
		else if (esd_detect)
			return DSIM_ESD_ERROR;
		else
			return DSIM_ESD_CHECK_ERROR;


	case 1: /* simple check and always return esd_detection */
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
					MIPI_DCS_GET_POWER_MODE, 0x1, buf);
		if (ret < 0) {
			dsim_err("Failed to read panel REG 0x%02X!: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
		} else {
			if ((buf[0] & 0x7c) == 0x1c)
				dsim_info("s6e3fa0 panel REG 0x%02X=0x%02x\n",
					MIPI_DCS_GET_POWER_MODE, buf[0]);
			else {
				dsim_err("s6e3fa0 panel REG 0x%02X Not match: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
			}
		}
		dsim->esd_test = 0;
		return DSIM_ESD_ERROR;
	case 2: /* always return esd detection and initialize  */
		dsim->esd_test = 0;
		return DSIM_ESD_ERROR;
	case 3: /* always return esd detection */
		dsim->esd_test = 3;
		return DSIM_ESD_ERROR;
	case 4: /* always return esd ok */
		dsim->esd_test = 4;
		return DSIM_ESD_OK;
	case 5: /* always return esd ok and display off/on by force*/
		dsim_info("%s, lcd_disable is called for ESD test\n", __func__);
		s6e3fa0_lcd_disable(dsim->id);
		dsim_info("%s, 2sec sleep for ESD test\n", __func__);
		msleep(2000);
		dsim_info("%s, lcd_enable is called for ESD test\n", __func__);
		s6e3fa0_lcd_enable(dsim->id);
		dsim->esd_test = 4;
		return DSIM_ESD_OK;
	case 6: /* always return esd detection and display off by force*/
		dsim_info("%s, lcd_disable is called for ESD test\n", __func__);
		s6e3fa0_lcd_disable(dsim->id);
		dsim_info("%s, 2sec sleep for ESD test\n", __func__);
		msleep(2000);
		dsim_info("%s, lcd_enable is called for ESD test\n", __func__);
		s6e3fa0_lcd_enable(dsim->id);
		dsim->esd_test = 4;
		return DSIM_ESD_ERROR;
	case 7: /* return DSIM_ESD_CHECK_ERROR by force */
		for (i = 0; i < RETRY; i++) {
			ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
					MIPI_DCS_GET_POWER_MODE, 0x1, buf);
			ret = -ETIMEDOUT;
			if (ret < 0) {
				dsim_err("Failed to read panel REG 0x%02X!: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
				if (dsim->state != DSIM_STATE_ON)
					return DSIM_ESD_OK;
				usleep_range(1000, 1100);
				continue;
			}
		}

		dsim->esd_test = 0;

		if (i < RETRY)
			return DSIM_ESD_OK;
		else if (esd_detect)
			return DSIM_ESD_ERROR;
		else
			return DSIM_ESD_CHECK_ERROR;
	default:
		break;
	}
	return DSIM_ESD_OK;
}

#else
static int s6e3fa0_read_state(struct dsim_device *dsim)
{
	int ret = 0;
	u8 buf[2] = {0x0};
	int i = 0;
	int RETRY = 5; /* several retries are allowed */
	bool esd_detect = false;

	dsim_info("%s, read DDI for checking ESD\n", __func__);

	for (i = 0; i < RETRY; i++) {
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
					MIPI_DCS_GET_POWER_MODE, 0x1, buf);
		if (ret < 0) {
			dsim_err("Failed to read panel REG 0x%02X!: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
			if (dsim->state != DSIM_STATE_ON)
				return DSIM_ESD_OK;
			usleep_range(1000, 1100);
			continue;
		}

		if ((buf[0] & 0x7c) == 0x1c) {
			dsim_info("s6e3fa0 panel REG 0x%02X=0x%02x\n",
					MIPI_DCS_GET_POWER_MODE, buf[0]);
			break;
		}

		dsim_err("s6e3fa0 panel REG 0x%02X Not match: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
		esd_detect = true;
	}

	if (i < RETRY)
		return DSIM_ESD_OK;
	else if (esd_detect)
		return DSIM_ESD_ERROR;
	else
		return DSIM_ESD_CHECK_ERROR;
}
#endif
#endif

struct dsim_lcd_driver s6e3fa0_mipi_lcd_driver = {
	.probe		= s6e3fa0_probe,
	.displayon	= s6e3fa0_displayon,
	.suspend	= s6e3fa0_suspend,
	.resume		= s6e3fa0_resume,
#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
	.read_state	= s6e3fa0_read_state,
#endif
};
