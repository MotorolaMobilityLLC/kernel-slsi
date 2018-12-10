/* drivers/video/fbdev/exynos/dpu20/panels/nov36672a_mipi_lcd.c
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2018 Samsung Electronics
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
#include "nov36672a_param.h"

#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0
#define DEFAULT_BRIGHTNESS 170

static struct dsim_device *dsim_base;
static struct backlight_device *bd;

struct panel_device {
	struct device *dev;
	struct dsim_device *dsim;
	struct mutex lock;
	int cabc_mode;
};

struct panel_device *nov36672a_panel_drvdata;
struct class *nov36672a_panel_class;

static int nov36672a_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int nov36672a_get_backlight_level(int brightness)
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

static int nov36672a_update_brightness(int brightness)
{
	int backlightlevel;

	backlightlevel = nov36672a_get_backlight_level(brightness);

	pr_info("brightness [%d]\n", brightness);
	if (dsim_wr_data(0, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_0[0],
				brightness) < 0)
		dsim_err("fail to send SEQ_CMD_0 command.\n");

	return 0;
}

static int nov36672a_set_brightness(struct backlight_device *bd)
{
	struct dsim_device *dsim;
	int brightness = bd->props.brightness;

	dsim = get_dsim_drvdata(0);

	if (!IS_DSIM_ON_STATE(dsim))
		return 0;

	if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
		pr_err("Brightness should be in the range of 0 ~ 255\n");
		return -EINVAL;
	}

	dsim->user_brightness = brightness;
	if ((brightness > dsim->max_brightness) &&
			(brightness <= MAX_BRIGHTNESS)) {
		brightness = dsim->max_brightness;
	}

	nov36672a_update_brightness(brightness);
	dsim->brightness = brightness;

	return 0;
}

static const struct backlight_ops nov36672a_backlight_ops = {
	.get_brightness = nov36672a_get_brightness,
	.update_status = nov36672a_set_brightness,
};

#if defined(CONFIG_EXYNOS_PANEL_CABC)
static int nov36672a_cabc_mode(struct dsim_device *dsim, int mode)
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
#endif

static ssize_t panel_cabc_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int ret = 0;
	struct panel_device *panel = dev_get_drvdata(dev);

	mutex_lock(&panel->lock);

#if defined(CONFIG_EXYNOS_PANEL_CABC)
	ret = nov36672a_cabc_mode(panel->dsim, CABC_READ_MODE);
#endif
	mutex_unlock(&panel->lock);

	count = snprintf(buf, PAGE_SIZE, "cabc_mode = %d, ret = %d\n",
			panel->cabc_mode, ret);

	return count;
}

static ssize_t panel_cabc_mode_store(struct device *dev,
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
#if defined(CONFIG_EXYNOS_PANEL_CABC)
	nov36672a_cabc_mode(panel->dsim, panel->cabc_mode);
#endif
	return count;
}

static ssize_t panel_max_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct dsim_device *dsim = get_dsim_drvdata(0);

	count = snprintf(buf, PAGE_SIZE, "max_brightness = %d\n",
			dsim->max_brightness);

	return count;
}

static ssize_t panel_max_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int value = 0;
	struct dsim_device *dsim = get_dsim_drvdata(0);
	int old_brightness;

	ret = kstrtouint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&dsim->bl_lock);

	old_brightness = dsim->brightness;

	if (value > MAX_BRIGHTNESS) {
		dsim->max_brightness = MAX_BRIGHTNESS;
		dsim->brightness = dsim->user_brightness;
	} else if ((value >= MIN_BRIGHTNESS) && (value <= MAX_BRIGHTNESS)) {
		dsim->max_brightness = value;
		if (dsim->user_brightness > dsim->max_brightness)
			dsim->brightness = dsim->max_brightness;
		else
			dsim->brightness = dsim->user_brightness;
	} else {
		goto end;
	}

	if (old_brightness != dsim->brightness) {
		nov36672a_update_brightness(dsim->brightness);
	}

end:
	mutex_unlock(&dsim->bl_lock);

	pr_info("%s: %d\n", __func__, dsim->max_brightness);

	return count;
}

static DEVICE_ATTR(max_brightness, 0660, panel_max_brightness_show,
			panel_max_brightness_store);


static ssize_t panel_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct dsim_device *dsim = get_dsim_drvdata(0);

	count = snprintf(buf, PAGE_SIZE, "brightness = %d\n",
			dsim->brightness);

	return count;
}

static ssize_t panel_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int value = 0;
	struct dsim_device *dsim = get_dsim_drvdata(0);

	ret = kstrtouint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&dsim->bl_lock);

	if (value <= dsim->max_brightness) {
		dsim->brightness = value;
		nov36672a_update_brightness(dsim->brightness);
	} else if (value <= MAX_BRIGHTNESS) {
		dsim->user_brightness = value;
	} else {
		pr_err("%s, brightness value is wrong[%d]\n",
				__func__, value);
	}

	mutex_unlock(&dsim->bl_lock);

	pr_info("%s: %d\n", __func__, dsim->brightness);

	return count;
}

static DEVICE_ATTR(brightness, 0660, panel_brightness_show,
			panel_brightness_store);

static DEVICE_ATTR(cabc_mode, 0660, panel_cabc_mode_show,
		panel_cabc_mode_store);

static ssize_t lcd_panel_supplier_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;

	count = snprintf(buf, PAGE_SIZE, "%s\n","nov36672a");
	return count;
}

static DEVICE_ATTR(panel_supplier, 0644, lcd_panel_supplier_show,NULL);

static struct attribute *panel_attrs[] = {
	&dev_attr_cabc_mode.attr,
	&dev_attr_panel_supplier.attr,
	&dev_attr_max_brightness.attr,
	&dev_attr_brightness.attr,
	NULL,
};
ATTRIBUTE_GROUPS(panel);

static struct attribute_group panel = {
	.attrs = panel_attrs,
};

static int nov36672a_create_sysfs(struct dsim_device *dsim)
{
	int rc;
	rc = sysfs_create_group(&dsim->dev->kobj, &panel);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);
	return rc;
}


static int nov36672a_probe(struct dsim_device *dsim)
{
	int ret = 1;
	struct panel_device *panel;
	static unsigned int panel_no;

	dsim_base = dsim;

	bd = backlight_device_register("backlight_0", NULL,
			NULL, &nov36672a_backlight_ops, NULL);
	if (IS_ERR(bd))
		pr_err("failed to register backlight device!\n");

	bd->props.max_brightness = MAX_BRIGHTNESS;
	bd->props.brightness = DEFAULT_BRIGHTNESS;
	dsim->max_brightness = MAX_BRIGHTNESS;
	dsim->brightness = DEFAULT_BRIGHTNESS;
	nov36672a_create_sysfs(dsim);
	panel = kzalloc(sizeof(struct panel_device), GFP_KERNEL);
	if (!panel) {
		pr_err("failed to allocate panel\n");
		ret = -ENOMEM;
		goto exit0;
	}

	nov36672a_panel_drvdata = panel;

	panel->dsim = dsim;
	panel->cabc_mode = 0;

	if (IS_ERR_OR_NULL(nov36672a_panel_class)) {
		nov36672a_panel_class = class_create(THIS_MODULE, "panel");
		if (IS_ERR_OR_NULL(nov36672a_panel_class)) {
			pr_err("failed to create panel class\n");
			ret = -EINVAL;
			goto exit1;
		}

		nov36672a_panel_class->dev_groups = panel_groups;
	}

	panel->dev = device_create(nov36672a_panel_class, dsim->dev, 0,
			&panel, !panel_no ? "panel" : "panel%d", panel_no);
	if (IS_ERR_OR_NULL(panel->dev)) {
		pr_err("failed to create panel device\n");
		ret = -EINVAL;
		goto exit2;
	}

	mutex_init(&panel->lock);
	mutex_init(&dsim->bl_lock);
	dev_set_drvdata(panel->dev, panel);

	panel_no++;

	return ret;

exit2:
	class_destroy(nov36672a_panel_class);
exit1:
	kfree(panel);
exit0:
	return ret;
}

static int nov36672a_displayon(struct dsim_device *dsim)
{
#if defined(CONFIG_EXYNOS_PANEL_CABC)
	struct panel_device *panel = nov36672a_panel_drvdata;
#endif
	dsim_info("%s +\n", __func__);
	nov36672a_lcd_init(dsim->id, &dsim->lcd_info);
	nov36672a_lcd_enable(dsim->id);
	dsim_info("%s -\n", __func__);
#if defined(CONFIG_EXYNOS_PANEL_CABC)
	if (panel)
		nov36672a_cabc_mode(dsim, panel->cabc_mode);
#endif
#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
	if (dsim->esd_recovering)
		nov36672a_update_brightness(dsim->brightness);
#endif
	return 1;
}

static int nov36672a_suspend(struct dsim_device *dsim)
{
	dsim_info("%s +\n", __func__);
	nov36672a_lcd_disable(dsim->id);
	dsim_info("%s -\n", __func__);
	return 1;
}

static int nov36672a_resume(struct dsim_device *dsim)
{
	return 0;
}

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
static int nov36672a_read_state(struct dsim_device *dsim)
{
	int ret = 0;
	u8 buf[2] = {0x0};
	int i = 0;
	int RETRY = 2; /* several retries are allowed */
	bool esd_detect = false;

	dsim_info("%s, read DDI for checking ESD\n", __func__);

	for (i = 0; i < RETRY; i++) {
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
				MIPI_DCS_GET_POWER_MODE, 0x1, buf);
		if (ret == -ETIMEDOUT) {
			dsim_info("recovery is already operated\n");
			return DSIM_ESD_OK;
		} else if ((ret < 0) && (ret != -ETIMEDOUT)) {
			dsim_err("Failed to read panel REG 0x%02X!: 0x%02x, i(%d)\n",
					MIPI_DCS_GET_POWER_MODE,
					*(unsigned int *)buf & 0xFF, i);
			if (dsim->state != DSIM_STATE_ON)
				return DSIM_ESD_OK;
			usleep_range(1000, 1100);
			continue;
		}
		if ((buf[0] & 0x7c) == 0x1c) {
			dsim_info("nov36672a panel REG 0x%02X=0x%02x\n",
					MIPI_DCS_GET_POWER_MODE, buf[0]);
			break;
		}

		dsim_err("nov36672a panel REG 0x%02X Not match: 0x%02x, i(%d)\n",
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

struct dsim_lcd_driver nov36672a_mipi_lcd_driver = {
	.probe		= nov36672a_probe,
	.displayon	= nov36672a_displayon,
	.suspend	= nov36672a_suspend,
	.resume		= nov36672a_resume,
#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
	.read_state	= nov36672a_read_state,
#endif
};
