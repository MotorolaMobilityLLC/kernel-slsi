/*
 * leds-s2mu106-rgb.c - LED class driver for S2MU106 RGB LEDs.
 *
 * Copyright (C) 2018 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/samsung/s2mu106.h>
#include <linux/leds-s2mu106-rgb.h>
#include <linux/platform_device.h>

static void s2mu106_rgb_test_read(struct s2mu106_rgb_data *rgb)
{
	u8 data;
	char str[1016] = {0,};
	int i;
	struct i2c_client *i2c = rgb->i2c;

    for (i = 0x43; i <= 0x51; i++) {
		s2mu106_read_reg(i2c, i, &data);
		sprintf(str+strlen(str), "0x%02x:0x%02x, ", i, data);
	}

	s2mu106_read_reg(i2c, 0x52, &data);
	pr_err("%s: %s0x52:0x%02x\n", __func__, str, data);
}

static int s2mu106_rgb_get_ramp_up(struct s2mu106_rgb_data *rgb, int led)
{
	u8 data;
	int ret = -1;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	s2mu106_read_reg(rgb->i2c,
		s2mu106_rgb_ramp_reg[led], &data);
	ret = (data & S2MU106_RGB_LEDX_RAMP_UP_MASK) >> 4;
	ret = (ret > 8) ? (ret - 8) * 200 + 800:
				ret * 100;

	pr_info("%s: data = 0x%02x, %dms \n", __func__,
		data, ret);

	return ret;
}

/* 1ms unit*/
static int s2mu106_rgb_set_ramp_up(struct s2mu106_rgb_data *rgb,
								int led, int time)
{
	u8 data = 0;
	int set_time = 0;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	if (time > S2MU106_RGB_RAMP_MAX)
		time = S2MU106_RGB_RAMP_MAX;
	else if (time < 0)
		time = 0;

	data = (time > 800)?(time - 800)/200 + 8:
		time / 100;

	s2mu106_update_reg(rgb->i2c, s2mu106_rgb_ramp_reg[led],
		data << 4, S2MU106_RGB_LEDX_RAMP_UP_MASK);

	set_time = s2mu106_rgb_get_ramp_up(rgb, led);
	pr_info("%s: time = %dms, set_time = %dms \n", __func__,
		time, set_time);

	return 0;
}

static int s2mu106_rgb_get_ramp_down(struct s2mu106_rgb_data *rgb, int led)
{
	u8 data;
	int ret = -1;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	s2mu106_read_reg(rgb->i2c,
		s2mu106_rgb_ramp_reg[led], &data);
	ret = data & S2MU106_RGB_LEDX_RAMP_DOWN_MASK;
	ret = (ret > 8) ? (ret - 8) * 200 + 800:
				ret * 100;

	pr_info("%s: data = 0x%02x, %dms \n", __func__,
		data, ret);

	return ret;

}

static int s2mu106_rgb_set_ramp_down(struct s2mu106_rgb_data *rgb,
								int led, int time)
{
	u8 data = 0;
	int set_time = 0;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	if (time > S2MU106_RGB_RAMP_MAX)
		time = S2MU106_RGB_RAMP_MAX;
	else if (time < 0)
		time = 0;

	data = (time > 800)?(time - 800)/200 + 8:
		time / 100;

	s2mu106_update_reg(rgb->i2c, s2mu106_rgb_ramp_reg[led],
		data, S2MU106_RGB_LEDX_RAMP_DOWN_MASK);

	set_time = s2mu106_rgb_get_ramp_down(rgb, led);
	pr_info("%s: time = %dms, set_time = %dms \n", __func__,
		time, set_time);

	return 0;
}

/* 1ms unit */
static int s2mu106_rgb_get_on_dur(struct s2mu106_rgb_data *rgb, int led)
{
	u8 data;
	int ret = -1;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	s2mu106_read_reg(rgb->i2c, s2mu106_rgb_dur_reg[led],
		&data);
	data = (data & S2MU106_RGB_LEDX_ON_DUR_MASK) >> 4;
	ret = (data < 5) ? data * 100 + 100 : (data - 4) * 250 + 500;

	pr_info("%s: data = 0x%02x, %dms \n", __func__,
		data, ret);

	return ret;
}

static int s2mu106_rgb_set_on_dur(struct s2mu106_rgb_data *rgb,
								int led, int time)
{
	u8 data;
	int set_time = 0;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	if (time > S2MU106_RGB_ON_MAX)
		time = S2MU106_RGB_ON_MAX;
	else if (time < 0)
		time = 0;

	data = (time > 500)?(time - 500) / 250 + 4:
		(time - 100) / 100;

	s2mu106_update_reg(rgb->i2c, s2mu106_rgb_dur_reg[led],
		data << 4, S2MU106_RGB_LEDX_ON_DUR_MASK);

	set_time = s2mu106_rgb_get_on_dur(rgb, led);
	pr_info("%s: time = %dms, set_time = %dms\n", __func__,
		time, set_time);

	return 0;
}

static int s2mu106_rgb_get_off_dur(struct s2mu106_rgb_data *rgb, int led)
{
	u8 data;
	u8 time_mode;
	int ret = -1;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	s2mu106_read_reg(rgb->i2c, s2mu106_rgb_dur_reg[led],
		&data);
	s2mu106_read_reg(rgb->i2c, S2MU106_RGB_LED_CTRL0,
		&time_mode);

	data = (data & S2MU106_RGB_LEDX_OFF_DUR_MASK);
	time_mode = !!(time_mode & s2mu106_rgb_off_dur_mode_mask[led]);
	ret = s2mu106_off_time[time_mode][data];

	pr_info("%s: time_mode = %d, data = 0x%02x, %dms \n", __func__,
		time_mode, data, ret);

	return ret;
}

static int s2mu106_rgb_set_off_dur(struct s2mu106_rgb_data *rgb,
								int led, int time)
{
	u8 data;
	int max = rgb->off_dur_mode?S2MU106_RGB_OFF_MAX1:S2MU106_RGB_OFF_MAX0;
	int diff = max;
	int set_time = 0;
	int i;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	if (time > max)
		time = max;
	else if (time < 0)
		time = 0;

	for (i = 0; i < S2MU106_RGB_ONOFF_TIME_NUM; i++) {
		if (ABS(time - s2mu106_off_time[rgb->off_dur_mode][i]) < diff) {
			diff = ABS(time - s2mu106_off_time[rgb->off_dur_mode][i]);
			data = i;
		}
	}

	pr_info("%s: mode = 0x%02x, data = 0x%02x\n", __func__,
		rgb->off_dur_mode, data);

	s2mu106_update_reg(rgb->i2c, s2mu106_rgb_dur_reg[led],
		data, S2MU106_RGB_LEDX_OFF_DUR_MASK);

	set_time = s2mu106_rgb_get_off_dur(rgb, led);
	pr_info("%s: time = %dms, set_time = %dms\n", __func__,
		time, set_time);

	return 0;
}

/* 0.1mA unit*/
static int s2mu106_rgb_get_curr(struct s2mu106_rgb_data *rgb, int led)
{
	u8 data;
	u8 source = 0;
	int ret = -1;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	source = s2mu106_rgb_curr_reg[led];

	s2mu106_read_reg(rgb->i2c, source, &data);

	pr_info("%s: data = 0x%02x, curr = %d.%d\n", __func__, data,
		data/10, data%10);

	ret = data;

	return ret;
}

/* 0.1mA unit*/
static int s2mu106_rgb_set_curr(struct s2mu106_rgb_data *rgb,
								int led, int curr)
{
	u8 data = 0;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	if (curr < 0)
		data = 0;
	else if (curr > S2MU106_RGB_CURR_MAX)
		data = S2MU106_RGB_CURR_MAX;
	else
		data = curr;

	pr_err("%s: led: %d, curr: %d.%dmA\n", __func__, led,
		data/10, data%10);

	s2mu106_write_reg(rgb->i2c,
		s2mu106_rgb_curr_reg[led], data);

	return 0;
}

static int s2mu106_rgb_set_mode(struct s2mu106_rgb_data *rgb,
								int led, int mode)
{
	if ((led < 1) || (led > S2MU106_RGB_LED_MAX) ||
		(mode < 0) || (mode > S2MU106_RGB_MODE_MAX)) {
			pr_err("%s: Wrong led or mode.\n", __func__);
			return -EFAULT;
	}

	pr_err("%s: led: %d, mode: %d\n", __func__, led, mode);

	s2mu106_update_reg(rgb->i2c, S2MU106_RGB_LED_EN,
		mode << s2mu106_rgb_led_en_shift[led],
		s2mu106_rgb_led_en_mask[led]);

	return 0;
}

static int s2mu106_rgb_get_mode(struct s2mu106_rgb_data *rgb, int led)
{
	u8 led_en;
	int ret = -1;

	if ((led < 1) || (led > S2MU106_RGB_LED_MAX)) {
		pr_info("%s: Wrong led!!\n", __func__);
		return -1;
	}

	s2mu106_read_reg(rgb->i2c, S2MU106_RGB_LED_EN, &led_en);

	pr_info("%s: S2MU106_RGB_LED_EN: 0x%02x\n", __func__, led_en);

	ret = (led_en & s2mu106_rgb_led_en_mask[led])
					>> s2mu106_rgb_led_en_shift[led];

	return ret;
}

static void s2mu106_rgb_set_off_dur_mode(struct s2mu106_rgb_data *rgb,
	int mode)
{
	int i, cnt = 0;
    char str[200] = {0,};

	s2mu106_update_reg(rgb->i2c, S2MU106_RGB_LED_CTRL0,
		mode ? S2MU106_RGB_OFF_DUR_MODE_MASK:0,
		S2MU106_RGB_OFF_DUR_MODE_MASK);
	for (i = 0; i < S2MU106_RGB_ONOFF_TIME_NUM; i++) {
		cnt += sprintf(str+strlen(str), "%dms, ",
			s2mu106_off_time[mode][i]);
	}

	pr_err("%s: mode: %d, time: %s\n", __func__, mode, str);
}

/* Control RGB LED1
 * brightness: 0 -> off
 * brightness: 1 ~ 255 -> on(constant mode),
 *                        brightness * 0.1mA current
 */
static void s2mu106_rgb_set_brightness(struct led_classdev *led_cdev,
		enum led_brightness brightness)
{
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);

	if (brightness < 0) {
		pr_err("%s: Wrong brightness.\n", __func__);
		return;
	}

	if (brightness == LED_OFF)
		s2mu106_rgb_set_mode(rgb,
				S2MU106_RGB_BRIGHTNESS_LED, S2MU106_RGB_MODE_OFF);
	else {
		s2mu106_rgb_set_curr(rgb,
				S2MU106_RGB_BRIGHTNESS_LED, brightness);
		s2mu106_rgb_set_mode(rgb,
				S2MU106_RGB_BRIGHTNESS_LED, S2MU106_RGB_MODE_CONST);
	}

	pr_info("%s: brightness = %d\n", __func__, brightness);

	s2mu106_rgb_test_read(rgb);
}

static enum led_brightness s2mu106_rgb_get_brightness(
		struct led_classdev *led_cdev)
{
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int mode = -1;
	int brightness = -1;

	mode = s2mu106_rgb_get_mode(rgb, S2MU106_RGB_BRIGHTNESS_LED);

	if (mode == S2MU106_RGB_MODE_OFF)
		brightness = LED_OFF;
	else
		brightness = s2mu106_rgb_get_curr(rgb,
				S2MU106_RGB_BRIGHTNESS_LED);

	pr_info("%s: mode = %s, brightness = %d\n", __func__,
			s2mu106_rgb_mode_string[mode], brightness);

	return brightness;
}

static ssize_t rgb_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int cnt = 0;
	int mode = 0;
	int i;
    char str[1016] = {0,};

	s2mu106_rgb_test_read(rgb);

	for (i = 1; i <= S2MU106_RGB_LED_MAX; i++) {
		mode = s2mu106_rgb_get_mode(rgb, i);
		if (mode >= 0)
			cnt += sprintf(str+strlen(str), "LED%02d: %s, ", i,
				s2mu106_rgb_mode_string[mode]);
	}

	cnt += sprintf(str+strlen(str), "\n");

	strcpy(buf, str);

    return cnt;
}

static ssize_t rgb_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int led = -1;
	int mode = -1;

	sscanf(buf, "%d %d", &led, &mode);

	if ((led <= 0) || (led > S2MU106_RGB_LED_MAX) ||
		(mode < 0) || (mode > S2MU106_RGB_MODE_MAX)) {
			pr_err("%s: led: %d, mode: %d\n", __func__, led, mode);
			pr_err("%s: Wrong led or mode.\n", __func__);
			return -EFAULT;
	}

	s2mu106_rgb_set_mode(rgb, led, mode);
	s2mu106_rgb_test_read(rgb);

	return size;
}

static ssize_t rgb_curr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int cnt = 0;
	int curr = 0;
	int i;
    char str[1016] = {0,};

	for (i = 1; i <= S2MU106_RGB_LED_MAX; i++) {
		curr = s2mu106_rgb_get_curr(rgb, i);
		if (curr >= 0)
			cnt += sprintf(str+strlen(str), "LED%02d: %d.%dmA, ",
				i, curr/10, curr%10);
	}

	cnt += sprintf(str+strlen(str), "\n");

	strcpy(buf, str);

    return cnt;
}

static ssize_t rgb_curr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int led = -1;
	int curr = -1;

	sscanf(buf, "%d %d", &led, &curr);

	if ((led <= 0) || (led > S2MU106_RGB_LED_MAX)) {
			pr_err("%s: Wrong channel.\n", __func__);
			return -EFAULT;
	}

	s2mu106_rgb_set_curr(rgb, led, curr);

	s2mu106_rgb_test_read(rgb);

	return size;
}

static ssize_t rgb_ramp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int ramp_up, ramp_down;
	int i, cnt = 0;
    char str[1016] = {0,};

	for (i = 1; i <= S2MU106_RGB_LED_MAX; i++) {
		ramp_up = s2mu106_rgb_get_ramp_up(rgb, i);
		ramp_down = s2mu106_rgb_get_ramp_down(rgb, i);
		cnt += sprintf(str+strlen(str),
			"LED%d: ramp_up = %dms, ramp_down = %dms\n", i, ramp_up, ramp_down);
	}

	strcpy(buf, str);

    return cnt;
}

static ssize_t rgb_ramp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int led = -1;
	int ramp_up, ramp_down;

	sscanf(buf, "%d %d %d", &led, &ramp_up, &ramp_down);

	if ((led <= 0) || (led > S2MU106_RGB_LED_MAX)) {
			pr_err("%s: Wrong channel.\n", __func__);
			return -EFAULT;
	}

	s2mu106_rgb_set_ramp_up(rgb, led, ramp_up);

	s2mu106_rgb_set_ramp_down(rgb, led, ramp_down);

	s2mu106_rgb_test_read(rgb);

	return size;
}

static ssize_t rgb_onoff_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int on_dur, off_dur;
	int i, cnt = 0;
    char str[1016] = {0,};

	for (i = 1; i <= S2MU106_RGB_LED_MAX; i++) {
		on_dur = s2mu106_rgb_get_on_dur(rgb, i);
		off_dur = s2mu106_rgb_get_off_dur(rgb, i);
		cnt += sprintf(str+strlen(str),
			"LED%d: on_dur = %dms, off_dur = %dms\n", i, on_dur, off_dur);
	}

	strcpy(buf, str);

    return cnt;
}

static ssize_t rgb_onoff_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct s2mu106_rgb_data *rgb =
		container_of(led_cdev, struct s2mu106_rgb_data, cdev);
	int led = -1;
	int on_dur, off_dur;

	sscanf(buf, "%d %d %d", &led, &on_dur, &off_dur);

	if ((led <= 0) || (led > S2MU106_RGB_LED_MAX)) {
			pr_err("%s: Wrong channel.\n", __func__);
			return -EFAULT;
	}

	s2mu106_rgb_set_on_dur(rgb, led, on_dur);

	s2mu106_rgb_set_off_dur(rgb, led, off_dur);

	s2mu106_rgb_test_read(rgb);

	return size;
}

static DEVICE_ATTR(rgb_mode, 0644, rgb_mode_show, rgb_mode_store);
static DEVICE_ATTR(rgb_curr, 0644, rgb_curr_show, rgb_curr_store);
static DEVICE_ATTR(rgb_ramp, 0644, rgb_ramp_show, rgb_ramp_store);
static DEVICE_ATTR(rgb_onoff, 0644, rgb_onoff_show, rgb_onoff_store);

static struct attribute *s2mu106_rgb_attrs[] = {
	&dev_attr_rgb_mode.attr,
	&dev_attr_rgb_curr.attr,
	&dev_attr_rgb_ramp.attr,
	&dev_attr_rgb_onoff.attr,
    NULL
};
ATTRIBUTE_GROUPS(s2mu106_rgb);

static void s2mu106_rgb_init(struct s2mu106_rgb_data *rgb)
{
	pr_info("%s: s2mu106_rgb init start\n", __func__);

	rgb->off_dur_mode = rgb->pdata->def_off_dur_mode;
	s2mu106_rgb_set_off_dur_mode(rgb, rgb->off_dur_mode);

	/* Set LED1 for test */
	s2mu106_rgb_set_on_dur(rgb, 1, 1000);
	s2mu106_rgb_set_off_dur(rgb, 1, 500);
	s2mu106_rgb_set_ramp_up(rgb, 1, 500);
	s2mu106_rgb_set_ramp_down(rgb, 1, 1000);

	s2mu106_rgb_test_read(rgb);
}

#if defined(CONFIG_OF)
static int s2mu106_led_dt_parse_pdata(struct device *dev,
				struct s2mu106_rgb_platform_data *pdata)
{
	struct device_node *led_np, *np, *c_np;
	int ret;
	u32 temp;
	u32 index;

	led_np = dev->parent->of_node;

	if (!led_np) {
		pr_err("<%s> could not find led sub-node led_np\n", __func__);
		return -ENODEV;
	}

	np = of_find_node_by_name(led_np, "rgb_led");
	if (!np) {
		pr_err("%s : could not find led sub-node np\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "default_current",
			&pdata->default_current);
	if (ret < 0)
		pr_err("%s : could not find default_current\n", __func__);

	ret = of_property_read_u32(np, "max_current",
			&pdata->max_current);
	if (ret < 0)
		pr_err("%s : could not find max_current\n", __func__);

	ret = of_property_read_u32(np, "default_off_dur_mode",
			&pdata->def_off_dur_mode);
	if (ret < 0) {
		pr_err("%s : could not find def_onoff_mode\n", __func__);
		pdata->def_off_dur_mode = 0;
	}

	pdata->led_num = of_get_child_count(np);

	if (pdata->led_num > S2MU106_RGB_LED_MAX)
		pdata->led_num = S2MU106_RGB_LED_MAX;

	pdata->led = devm_kzalloc(dev,
		sizeof(struct s2mu106_rgb_led) * pdata->led_num, GFP_KERNEL);

	for_each_child_of_node(np, c_np) {
		ret = of_property_read_u32(c_np, "id", &temp);
		if (ret < 0)
			goto dt_err;
		index = temp;

		pr_info("%s: temp = %d, index = %d\n",
			__func__, temp, index);

		if (index < S2MU106_RGB_LED_MAX) {
			pdata->led[index].id = index;

			ret = of_property_read_u32_index(np, "current", index,
					&pdata->led[index].curr);
			if (ret < 0) {
				pr_err("%s : could not find current for led%d\n",
					__func__, pdata->led[index].id);
				pdata->led[index].curr = pdata->default_current;
			}
		}
	}
	return 0;
dt_err:
	pr_err("%s: DT parsing finish. ret = %d\n", __func__, ret);
	return ret;
}
#endif /* CONFIG_OF */

static int s2mu106_rgb_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct s2mu106_dev *s2mu106 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu106_rgb_data *rgb_data;
	char name[20];

	pr_info("%s: s2mu106_rgb start\n", __func__);

	if (!s2mu106) {
		dev_err(&pdev->dev, "drvdata->dev.parent not supplied\n");
		return -ENODEV;
	}

	rgb_data = devm_kzalloc(&pdev->dev,
		sizeof(struct s2mu106_rgb_data), GFP_KERNEL);
	if (!rgb_data) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		return -ENOMEM;
	}

	rgb_data->dev = &pdev->dev;
	rgb_data->i2c = s2mu106->i2c;
	rgb_data->pdata = devm_kzalloc(&pdev->dev,
		sizeof(*(rgb_data->pdata)), GFP_KERNEL);
	if (!rgb_data->pdata) {
		pr_err("%s: failed to allocate platform data\n", __func__);
		return -ENOMEM;
	}

	if (s2mu106->dev->of_node) {
		ret = s2mu106_led_dt_parse_pdata(&pdev->dev, rgb_data->pdata);
		if (ret < 0) {
			pr_err("%s: not found leds dt! ret=%d\n",
				__func__, ret);
			return -1;
		}
	}

	platform_set_drvdata(pdev, rgb_data);

	s2mu106_rgb_init(rgb_data);

    snprintf(name, sizeof(name), "charging");
    rgb_data->cdev.name = name;
	rgb_data->cdev.groups = s2mu106_rgb_groups;
	rgb_data->cdev.brightness_get = s2mu106_rgb_get_brightness;
	rgb_data->cdev.brightness_set = s2mu106_rgb_set_brightness;

	ret = devm_led_classdev_register(&pdev->dev, &rgb_data->cdev);
	if (ret < 0) {
		pr_err("%s: unable to register LED class dev\n", __func__);
		return ret;
	}

	return 0;
}

static int s2mu106_rgb_led_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver s2mu106_rgb_led_driver = {
	.driver = {
		.name  = "leds-s2mu106-rgb",
		.owner = THIS_MODULE,
		},
	.probe  = s2mu106_rgb_led_probe,
	.remove = s2mu106_rgb_led_remove,
};

static int __init s2mu106_rgb_led_driver_init(void)
{
	return platform_driver_register(&s2mu106_rgb_led_driver);
}
module_init(s2mu106_rgb_led_driver_init);

static void __exit s2mu106_rgb_led_driver_exit(void)
{
	platform_driver_unregister(&s2mu106_rgb_led_driver);
}
module_exit(s2mu106_rgb_led_driver_exit);

MODULE_AUTHOR("Keunho Hwang <keunho.hwang@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG s2mu106 RGB LED Driver");
MODULE_LICENSE("GPL");
