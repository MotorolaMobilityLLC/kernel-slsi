/*
 * haptic driver for s2mu106 - s2mu106_haptic.c
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/samsung/s2mu106.h>
#include <linux/input/s2mu106_haptic.h>
#include <linux/sysfs.h>

#define TEST_MODE_TIME 10000
#define MAX_INTENSITY 100
struct s2mu106_haptic_data *g_haptic;

static void s2mu106_set_boost_voltage(struct s2mu106_haptic_data *haptic, int voltage)
{
	u8 data;
	if (voltage <= 3150)
		data = 0x00;
	else if (voltage > 3150 && voltage <= 6300)
		data = (voltage - 3150) / 50;
	else
		data = 0xFF;
	pr_info("%s: boost voltage %d, 0x%02x\n", __func__, voltage, data);

	s2mu106_update_reg(haptic->i2c, S2MU106_REG_HBST_CTRL1,
				data, HAPTIC_BOOST_VOLTAGE_MASK);
}

static  void s2mu106_set_intensity(struct s2mu106_haptic_data *haptic, int intensity)
{
	int data = 0x3FFFF;
	int max = 0x7FFFF;
	u8 val1, val2, val3;

	if (intensity == MAX_INTENSITY)
		data = max;
	else if (intensity != 0) {
		long long tmp;

		tmp = (intensity * max) / 100;
		data = (int)tmp;
	} else
		data = 0;

	data &= 0x7FFFF;
	val1 = data & 0x0000F;
	val2 = (data & 0x00FF0) >> 4;
	val3 = (data & 0x7F000) >> 12;

	s2mu106_update_reg(haptic->i2c, S2MU106_REG_AMPCOEF1, val3, 0x7F);
	s2mu106_write_reg(haptic->i2c, S2MU106_REG_AMPCOEF2, val2);
	s2mu106_update_reg(haptic->i2c, S2MU106_REG_AMPCOEF3, val1 << 4, 0xF0);

	pr_info("%s, intensity = %d, coef1 = 0x%2x, coef2 = 0x%2x, coef3 = 0x%2x\n",
				__func__, intensity, val1, val2, val3);
}

static void haptic_work(struct work_struct *work)
{
	struct s2mu106_haptic_data *haptic
		= container_of(work, struct s2mu106_haptic_data, work);

	pr_info("%s\n", __func__);
	if (haptic->intensity > 0) {
		if (haptic->running)
			return;
		haptic->running = true;
		pr_info("Motor Enable\n");
		if (haptic->hap_mode == S2MU106_HAPTIC_ERM_I2C)
			s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, ERM_MODE_ON);
		else
			pwm_enable(haptic->pwm);
	} else {
		if (!haptic->running)
			return;
		haptic->running = false;
		pr_info("Motor Disable\n");
		if (haptic->hap_mode == S2MU106_HAPTIC_ERM_I2C)
			s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, ERM_MODE_OFF);
		else
			pwm_disable(haptic->pwm);
	}
}

static int s2mu106_haptic_play(struct input_dev *dev, void *data,
				       struct ff_effect *effect)
{
	struct s2mu106_haptic_data *haptic = input_get_drvdata(dev);

	if (effect->u.rumble.strong_magnitude) {
		haptic->intensity = effect->u.rumble.strong_magnitude;
		s2mu106_set_intensity(haptic, haptic->intensity);
	} else {
		haptic->intensity = 0;
	}

	schedule_work(&haptic->work);
	return 0;
}

/*
static void s2mu106_haptic_open(struct input_dev *input)
{
//	struct s2mu106_haptic *haptic = input_get_drvdata(dev);
}

static void s2mu106_haptic_close(struct input_dev *input)
{
//	struct s2mu106_haptic *haptic = input_get_drvdata(dev);

}
*/
static ssize_t intensity_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	int intensity = 0, ret = 0;

	ret = kstrtoint(buf, 0, &intensity);

	if ((intensity < 0) || (intensity > MAX_INTENSITY)) {
		dev_err(&g_haptic->i2c->dev, "Setting out of range intensity value\n");
		return -EINVAL;
	}

	g_haptic->intensity = intensity;
	s2mu106_set_intensity(g_haptic, g_haptic->intensity);

	pr_debug("%s, intensity = %d\n", __func__, intensity);

	return count;
}

static ssize_t intensity_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "intensity: %u\n", g_haptic->intensity);
}

static DEVICE_ATTR(intensity, 0660, intensity_show, intensity_store);

static ssize_t vib_enable_store(struct device *dev,
			struct device_attribute *devattr, const char *buf, size_t count)
{
	int enable = 0;
	int ret;

	ret = kstrtoint(buf, 0, &enable);

	if (enable == 1)
		schedule_work(&g_haptic->work);
	else if (enable == 0) {
		g_haptic->intensity = 0;
		s2mu106_set_intensity(g_haptic, g_haptic->intensity);
		schedule_work(&g_haptic->work);
	} else {
		pr_err("Invalid value!\n 0 : disable 1: enable\n");
		return -EINVAL;
	}

	pr_debug("%s, VIB %s\n", __func__,
			((enable == 1) ? "ENABLE" : "DISABLE"));

	return count;
}

static ssize_t vib_enable_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo 1 > vib_enable\necho 0 > vib_enable\n");
}

static DEVICE_ATTR(vib_enable, 0660, vib_enable_show, vib_enable_store);

#if defined(CONFIG_OF)
static int s2mu106_haptic_parse_dt(struct device *dev,
			struct s2mu106_haptic_platform_data *pdata)
{
	struct device_node *np = of_find_node_by_name(NULL, "s2mu106-haptic");
	u32 temp;
	int ret;

	pr_info("%s : start dt parsing\n", __func__);

	if (np == NULL) {
		pr_err("%s : error to get dt node\n", __func__);
		goto err_parsing_dt;
	}

	/* 30.08kHz 99% duty */
	ret = of_property_read_u32(np, "haptic,duty", &temp);
	if (ret < 0)
		pdata->duty = 32911;
	else
		pdata->duty = temp;

	ret = of_property_read_u32(np, "haptic,period", &temp);
	if (ret < 0)
		pdata->period = 33244;
	else
		pdata->period = temp;

	ret = of_property_read_u32(np, "haptic,pwm_id", &temp);
	if (ret < 0) {
		pr_err("%s : error to get dt node pwm_id\n", __func__);
		goto err_parsing_dt;
	} else
		pdata->pwm_id = (u16)temp;

	/* Haptic operation mode
		0 : S2MU106_HAPTIC_ERM_I2C
		1 : S2MU106_HAPTIC_ERM_GPIO
		2 : S2MU106_HAPTIC_LRA
		default : S2MU106_HAPTIC_LRA */
	pdata->hap_mode = 2;
	ret = of_property_read_u32(np, "haptic,operation_mode", &temp);
	if (ret < 0) {
		pr_err("%s : eror to get operation mode\n", __func__);
		goto err_parsing_dt;
	} else
		pdata->hap_mode = temp;

	/* Haptic boost setting */
	pdata->hbst.en = (of_find_property(np, "haptic,hbst_en", NULL)) ? true : false;

	pdata->hbst.automode =
		(of_find_property(np, "haptic,hbst_automode", NULL)) ? true : false;

	ret = of_property_read_u32(np, "haptic,boost_level", &temp);
	if (ret < 0)
		pdata->hbst.level = 5500;
	else
		pdata->hbst.level = temp;

	/* parsing info */
	pr_info("%s :operation_mode = %d,  period = %d, duty = %d, HBST_EN %s, HBST_AUTO_MODE %s\n", __func__,
			pdata->hap_mode,
			pdata->period, pdata->duty,
			pdata->hbst.en ? "enabled" : "disabled",
			pdata->hbst.automode ? "enabled" : "disabled");
	pdata->init_hw = NULL;
	pdata->motor_en = NULL;

	return 0;

err_parsing_dt:
	return -1;
}
#endif

static struct of_device_id s2mu106_haptic_match_table[] = {
	{ .compatible = "samsung,s2mu106-haptic",},
	{},
};

static void s2mu106_haptic_initial(struct s2mu106_haptic_data *haptic)
{
	u8 data;
	int ret;

	haptic->hap_mode = haptic->pdata->hap_mode;

	/* Haptic Boost initial setting */
	if (haptic->pdata->hbst.en){
		pr_info("%s : Haptic Boost Enable - Auto mode(%s)\n", __func__,
				haptic->pdata->hbst.automode ? "enabled" : "disabled");
		/* Boost voltage level setting
			default : 5.5V */
		s2mu106_set_boost_voltage(haptic, haptic->pdata->hbst.level);

		if (haptic->pdata->hbst.automode) {
			s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP0,
						HBST_OK_MASK_EN, HBST_OK_MASK_EN);
			s2mu106_update_reg(haptic->i2c,	S2MU106_REG_HBST_CTRL0,
						0, SEL_HBST_HAPTIC_MASK);
		} else {
			s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP0,
						0, HBST_OK_MASK_EN);
			s2mu106_update_reg(haptic->i2c,	S2MU106_REG_HBST_CTRL0,
						SEL_HBST_HAPTIC_MASK, SEL_HBST_HAPTIC_MASK);
		}
	} else {
		pr_info("%s : HDVIN - Vsys HDVIN voltage : Min 3.5V\n", __func__);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP2,
							0x00, VCEN_SEL_MASK);
	}

	/* Intensity setting to 0% */
	s2mu106_set_intensity(haptic, haptic->intensity);
	haptic->running = false;

	/* mode setting */
	switch (haptic->hap_mode) {
	case S2MU106_HAPTIC_LRA:
		data = LRA_MODE_EN;
		pwm_config(haptic->pwm, haptic->pdata->duty,
				haptic->pdata->period);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_OV_BK_OPTION,
					LRA_MODE_SET_MASK, LRA_MODE_SET_MASK);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_FILTERCOEF1, 0x7F);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_FILTERCOEF2, 0x5A);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_FILTERCOEF3, 0x02);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_OV_WAVE_NUM, 0xF0, 0xF0);
		break;
	case S2MU106_HAPTIC_ERM_GPIO:
		data = ERM_HDPWM_MODE_EN;
		haptic->pdata->duty = 100;
		haptic->pdata->period = 100;
		pwm_config(haptic->pwm, haptic->pdata->duty,
				haptic->pdata->period);
		break;
	case S2MU106_HAPTIC_ERM_I2C:
		data = ERM_MODE_OFF;
		break;
	default:
		data = LRA_MODE_EN;
		break;
	}
	s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, data);

	if (haptic->hap_mode == S2MU106_HAPTIC_ERM_I2C ||
		haptic->hap_mode == S2MU106_HAPTIC_ERM_GPIO) {
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_PERI_TAR1, 0x00);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_PERI_TAR2, 0x00);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_DUTY_TAR1, 0x00);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_DUTY_TAR2, 0x01);
	}
	pr_info("%s, haptic operation mode = %d\n", __func__, haptic->hap_mode);

	/* ON/OFF sysfs */
	ret = device_create_file(haptic->dev, &dev_attr_intensity);
	if (ret)
		pr_err("%s : failed to create sysfs\n", __func__);

	ret = device_create_file(haptic->dev, &dev_attr_vib_enable);
	if (ret)
		pr_err("%s : failed to create sysfs\n", __func__);

}

static int s2mu106_haptic_probe(struct platform_device *pdev)
{
	struct s2mu106_dev *s2mu106 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu106_haptic_data *haptic;

	int ret = 0;
	int error = 0;

	pr_info("%s Start\n", __func__);

	haptic = devm_kzalloc(&pdev->dev,
			sizeof(struct s2mu106_haptic_data), GFP_KERNEL);
	if (!haptic) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	haptic->dev = &pdev->dev;
	haptic->i2c = s2mu106->haptic;

	haptic->pdata = devm_kzalloc(&pdev->dev, sizeof(*(haptic->pdata)), GFP_KERNEL);
	if (!haptic->pdata) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_work_queue;
	}

	ret = s2mu106_haptic_parse_dt(&pdev->dev, haptic->pdata);
	if (ret < 0)
		goto err_work_queue;

	haptic->workqueue = create_singlethread_workqueue("hap_work");
	if (haptic->workqueue == NULL) {
		error = -EFAULT;
		pr_err("%s Failed to create workqueue\n", __func__);
		goto err_work_queue;
	}

	INIT_WORK(&(haptic->work), haptic_work);
	spin_lock_init(&(haptic->lock));
	mutex_init(&haptic->mutex);

	haptic->pwm = pwm_request(haptic->pdata->pwm_id, "vibrator");

	if (haptic->pwm < 0) {
		error = -EFAULT;
		pr_err("%s : Failed to request pwm\n", __func__);
		goto err_pwm_request;
	}

	pr_info("%s : request pwm, err num: %d Success\n", __func__, error);

	/* Default strength of vibration 0% */
	haptic->intensity = 0;

	haptic->input_dev = devm_input_allocate_device(&pdev->dev);
	if(!haptic->input_dev) {
		pr_err("%s : Failed to allocate device\n", __func__);
		goto err_create_device;
	}
	input_set_drvdata(haptic->input_dev, haptic);

	haptic->input_dev->name = "s2mu106-haptic";
	haptic->input_dev->dev.parent = &pdev->dev;
//	haptic->input_dev->open = s2mu106_haptic_open;
//	haptic->input_dev->close = s2mu106_haptic_close;

	input_set_capability(haptic->input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(haptic->input_dev, NULL,
						s2mu106_haptic_play);

	ret = input_register_device(haptic->input_dev);
	if (ret) {
		pr_err("Failed to register input device\n");
		goto err_create_device;
	}

	platform_set_drvdata(pdev, haptic);

	s2mu106_haptic_initial(haptic);

	g_haptic = haptic;
	pr_info("%s Loaded\n", __func__);
	return ret;

err_create_device:
	pwm_free(haptic->pwm);
err_pwm_request:
	destroy_workqueue(haptic->workqueue);
err_work_queue:
	kfree(haptic);
	return error;
}

static int s2mu106_haptic_remove(struct platform_device *pdev)
{
	struct s2mu106_haptic_data *haptic = platform_get_drvdata(pdev);

	pwm_free(haptic->pwm);
	destroy_workqueue(haptic->workqueue);
	device_remove_file(haptic->dev, &dev_attr_vib_enable);
	device_remove_file(haptic->dev, &dev_attr_intensity);

	kfree(haptic);
	return 0;
}

static int s2mu106_haptic_suspend(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct s2mu106_haptic_data *haptic = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);
	cancel_work_sync(&haptic->work);
	return 0;
}
static int s2mu106_haptic_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(s2mu106_haptic_pm_ops, s2mu106_haptic_suspend, s2mu106_haptic_resume);

static struct platform_driver s2mu106_haptic_driver = {
	.driver = {
		.name	= "s2mu106-haptic",
		.owner	= THIS_MODULE,
		.pm	= &s2mu106_haptic_pm_ops,
		.of_match_table = s2mu106_haptic_match_table,
	},
	.probe		= s2mu106_haptic_probe,
	.remove		= s2mu106_haptic_remove,
};

static int __init s2mu106_haptic_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&s2mu106_haptic_driver);
}
module_init(s2mu106_haptic_init);

static void __exit s2mu106_haptic_exit(void)
{
	platform_driver_unregister(&s2mu106_haptic_driver);
}
module_exit(s2mu106_haptic_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Haptic driver for S2MU106");
