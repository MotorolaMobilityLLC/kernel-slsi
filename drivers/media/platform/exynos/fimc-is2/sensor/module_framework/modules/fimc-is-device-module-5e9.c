/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <exynos-fimc-is-sensor.h>
#include "fimc-is-hw.h"
#include "fimc-is-core.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-resourcemgr.h"
#include "fimc-is-dt.h"

#include "fimc-is-device-module-base.h"

#define MAX_5E9_SETPIN_CNT	2

static struct fimc_is_sensor_cfg config_module_5e9[] = {
	/* 2592x1944@30fps */
	FIMC_IS_SENSOR_CFG(2592, 1944,  30, 0, 0, CSI_DATA_LANES_2, 871, CSI_MODE_VC_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2592, 1944), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
};

static const struct v4l2_subdev_core_ops core_ops = {
	.init = sensor_module_init,
	.g_ctrl = sensor_module_g_ctrl,
	.s_ctrl = sensor_module_s_ctrl,
	.g_ext_ctrls = sensor_module_g_ext_ctrls,
	.s_ext_ctrls = sensor_module_s_ext_ctrls,
	.ioctl = sensor_module_ioctl,
	.log_status = sensor_module_log_status,
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_routing = sensor_module_s_routing,
	.s_stream = sensor_module_s_stream,
	.s_parm = sensor_module_s_param
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt = sensor_module_s_format
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad = &pad_ops
};

static int sensor_module_5e9_power_setpin_0(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_none = 0;
	int gpio_iovdd_en = 0;
	int gpio_mclk = 0;
	struct fimc_is_core *core;

	FIMC_BUG(!dev);

	dnode = dev->of_node;

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	dev_info(dev, "%s E v4\n", __func__);

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get gpio_reset\n");
	} else {
		gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_reset);
	}

	gpio_iovdd_en = of_get_named_gpio(dnode, "gpio_iovdd_en", 0);
	if (gpio_is_valid(gpio_iovdd_en)) {
		if (gpio_request_one(gpio_iovdd_en, GPIOF_OUT_INIT_LOW, "CAM_IOVDD_EN_LOW")) {
			dev_err(dev, "%s: failed to gpio request iovdd_en\n", __func__);
			return -ENODEV;
		}
		gpio_free(gpio_iovdd_en);
	} else {
		dev_err(dev, "%s: failed to get iovdd_en\n", __func__);
		return -EINVAL;
	}

	gpio_mclk = of_get_named_gpio(dnode, "gpio_mclk", 0);
	if (gpio_is_valid(gpio_mclk)) {
		if (gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW")) {
			dev_err(dev, "%s: failed to gpio request mclk\n", __func__);
			return -ENODEV;
		}
		gpio_free(gpio_mclk);
	} else {
		dev_err(dev, "%s: failed to get mclk\n", __func__);
		return -EINVAL;
	}

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF);

	/* Normal On */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VLDO41_PMIC_FCAM_AVDD_2P8", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 1, 1000);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 2000);

	/* Normal Off */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 0, 0);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VLDO41_PMIC_FCAM_AVDD_2P8", PIN_REGULATOR, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 0, 0);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);

	/* SENSOR FACTORY TEST - POWER ON */
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_none, "VLDO41_PMIC_FCAM_AVDD_2P8", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 1, 1000);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, SRT_ACQUIRE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);

	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 2000);

	/* SENSOR FACTORY TEST - POWER Off */
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 0, 0);

	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "VLDO41_PMIC_FCAM_AVDD_2P8", PIN_REGULATOR, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 0, 0);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, SRT_RELEASE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int sensor_module_5e9_power_setpin_1(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_none = 0;
	int gpio_avdd_en = 0;
	int gpio_iovdd_en = 0;
	int gpio_mclk = 0;
	struct fimc_is_core *core;

	FIMC_BUG(!dev);

	dnode = dev->of_node;

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	dev_info(dev, "%s E v4\n", __func__);

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get gpio_reset\n");
	} else {
		gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_reset);
	}

	gpio_avdd_en = of_get_named_gpio(dnode, "gpio_avdd_en", 0);
	if (gpio_is_valid(gpio_avdd_en)) {
		if (gpio_request_one(gpio_avdd_en, GPIOF_OUT_INIT_LOW, "CAM_AVDD_EN_LOW")) {
			dev_err(dev, "%s: failed to gpio request avdd_en\n", __func__);
			return -ENODEV;
		}
		gpio_free(gpio_avdd_en);
	} else {
		dev_err(dev, "%s: failed to get avdd_en\n", __func__);
		return -EINVAL;
	}

	gpio_iovdd_en = of_get_named_gpio(dnode, "gpio_iovdd_en", 0);
	if (gpio_is_valid(gpio_iovdd_en)) {
		if (gpio_request_one(gpio_iovdd_en, GPIOF_OUT_INIT_LOW, "CAM_IOVDD_EN_LOW")) {
			dev_err(dev, "%s: failed to gpio request iovdd_en\n", __func__);
			return -ENODEV;
		}
		gpio_free(gpio_iovdd_en);
	} else {
		dev_err(dev, "%s: failed to get iovdd_en\n", __func__);
		return -EINVAL;
	}

	gpio_mclk = of_get_named_gpio(dnode, "gpio_mclk", 0);
	if (gpio_is_valid(gpio_mclk)) {
		if (gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW")) {
			dev_err(dev, "%s: failed to gpio request mclk\n", __func__);
			return -ENODEV;
		}
		gpio_free(gpio_mclk);
	} else {
		dev_err(dev, "%s: failed to get mclk\n", __func__);
		return -EINVAL;
	}

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF);

	/* Normal On */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 100);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 1, 0);
	/* TODO: need to check */
	/* SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VLDO41_PMIC_FCAM_AVDD_2P8", PIN_REGULATOR, 1, 0); */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_avdd_en, "avdd_en", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 1, 1000);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 2000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "retry delay", PIN_RETRY, 0, 2000);

	/* Normal Off */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_avdd_en, "avdd_en", PIN_OUTPUT, 0, 0);
	/* TODO: need to check */
	/* SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VLDO41_PMIC_FCAM_AVDD_2P8", PIN_REGULATOR, 0, 0); */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 0, 0);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);

	/* SENSOR FACTORY TEST - POWER ON */
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 100);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_avdd_en, "avdd_en", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 1, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 2000);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_ON, gpio_none, "retry delay", PIN_RETRY, 0, 2000);

	/* Normal Off */
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_none, "VLDO44_PMIC_DCAM_DVDD_1P2", PIN_REGULATOR, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_avdd_en, "avdd_en", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_FACTORY, GPIO_SCENARIO_OFF, gpio_iovdd_en, "iovdd_en", PIN_OUTPUT, 0, 0);

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int (*sensor_module_5e9_power_setpin[MAX_5E9_SETPIN_CNT])(struct device *dev,
	struct exynos_platform_fimc_is_module *pdata) = {
	sensor_module_5e9_power_setpin_0,
	sensor_module_5e9_power_setpin_1
};

static int __init sensor_module_5e9_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_module;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device;
	struct sensor_open_extended *ext;
	struct exynos_platform_fimc_is_module *pdata;
	struct device *dev;
	struct pinctrl_state *s;
	int power_seq_idx = 0;
	int setfile_idx = 0;

	FIMC_BUG(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &pdev->dev;

	if (of_property_read_bool(dev->of_node, "power_seq_idx")) {
		ret = of_property_read_u32(dev->of_node, "power_seq_idx", &power_seq_idx);
		if (ret) {
			warn("power_seq_idx read is fail(%d)", ret);
		} else {
			if (power_seq_idx >= MAX_5E9_SETPIN_CNT) {
				warn("wrong power_seq_idx(%d >= %d) set as default\n",
						power_seq_idx, MAX_5E9_SETPIN_CNT);
				power_seq_idx = 0;
			}
		}
	} else {
		power_seq_idx = 0;
	}
	probe_info("%s power_seq_idx(%d)\n", __func__, power_seq_idx);

	fimc_is_module_parse_dt(dev, sensor_module_5e9_power_setpin[power_seq_idx]);

	if (of_property_read_bool(dev->of_node, "setfile_idx")) {
		ret = of_property_read_u32(dev->of_node, "setfile_idx", &setfile_idx);
		if (ret) {
			warn("setfile_idx read is fail(%d)", ret);
			setfile_idx = 0;
		}
	} else {
		setfile_idx = 0;
	}
	probe_info("%s setfile_idx(%d)\n", __func__, setfile_idx);

	pdata = dev_get_platdata(dev);
	device = &core->sensor[pdata->id];

	subdev_module = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_module) {
		ret = -ENOMEM;
		goto p_err;
	}

	probe_info("%s pdta->id(%d), module_enum id = %d\n", __func__, pdata->id, atomic_read(&device->module_count));
	module = &device->module_enum[atomic_read(&device->module_count)];
	atomic_inc(&device->module_count);
	clear_bit(FIMC_IS_MODULE_GPIO_ON, &module->state);
	module->pdata = pdata;
	module->dev = dev;
	module->sensor_id = SENSOR_NAME_S5K5E9;
	module->subdev = subdev_module;
	module->device = pdata->id;
	module->client = NULL;
	module->active_width = 2592;
	module->active_height = 1944;
	module->margin_left = 0;
	module->margin_right = 0;
	module->margin_top = 0;
	module->margin_bottom = 0;
	module->pixel_width = module->active_width;
	module->pixel_height = module->active_height;
	module->max_framerate = 120;
	module->position = pdata->position;
	module->bitwidth = 10;
	module->sensor_maker = "SLSI";
	module->sensor_name = "S5K5E9";
	if (setfile_idx)
		module->setfile_name = "setfile_5e9_troika.bin";
	else
		module->setfile_name = "setfile_5e9.bin";
	module->cfgs = ARRAY_SIZE(config_module_5e9);
	module->cfg = config_module_5e9;
	module->ops = NULL;
	/* Sensor peri */
	module->private_data = kzalloc(sizeof(struct fimc_is_device_sensor_peri), GFP_KERNEL);
	if (!module->private_data) {
		ret = -ENOMEM;
		goto p_err;
	}
	fimc_is_sensor_peri_probe((struct fimc_is_device_sensor_peri *)module->private_data);
	PERI_SET_MODULE(module);

	ext = &module->ext;

	ext->sensor_con.product_name = module->sensor_id;
	ext->sensor_con.peri_type = SE_I2C;
	ext->sensor_con.peri_setting.i2c.channel = pdata->sensor_i2c_ch;
	ext->sensor_con.peri_setting.i2c.slave_address = pdata->sensor_i2c_addr;
	ext->sensor_con.peri_setting.i2c.speed = 400000;

	if (pdata->af_product_name !=  ACTUATOR_NAME_NOTHING) {
		ext->actuator_con.product_name = pdata->af_product_name;
		ext->actuator_con.peri_type = SE_I2C;
		ext->actuator_con.peri_setting.i2c.channel = pdata->af_i2c_ch;
		ext->actuator_con.peri_setting.i2c.slave_address = pdata->af_i2c_addr;
		ext->actuator_con.peri_setting.i2c.speed = 400000;
	}

	if (pdata->flash_product_name != FLADRV_NAME_NOTHING) {
		ext->flash_con.product_name = pdata->flash_product_name;
		ext->flash_con.peri_type = SE_GPIO;
		ext->flash_con.peri_setting.gpio.first_gpio_port_no = pdata->flash_first_gpio;
		ext->flash_con.peri_setting.gpio.second_gpio_port_no = pdata->flash_second_gpio;
	}

	ext->from_con.product_name = FROMDRV_NAME_NOTHING;

	if (pdata->preprocessor_product_name != PREPROCESSOR_NAME_NOTHING) {
		ext->preprocessor_con.product_name = pdata->preprocessor_product_name;
		ext->preprocessor_con.peri_info0.valid = true;
		ext->preprocessor_con.peri_info0.peri_type = SE_SPI;
		ext->preprocessor_con.peri_info0.peri_setting.spi.channel = pdata->preprocessor_spi_channel;
		ext->preprocessor_con.peri_info1.valid = true;
		ext->preprocessor_con.peri_info1.peri_type = SE_I2C;
		ext->preprocessor_con.peri_info1.peri_setting.i2c.channel = pdata->preprocessor_i2c_ch;
		ext->preprocessor_con.peri_info1.peri_setting.i2c.slave_address = pdata->preprocessor_i2c_addr;
		ext->preprocessor_con.peri_info1.peri_setting.i2c.speed = 400000;
		ext->preprocessor_con.peri_info2.valid = true;
		ext->preprocessor_con.peri_info2.peri_type = SE_DMA;
		ext->preprocessor_con.peri_info2.peri_setting.dma.channel = FLITE_ID_D;
	} else {
		ext->preprocessor_con.product_name = pdata->preprocessor_product_name;
	}

	if (pdata->ois_product_name != OIS_NAME_NOTHING) {
		ext->ois_con.product_name = pdata->ois_product_name;
		ext->ois_con.peri_type = SE_I2C;
		ext->ois_con.peri_setting.i2c.channel = pdata->ois_i2c_ch;
		ext->ois_con.peri_setting.i2c.slave_address = pdata->ois_i2c_addr;
		ext->ois_con.peri_setting.i2c.speed = 400000;
	} else {
		ext->ois_con.product_name = pdata->ois_product_name;
		ext->ois_con.peri_type = SE_NULL;
	}

	if (pdata->eeprom_product_name != EEPROM_NAME_NOTHING) {
		ext->eeprom_con.product_name = pdata->eeprom_product_name;
		ext->eeprom_con.peri_type = SE_I2C;
		ext->eeprom_con.peri_setting.i2c.channel = pdata->eeprom_i2c_ch;
		ext->eeprom_con.peri_setting.i2c.slave_address = pdata->eeprom_i2c_addr;
		ext->eeprom_con.peri_setting.i2c.speed = 400000;
	} else {
		ext->eeprom_con.product_name = pdata->eeprom_product_name;
		ext->eeprom_con.peri_type = SE_NULL;
	}

	v4l2_subdev_init(subdev_module, &subdev_ops);

	v4l2_set_subdevdata(subdev_module, module);
	v4l2_set_subdev_hostdata(subdev_module, device);
	snprintf(subdev_module->name, V4L2_SUBDEV_NAME_SIZE, "sensor-subdev.%d", module->sensor_id);

	s = pinctrl_lookup_state(pdata->pinctrl, "release");

	if (pinctrl_select_state(pdata->pinctrl, s) < 0) {
		probe_err("pinctrl_select_state is fail\n");
		goto p_err;
	}

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static const struct of_device_id exynos_fimc_is_sensor_module_5e9_match[] = {
	{
		.compatible = "samsung,sensor-module-5e9",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_sensor_module_5e9_match);

static struct platform_driver sensor_module_5e9_driver = {
	.driver = {
		.name   = "FIMC-IS-SENSOR-MODULE-5E9",
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_5e9_match,
	}
};

static int __init fimc_is_sensor_module_5e9_init(void)
{
	int ret;

	ret = platform_driver_probe(&sensor_module_5e9_driver,
				sensor_module_5e9_probe);
	if (ret)
		err("failed to probe %s driver: %d\n",
			sensor_module_5e9_driver.driver.name, ret);

	return ret;
}
late_initcall(fimc_is_sensor_module_5e9_init);
