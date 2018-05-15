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
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
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
#include "fimc-is-device-module-imx576.h"

// Reference Version : 'IMX576-AAKH5_SAM-Set-26MHz-DPHY_RegisterSetting_ver2.00-9.00_MP_180712.xlsx'
// Reference Version : 'IMX576-AAKH5_SAM-Set-26MHz-DPHY_RegisterSetting_ver2.00-8.00_MP0_180712.xlsx'

/*
 * [Mode Information]
 *	- Global Setting -
 *
 *	- 2X2 BINNING -
 *	[0] REG_A: Single Still Preview (4:3)   : 2832x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[1] REG_B: Single Still Preview (16:9)  : 2832x1592@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[2] REG_C: Single Still Preview (18.5:9): 2832x1376@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[3] REG_D: Single Still Preview (1:1)   : 2124x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_HDR -
 *	[4] REG_E: Single Still 3HDR (4:3)      : 2832x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[5] REG_F: Single Still 3HDR (16:9)     : 2832x1592@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[6] REG_G: Single Still 3HDR (18.5:9)   : 2832x1376@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[7] REG_H: Single Still 3HDR (1:1)      : 2124x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_REMOSAIC -
 *	[8] REG_I: Single Still Capture (4:3)   : 5664X4248@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[9] REG_J: Single Still Capture (16:9)  : 5664X3184@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[10]REG_K: Single Still Capture (18.5:9): 5664X2752@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[11]REG_L: Single Still Capture (1:1)   : 4248X4248@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- Super Slow Motion (SSM) -
 *	[12]REG_M: Super Slow Motion (16:9)     : 1872x1052@240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[13]REG_N: Super Slow Motion (16:9)     : 1920x1080@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[14]REG_O: Super Slow Motion (16:9)     : 1280x720 @240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[15]REG_U: Super Slow Motion (16:9)     : 1280x720 @120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- FAST AE -
 *	[16]REG_R: Single Preview Fast(4:3)     : 2832x2124@117,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[17]REG_S: Single Preview Fast(4:3)     : 2832x2124@ 60,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[18]REG_T: Single Preview Fast(16:9)    : 2832x1592@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 */

static struct fimc_is_sensor_cfg config_imx576[] = {
	/* 0 : 2832x2124@30fps 2X2BIN */
	FIMC_IS_SENSOR_CFG(2832, 2124,  30, 0, 0, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2832, 2124), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 1 : 2832x1592@30fps 2X2BIN */
	FIMC_IS_SENSOR_CFG(2832, 1592,  30, 0, 1, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2832, 1592), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 2 : 2832x1376@30fps 2X2BIN */
	FIMC_IS_SENSOR_CFG(2832, 1376,  30, 0, 2, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2832, 1376), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 3 : 2124x2124@30fps 2X2BIN */
	FIMC_IS_SENSOR_CFG(2124, 2124,  30, 0, 3, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  2124, 2124), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 4 : 2832x2124@30fps QBC_HDR */
	FIMC_IS_SENSOR_CFG(2832, 2124, 30, 0, 4, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2832, 2124), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 5 : 2832x1592@30fps QBC_HDR */
	FIMC_IS_SENSOR_CFG(2832, 1592, 30, 0, 5, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2832, 1592), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 6 : 2832x1376@30fps QBC_HDR */
	FIMC_IS_SENSOR_CFG(2832,  1376, 30, 0, 6, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2832, 1376), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 7 : 2124x2124@30fps QBC_HDR */
	FIMC_IS_SENSOR_CFG(2124, 2124,  30, 0, 7, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 2124, 2124), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 8 : 5664X4248@30fps QBC_REMOSAIC */
	FIMC_IS_SENSOR_CFG(5664, 4248,  30, 0, 8, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 5664, 4248), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 9 : 5664X3184@30fps QBC_REMOSAIC */
	FIMC_IS_SENSOR_CFG(5664, 3184,  30, 0, 9, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 5664, 3184), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 10 : 5664X2752@30fps QBC_REMOSAIC */
	FIMC_IS_SENSOR_CFG(5664, 2752,  30, 0, 10, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10, 5664, 2752), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 11 : 4248X4248@30fps QBC_REMOSAIC */
	FIMC_IS_SENSOR_CFG(4248,  4248,  30, 0, 11, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  4248,  4248), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 12 : 1872x1052@240fps Super Slow Motion (SSM) */
	FIMC_IS_SENSOR_CFG(1872,  1052,  240, 0, 12, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  1872,  1052), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 13 : 1920x1080@120fps Super Slow Motion (SSM) */
	FIMC_IS_SENSOR_CFG(1920,  1080,  120, 0, 13, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  1920,  1080), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 14 : 1280x720 @240fps Super Slow Motion (SSM) */
	FIMC_IS_SENSOR_CFG(1280,  720,  240, 0, 14, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  1280,  720), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 15 : 1280x720 @120fps Super Slow Motion (SSM) */
	FIMC_IS_SENSOR_CFG(1280,  720,	120, 0, 15, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  1280,  720), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 16 : 2832x2124@117fps FAST AE */
	FIMC_IS_SENSOR_CFG(2832,  2124,  117, 0, 16, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  2832,  2124), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 17 : 2832x2124@60fps FAST AE */
	FIMC_IS_SENSOR_CFG(2832,  2124,  60, 0, 17, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  2832,  2124), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(3, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0)),
	/* 18 : 2832x1592@120fps FAST AE */
	FIMC_IS_SENSOR_CFG(2832,  1592,  120, 0, 18, CSI_DATA_LANES_4, 2054, CSI_MODE_DT_ONLY, PD_NONE,
		VC_IN(0, HW_FORMAT_RAW10,  2832,  1592), VC_OUT(HW_FORMAT_RAW10, VC_NOTHING, 0, 0),
		VC_IN(1, HW_FORMAT_UNKNOWN, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
		VC_IN(2, HW_FORMAT_EMBEDDED_8BIT, 0, 0), VC_OUT(HW_FORMAT_UNKNOWN, VC_NOTHING, 0, 0),
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

static int sensor_imx576_power_setpin(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_mclk = 0;
	int gpio_reset = 0;
	int gpio_none = 0;
	int gpio_camcore_1p05_en = 0;
	int ret;
	struct fimc_is_core *core;

	FIMC_BUG(!dev);

	dnode = dev->of_node;

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);

	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	dev_info(dev, "%s E v4\n", __func__);

	gpio_mclk = of_get_named_gpio(dnode, "gpio_mclk", 0);
	if (!gpio_is_valid(gpio_mclk)) {
		dev_err(dev, "failed to get gpio_mclk\n");
		return -EINVAL;
	}

	gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW");
	gpio_free(gpio_mclk);

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get PIN_RESET\n");
		return -EINVAL;
	}

	gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
	gpio_free(gpio_reset);

	gpio_camcore_1p05_en = of_get_named_gpio(dnode, "gpio_camcore_1p05_en", 0);
	if (!gpio_is_valid(gpio_camcore_1p05_en)) {
		dev_info(dev, "failed to get gpio_camcore_1p05_en\n");
	} else {
		gpio_request_one(gpio_camcore_1p05_en, GPIOF_OUT_INIT_LOW, "CAMCORE_1P05_EN");
		gpio_free(gpio_camcore_1p05_en);
	}

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/* Normal on */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "vdd_ldo41", PIN_REGULATOR, 1, 0); /* VDD_RCAM1_A2P8 */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camcore_1p05_en, "cam_core high", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "vdd_ldo42", PIN_REGULATOR, 1, 0); /* VDD_RCAM1_IO_1P8 */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "vdd_ldo38", PIN_REGULATOR, 1, 0); /* VDD_RCAM1_AF_2P8 */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 1000);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);

	/* Normal off */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
			&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "vdd_ldo41", PIN_REGULATOR, 0, 0); /* VDD_RCAM1_A2P8 */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camcore_1p05_en, "cam_core high", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "vdd_ldo42", PIN_REGULATOR, 0, 0); /* VDD_RCAM1_IO_1P8 */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "vdd_ldo38", PIN_REGULATOR, 0, 0); /* VDD_RCAM1_AF_2P8 */


	/* READ_ROM - POWER ON */
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "vdd_ldo38", PIN_REGULATOR, 1, 0);
	/* READ_ROM - POWER OFF */
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "vdd_ldo38", PIN_REGULATOR, 0, 0);

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int __init sensor_module_imx576_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_module;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device;
	struct sensor_open_extended *ext;
	struct exynos_platform_fimc_is_module *pdata;
	struct device *dev;
	int ch, vc_idx;
	struct pinctrl_state *s;

	FIMC_BUG(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &pdev->dev;

	fimc_is_module_parse_dt(dev, sensor_imx576_power_setpin);

	pdata = dev_get_platdata(dev);
	device = &core->sensor[pdata->id];

	subdev_module = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_module) {
		ret = -ENOMEM;
		goto p_err;
	}

	module = &device->module_enum[atomic_read(&device->module_count)];
	atomic_inc(&device->module_count);
	clear_bit(FIMC_IS_MODULE_GPIO_ON, &module->state);
	module->pdata = pdata;
	module->dev = dev;
	module->sensor_id = SENSOR_NAME_IMX576;
	module->subdev = subdev_module;
	module->device = pdata->id;
	module->client = NULL;
	module->active_width = 5664;
	module->active_height = 4248;
	module->margin_left = 0;
	module->margin_right = 0;
	module->margin_top = 0;
	module->margin_bottom = 0;
	module->pixel_width = module->active_width + 0;
	module->pixel_height = module->active_height + 0;
	module->max_framerate = 300;
	module->position = pdata->position;
	module->bitwidth = 10;
	module->sensor_maker = "SONY";
	module->sensor_name = "IMX576";
	module->setfile_name = "setfile_imx576.bin";
	module->cfgs = ARRAY_SIZE(config_imx576);
	module->cfg = config_imx576;
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
	ext->sensor_con.product_name = module->sensor_id /*SENSOR_NAME_IMX576*/;
	ext->sensor_con.peri_type = SE_I2C;
	ext->sensor_con.peri_setting.i2c.channel = pdata->sensor_i2c_ch;
	ext->sensor_con.peri_setting.i2c.slave_address = pdata->sensor_i2c_addr;
	ext->sensor_con.peri_setting.i2c.speed = 1000000;

	ext->actuator_con.product_name = ACTUATOR_NAME_NOTHING;
	ext->flash_con.product_name = FLADRV_NAME_NOTHING;
	ext->from_con.product_name = FROMDRV_NAME_NOTHING;
	ext->preprocessor_con.product_name = PREPROCESSOR_NAME_NOTHING;
	ext->ois_con.product_name = OIS_NAME_NOTHING;

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

	/* ToDo: ???? */
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
		if (pdata->preprocessor_dma_channel == DMA_CH_NOT_DEFINED)
			ext->preprocessor_con.peri_info2.peri_setting.dma.channel = FLITE_ID_D;
		else
			ext->preprocessor_con.peri_info2.peri_setting.dma.channel = pdata->preprocessor_dma_channel;
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

	v4l2_subdev_init(subdev_module, &subdev_ops);

	v4l2_set_subdevdata(subdev_module, module);
	v4l2_set_subdev_hostdata(subdev_module, device);
	snprintf(subdev_module->name, V4L2_SUBDEV_NAME_SIZE, "sensor-subdev.%d", module->sensor_id);

	s = pinctrl_lookup_state(pdata->pinctrl, "release");

	if (pinctrl_select_state(pdata->pinctrl, s) < 0) {
		probe_err("pinctrl_select_state is fail\n");
		goto p_err;
	}
p_err:
	probe_info("%s(%d)\n", __func__, ret);
	return ret;
}

static const struct of_device_id exynos_fimc_is_sensor_module_imx576_match[] = {
	{
		.compatible = "samsung,sensor-module-imx576",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_sensor_module_imx576_match);

static struct platform_driver sensor_module_imx576_driver = {
	.driver = {
		.name   = "FIMC-IS-SENSOR-MODULE-IMX576",
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_imx576_match,
	}
};

static int __init fimc_is_sensor_module_imx576_init(void)
{
	int ret;

	ret = platform_driver_probe(&sensor_module_imx576_driver,
				sensor_module_imx576_probe);
	if (ret)
		err("failed to probe %s driver: %d\n",
			sensor_module_imx576_driver.driver.name, ret);

	return ret;
}
late_initcall(fimc_is_sensor_module_imx576_init);

