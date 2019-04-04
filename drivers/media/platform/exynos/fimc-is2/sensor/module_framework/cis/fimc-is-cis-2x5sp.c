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
#include "fimc-is-param.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-resourcemgr.h"
#include "fimc-is-dt.h"
#include "fimc-is-cis-2x5sp.h"
#include "fimc-is-cis-2x5sp-setA.h"
#include "fimc-is-cis-2x5sp-setB.h"

#include "fimc-is-helper-i2c.h"

#define SENSOR_NAME "S5K2X5SP"
/* #define DEBUG_2X5SP_PLL */

static const struct v4l2_subdev_ops subdev_ops;

static const u32 *sensor_2x5sp_global;
static u32 sensor_2x5sp_global_size;
static const u32 **sensor_2x5sp_setfiles;
static const u32 *sensor_2x5sp_setfile_sizes;
static const u32 *sensor_2x5sp_otp_initial;
static u32 sensor_2x5sp_otp_initial_size;
static const struct sensor_pll_info_compact **sensor_2x5sp_pllinfos;
static u32 sensor_2x5sp_max_setfile_num;

static const u32 *sensor_2x5sp_setfile_throttling;
static const struct sensor_pll_info_compact *sensor_2x5sp_pllinfo_throttling;

static void sensor_2x5sp_cis_data_calculation(const struct sensor_pll_info_compact *pll_info_compact, cis_shared_data *cis_data)
{
	u32 vt_pix_clk_hz = 0;
	u32 frame_rate = 0, max_fps = 0, frame_valid_us = 0;

	FIMC_BUG_VOID(!pll_info_compact);

	/* 1. get pclk value from pll info */
	vt_pix_clk_hz = pll_info_compact->pclk;

	dbg_sensor(1, "ext_clock(%d), mipi_datarate(%d), pclk(%d)\n",
			pll_info_compact->ext_clk, pll_info_compact->mipi_datarate, pll_info_compact->pclk);

	/* 2. the time of processing one frame calculation (us) */
	cis_data->min_frame_us_time = (pll_info_compact->frame_length_lines * pll_info_compact->line_length_pck
					/ (vt_pix_clk_hz / (1000 * 1000)));
	cis_data->cur_frame_us_time = cis_data->min_frame_us_time;

	/* 3. FPS calculation */
	frame_rate = vt_pix_clk_hz / (pll_info_compact->frame_length_lines * pll_info_compact->line_length_pck);
	dbg_sensor(1, "frame_rate (%d) = vt_pix_clk_hz(%d) / "
		KERN_CONT "(pll_info_compact->frame_length_lines(%d) * pll_info_compact->line_length_pck(%d))\n",
		frame_rate, vt_pix_clk_hz, pll_info_compact->frame_length_lines, pll_info_compact->line_length_pck);

	/* calculate max fps */
	max_fps = (vt_pix_clk_hz * 10) / (pll_info_compact->frame_length_lines * pll_info_compact->line_length_pck);
	max_fps = (max_fps % 10 >= 5 ? frame_rate + 1 : frame_rate);

	cis_data->pclk = vt_pix_clk_hz;
	cis_data->max_fps = max_fps;
	cis_data->frame_length_lines = pll_info_compact->frame_length_lines;
	cis_data->line_length_pck = pll_info_compact->line_length_pck;
	cis_data->line_readOut_time = sensor_cis_do_div64((u64)cis_data->line_length_pck * (u64)(1000 * 1000 * 1000), cis_data->pclk);
	cis_data->rolling_shutter_skew = (cis_data->cur_height - 1) * cis_data->line_readOut_time;
	cis_data->stream_on = false;

	/* Frame valid time calcuration */
	frame_valid_us = sensor_cis_do_div64((u64)cis_data->cur_height * (u64)cis_data->line_length_pck * (u64)(1000 * 1000), cis_data->pclk);
	cis_data->frame_valid_us_time = (int)frame_valid_us;

	dbg_sensor(1, "%s\n", __func__);
	dbg_sensor(1, "Sensor size(%d x %d) setting: SUCCESS!\n",
					cis_data->cur_width, cis_data->cur_height);
	dbg_sensor(1, "Frame Valid(us): %d\n", frame_valid_us);
	dbg_sensor(1, "rolling_shutter_skew: %lld\n", cis_data->rolling_shutter_skew);

	dbg_sensor(1, "Fps: %d, max fps(%d)\n", frame_rate, cis_data->max_fps);
	dbg_sensor(1, "min_frame_time(%d us)\n", cis_data->min_frame_us_time);
	dbg_sensor(1, "Pixel rate(Mbps): %d\n", cis_data->pclk / 1000000);

	/* Frame period calculation */
	cis_data->frame_time = (cis_data->line_readOut_time * cis_data->cur_height / 1000);
	cis_data->rolling_shutter_skew = (cis_data->cur_height - 1) * cis_data->line_readOut_time;

	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__,
		cis_data->frame_time, cis_data->rolling_shutter_skew);

	/* Constant values */
	cis_data->min_fine_integration_time = SENSOR_2X5SP_FINE_INTEGRATION_TIME_MIN;
	cis_data->max_fine_integration_time = cis_data->cur_width;
	cis_data->min_coarse_integration_time = SENSOR_2X5SP_COARSE_INTEGRATION_TIME_MIN;
	cis_data->max_margin_coarse_integration_time = SENSOR_2X5SP_COARSE_INTEGRATION_TIME_MAX_MARGIN;
}

int sensor_2x5sp_cis_check_rev(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u8 rev = 0;
	struct i2c_client *client;
	struct fimc_is_cis *cis = NULL;

	WARN_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	WARN_ON(!cis);
	WARN_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	memset(cis->cis_data, 0, sizeof(cis_shared_data));
	cis->rev_flag = false;

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = fimc_is_sensor_read8(client, 0x0002, &rev);
	if (ret < 0) {
		cis->rev_flag = true;
		ret = -EAGAIN;
	} else {
		cis->cis_data->cis_rev = rev;
		pr_info("%s : Rev. 0x%X\n", __func__, rev);
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_otp_check_awb_ratio(char *unit, char *golden, char *limit)
{
	int ret = 0;

	float r_g_min = (float)(limit[0]) / 1000;
	float r_g_max = (float)(limit[1]) / 1000;
	float b_g_min = (float)(limit[2]) / 1000;
	float b_g_max = (float)(limit[3]) / 1000;

	float rg = (float) ((unit[1]) | (unit[0] << 8)) / 16384;
	float bg = (float) ((unit[3]) | (unit[2] << 8)) / 16384;

	float golden_rg = (float) ((golden[1]) | (golden[0] << 8)) / 16384;
	float golden_bg = (float) ((golden[3]) | (golden[2] << 8)) / 16384;

	if (rg < (golden_rg - r_g_min) || rg > (golden_rg + r_g_max)) {
		err("%s(): Final RG calibration factors out of range! rg=0x%x golden_rg=0x%x",
			__func__, (unit[1] | unit[0] << 8), (golden[1] | golden[0] << 8));
		ret = 1;
	}

	if (bg < (golden_bg - b_g_min) || bg > (golden_bg + b_g_max)) {
		err("%s(): Final BG calibration factors out of range! bg=0x%x, golden_bg=0x%x",
			__func__, (unit[3] | unit[2] << 8), (golden[3] | golden[2] << 8));
		ret = 1;
	}

	return ret;
}

int sensor_2x5sp_cis_otp_check_crc(struct v4l2_subdev *subdev,
		struct fimc_is_device_sensor *device, int group)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	char *check_buf = (char *)&device->otp_cal_buf[0][0];

	switch (group) {
	case OTP_GROUP_ONE:
		/* OTP Group1 CRC check */
		crc_value = ((device->otp_cal_buf[254][60] << 8) | (device->otp_cal_buf[254][61]));
		crc16 = sensor_cis_otp_get_crc16(&check_buf[OTP_GRP1_AWB_CRC_START], OTP_GRP1_AWB_CRC_SIZE);
		if (crc_value != crc16) {
			sensor_cis_otp_data_set(&check_buf[OTP_GRP1_AWB_CRC_START], "awb",
				OTP_GRP1_AWB_CRC_SIZE, 0xff);
			err("GR1: Error to AWB CRC16 : 0x%x, cal buffer CRC: 0x%x", crc16, crc_value);
			ret = -EINVAL;
		} else
			info("GR1: AWB CRC16 : 0x%x, cal buffer CRC: 0x%x\n", crc16, crc_value);

		crc_value = ((device->otp_cal_buf[254][62] << 8) | (device->otp_cal_buf[254][63]));
		crc16 = sensor_cis_otp_get_crc16(&check_buf[OTP_GRP1_LSC_XTC_CRC_START], OTP_GRP1_LSC_XTC_CRC_SIZE);
		if (crc_value != crc16) {
			sensor_cis_otp_data_set(&check_buf[OTP_GRP1_LSC_XTC_CRC_START], "lsc_xtc",
				OTP_GRP1_LSC_XTC_CRC_SIZE, 0xff);
			err("GR1: Error to LSC & XTC CRC16 : 0x%x, cal buffer CRC: 0x%x", crc16, crc_value);
			ret = -EINVAL;
		} else
			info("GR1: LSC & XTC CRC16 : 0x%x, cal buffer CRC: 0x%x\n", crc16, crc_value);
		break;
	case OTP_GROUP_TWO:
		/* OTP Group2 CRC check */
		crc_value = ((device->otp_cal_buf[255][60] << 8) | (device->otp_cal_buf[255][61]));
		crc16 = sensor_cis_otp_get_crc16(&check_buf[OTP_GRP2_AWB_CRC_START], OTP_GRP2_AWB_CRC_SIZE);
		if (crc_value != crc16) {
			sensor_cis_otp_data_set(&check_buf[OTP_GRP2_AWB_CRC_START], "awb",
				OTP_GRP2_AWB_CRC_SIZE, 0xff);
			err("GR2: Error to AWB CRC16 : 0x%x, cal buffer CRC: 0x%x", crc16, crc_value);
			ret = -EINVAL;
		} else
			info("GR2: AWB CRC16 : 0x%x, cal buffer CRC: 0x%x\n", crc16, crc_value);

		crc_value = ((device->otp_cal_buf[255][62] << 8) | (device->otp_cal_buf[255][63]));
		crc16 = sensor_cis_otp_get_crc16(&check_buf[OTP_GRP2_LSC_XTC_CRC_START], OTP_GRP2_LSC_XTC_CRC_SIZE);
		if (crc_value != crc16) {
			sensor_cis_otp_data_set(&check_buf[OTP_GRP2_LSC_XTC_CRC_START], "lsc_xtc",
				OTP_GRP2_LSC_XTC_CRC_SIZE, 0xff);
			err("GR2: Error to LSC & XTC CRC16 : 0x%x, cal buffer CRC: 0x%x", crc16, crc_value);
			ret = -EINVAL;
		} else
			info("GR2: LSC & XTC CRC16 : 0x%x, cal buffer CRC: 0x%x\n", crc16, crc_value);
		break;
	default:
		err("invalid OTP group when crc check(%d), check map data", group);
		break;
	}

	return ret;
}

static int sensor_2x5sp_cis_otp_check(struct fimc_is_device_sensor *device, int group)
{
	int ret = 0;
	u16 group_flag = 0;

	switch (group) {
	case OTP_GROUP_ONE:
		/* Group1 valid check */
		group_flag = device->otp_cal_buf[254][0];
		if (group_flag != OTP_DATA_VALID) {
			err("error Group1 OTP data invalid(0x%x)", group_flag);
			ret = -EINVAL;
		} else {
			ret = OTP_GROUP_ONE;
		}

		break;
	case OTP_GROUP_TWO:
		/* First check Group2 data valid */
		group_flag = device->otp_cal_buf[255][0];
		if (group_flag != OTP_DATA_VALID) {
			err("error Group2 OTP data invalid(0x%x)", group_flag);
			ret = -EINVAL;

		} else {
			ret = OTP_GROUP_TWO;
		}

		break;
	default:
		err("invalid OTP group when invalid check(%d), check map data", group);
		break;
	}

	return ret;
}

int sensor_2x5sp_cis_otp_read(struct v4l2_subdev *subdev, struct fimc_is_device_sensor *device)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	u16 val = 1, page;
	int i;
	int retry;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (!cis) {
		err("cis is NULL");
		return -EINVAL;
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -ENODEV;
	}

	info("OTP read start\n");
	dbg_sensor(1, "%s, 1. sensor initial setting", __func__);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	/* Basic settings for OTP R/W */
	fimc_is_sensor_write16(client, 0x6028, 0x4000);
	fimc_is_sensor_write16(client, 0x6018, 0x0001);
	fimc_is_sensor_write16(client, 0x7002, 0x020C);
	fimc_is_sensor_write16(client, 0x6014, 0x0001);

	usleep_range(3000, 3001);

	fimc_is_sensor_write16(client, 0x0136, 0x1A00);
	fimc_is_sensor_write16(client, 0x0300, 0x0003);
	fimc_is_sensor_write16(client, 0x0302, 0x0001);
	fimc_is_sensor_write16(client, 0x0304, 0x0004);
	fimc_is_sensor_write16(client, 0x0306, 0x00DD);
	fimc_is_sensor_write16(client, 0x030C, 0x0001);
	fimc_is_sensor_write16(client, 0x0308, 0x0008);
	fimc_is_sensor_write16(client, 0x030A, 0x0001);
	fimc_is_sensor_write16(client, 0x030E, 0x0004);
	fimc_is_sensor_write16(client, 0x0310, 0x014A);
	fimc_is_sensor_write16(client, 0x0312, 0x0000);

	dbg_sensor(1, "%s, 2. sensor stream on", __func__);
	fimc_is_sensor_write16(client, 0x6028, 0x4000);
	fimc_is_sensor_write8(client, 0x0100, 0x01);

	/* wait streamon */
	CALL_CISOPS(cis, cis_wait_streamon, subdev);

	dbg_sensor(1, "%s, 3. page select & read cal", __func__);
	for (page = OTP_PAGE_START; page <= OTP_PAGE_END; page++) {
		/* page select & read start */
		fimc_is_sensor_write16(client, OTP_PAGE_SELECT, page);
		fimc_is_sensor_write16(client, OTP_PAGE_CTRL, 0x0100);

		/* wait 0x0A00 == 0 [0]: read completed with no errors */
		retry = 500;
		while (retry > 0 && val) {
			fimc_is_sensor_read16(client, OTP_PAGE_CTRL, &val);
			if (val == 0)
				break;

			usleep_range(100, 100);
			retry--;
		}

		if (!retry)
			err("%s: OTP page[%d] read fail with err(%d)\n",
				__func__, page, val);

		for (i = 0; i < OTP_PAGE_SIZE; i++) {
			fimc_is_sensor_read8(client, OTP_PAGE_BASE + i, &device->otp_cal_buf[page][i]);
			dbg_sensor(2, "cal: [%d][0x%x]: %x\n", page, OTP_PAGE_BASE + i, device->otp_cal_buf[page][i]);
		}

		/* make initial state */
		fimc_is_sensor_write16(client, OTP_PAGE_CTRL, 0x0000);
	}

	fimc_is_sensor_write8(client, 0x0100, 0x00);
	msleep(20);
	info("OTP read end\n");

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_otp(struct v4l2_subdev *subdev, struct fimc_is_device_sensor *device)
{
	int ret = 0;
	int i;
	int otp_group = 0x0;
	char *otp_buf = (char *)&device->otp_cal_buf[0][0];

	ret = sensor_cis_otp_read_file(OTP_DATA_PATH, (void *)device->otp_cal_buf, OTP_PAGE_SIZE * 256);
	if (ret) {
		/* OTP data read */
		ret = sensor_2x5sp_cis_otp_read(subdev, device);
		if (ret < 0) {
			sensor_cis_otp_data_set(&otp_buf[0], "all", OTP_PAGE_SIZE * 256, 0xff);
			err("Don't read to 2x5 OTP data");
			goto p_err;
		}

		/* Write to OTP data at file */
		ret = sensor_cis_otp_write_file(OTP_DATA_PATH, (void *)device->otp_cal_buf, OTP_PAGE_SIZE * 256);
		if (ret < 0) {
			sensor_cis_otp_data_set(&otp_buf[0], "all", OTP_PAGE_SIZE * 256, 0xff);
			err("2x5 OTP data don't file write");
			goto p_err;
		}
	}

	/* Need to first check GROUP2 */
	for (i = OTP_GROUP_MAX - 1; i >= OTP_GROUP_ONE; i--) {
		/* OTP valid check */
		otp_group = sensor_2x5sp_cis_otp_check(device, i);
		if (otp_group < 0) {
			if (i == OTP_GROUP_ONE) {
				sensor_cis_otp_data_set(&otp_buf[0], "all", OTP_PAGE_SIZE * 256, 0xff);
				err("All OTP data are invalid, check module");
				goto p_err;
			} else {
				continue;
			}
		} else {
			break;
		}
	}

	/* OTP CRC check */
	ret = sensor_2x5sp_cis_otp_check_crc(subdev, device, otp_group);
	if (ret < 0) {
		err("All OTP data CRC check fail, check module");

		device->cal_status[CAMERA_CRC_INDEX_AWB] = CRC_ERROR;
		goto p_err;
	} else {
		err("%s: OTP group%d data availble\n", __func__, otp_group);

		device->cal_status[CAMERA_CRC_INDEX_AWB] = CRC_NO_ERROR;

		ret = sensor_2x5sp_cis_otp_check_awb_ratio(&otp_buf[(253 + otp_group) * OTP_PAGE_SIZE + OTP_AWB_UNIT_OFFSET],
			&otp_buf[(253 + otp_group) * OTP_PAGE_SIZE + OTP_AWB_GOLDEN_OFFSET],
			&otp_buf[(253 + otp_group) * OTP_PAGE_SIZE + OTP_AWB_LIMIT_OFFSET]);

		if (ret) {
			err("%s(): 2X5 OTP AWB Group%d ratio out of limit(%d)", __func__, otp_group, ret);
			device->cal_status[CAMERA_CRC_INDEX_AWB] = LIMIT_FAILURE;
			ret = -1;
		}
	}

p_err:
	return ret;
}

/* CIS OPS */
int sensor_2x5sp_cis_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device = NULL;
	u32 setfile_index = 0;
	cis_setting_info setinfo;

	setinfo.return_value = 0;

	setinfo.param = NULL;

	FIMC_BUG(!subdev);

	module = (struct fimc_is_module_enum *)v4l2_get_subdev_hostdata(subdev);

	device = (struct fimc_is_device_sensor *)v4l2_get_subdev_hostdata(module->subdev);
	if (!device) {
		err("device sensor is NULL");
		ret = -ENODEV;
		goto p_err;
	}

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (!cis) {
		err("cis is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	FIMC_BUG(!cis->cis_data);
	memset(cis->cis_data, 0, sizeof(cis_shared_data));

	cis->cis_data->cur_width = SENSOR_2X5SP_MAX_WIDTH;
	cis->cis_data->cur_height = SENSOR_2X5SP_MAX_HEIGHT;
	cis->cis_data->low_expo_start = 33000;
	cis->need_mode_change = false;

	sensor_2x5sp_cis_data_calculation(sensor_2x5sp_pllinfos[setfile_index], cis->cis_data);

	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_min_exposure_time, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] min exposure time : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_max_exposure_time, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] max exposure time : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_min_analog_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] min again : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_max_analog_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] max again : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_min_digital_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] min dgain : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_max_digital_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] max dgain : %d\n", __func__, setinfo.return_value);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

	if (device->use_otp_cal) {
		ret = sensor_2x5sp_cis_otp(subdev, device);
		if (ret < 0) {
			err("2x5sp OTP data have probelm, check module");
			ret = 0;
		}
	}

p_err:
	return ret;
}

int sensor_2x5sp_cis_log_status(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client = NULL;
	u8 data8 = 0;
	u16 data16 = 0;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (!cis) {
		err("cis is NULL");
		return -ENODEV;
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -ENODEV;
	}

	I2C_MUTEX_LOCK(cis->i2c_lock);

	pr_err("[SEN:DUMP] *******************************\n");
	fimc_is_sensor_read16(client, 0x0000, &data16);
	pr_err("[SEN:DUMP] model_id(%x)\n", data16);
	fimc_is_sensor_read8(client, 0x0002, &data8);
	pr_err("[SEN:DUMP] revision_number(%x)\n", data8);
	fimc_is_sensor_read8(client, 0x0005, &data8);
	pr_err("[SEN:DUMP] frame_count(%x)\n", data8);
	fimc_is_sensor_read8(client, 0x0100, &data8);
	pr_err("[SEN:DUMP] mode_select(%x)\n", data8);

	sensor_cis_dump_registers(subdev, sensor_2x5sp_setfiles[0], sensor_2x5sp_setfile_sizes[0]);

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	pr_err("[SEN:DUMP] *******************************\n");

	return ret;
}

#if USE_GROUP_PARAM_HOLD
static int sensor_2x5sp_cis_group_param_hold_func(struct v4l2_subdev *subdev, unsigned int hold)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;
	struct i2c_client *client = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	if (hold == cis->cis_data->group_param_hold) {
		pr_debug("already group_param_hold (%d)\n", cis->cis_data->group_param_hold);
		goto p_err;
	}

	ret = fimc_is_sensor_write8(client, 0x0104, hold);
	if (ret < 0)
		goto p_err;

	cis->cis_data->group_param_hold = hold;
	ret = 1;
p_err:
	return ret;
}
#else
static inline int sensor_2x5sp_cis_group_param_hold_func(struct v4l2_subdev *subdev, unsigned int hold)
{ return 0; }
#endif

/* Input
 *	hold : true - hold, flase - no hold
 * Output
 *      return: 0 - no effect(already hold or no hold)
 *		positive - setted by request
 *		negative - ERROR value
 */
int sensor_2x5sp_cis_group_param_hold(struct v4l2_subdev *subdev, bool hold)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_2x5sp_cis_group_param_hold_func(subdev, hold);
	if (ret < 0)
		goto p_err;

p_err:
	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_set_global_setting(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_cis_set_registers(subdev, sensor_2x5sp_global, sensor_2x5sp_global_size);

	if (ret < 0) {
		err("sensor_3p8sp_set_registers fail!!");
		goto p_err;
	}

	dbg_sensor(1, "[%s] global setting done\n", __func__);

p_err:
	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_mode_change(struct v4l2_subdev *subdev, u32 mode)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	if (mode > sensor_2x5sp_max_setfile_num) {
		err("invalid mode(%d)!!", mode);
		return -EINVAL;
	}

	sensor_2x5sp_cis_data_calculation(sensor_2x5sp_pllinfos[mode], cis->cis_data);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_cis_set_registers(subdev, sensor_2x5sp_setfiles[mode], sensor_2x5sp_setfile_sizes[mode]);
	if (ret < 0) {
		err("sensor_2x5sp_set_registers fail!!");
		goto p_err;
	}

	cis->cis_data->frame_time = (cis->cis_data->line_readOut_time * cis->cis_data->cur_height / 1000);
	cis->cis_data->rolling_shutter_skew = (cis->cis_data->cur_height - 1) * cis->cis_data->line_readOut_time;
	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__,
		cis->cis_data->frame_time, cis->cis_data->rolling_shutter_skew);

	dbg_sensor(1, "[%s] mode changed(%d)\n", __func__, mode);

p_err:
	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_mode_change_throttling(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	sensor_2x5sp_cis_data_calculation(sensor_2x5sp_pllinfo_throttling, cis->cis_data);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_cis_set_registers(subdev, sensor_2x5sp_setfile_throttling,
				sizeof(sensor_2x5sp_setfile_throttling) / sizeof(sensor_2x5sp_setfile_throttling[0]));
	if (ret < 0) {
		err("sensor_gm1sp_set_registers fail!!");
		goto p_err;
	}

	cis->cis_data->frame_time = (cis->cis_data->line_readOut_time * cis->cis_data->cur_height / 1000);
	cis->cis_data->rolling_shutter_skew = (cis->cis_data->cur_height - 1) * cis->cis_data->line_readOut_time;
	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__,
		cis->cis_data->frame_time, cis->cis_data->rolling_shutter_skew);

	dbg_sensor(1, "[%s] throttling mode changed\n", __func__);

p_err:
	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

/* Deprecated */
int sensor_2x5sp_cis_set_size(struct v4l2_subdev *subdev, cis_shared_data *cis_data)
{
	return 0;
}

int sensor_2x5sp_cis_stream_on(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
	if (ret < 0)
		err("group_param_hold_func failed at stream on");

#ifdef DEBUG_2X5SP_PLL
	{
	u16 pll;

	fimc_is_sensor_read16(client, 0x0300, &pll);
	dbg_sensor(1, "______ vt_pix_clk_div(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x0302, &pll);
	dbg_sensor(1, "______ vt_sys_clk_div(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x0304, &pll);
	dbg_sensor(1, "______ pre_pll_clk_div(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x0306, &pll);
	dbg_sensor(1, "______ pll_multiplier(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x0308, &pll);
	dbg_sensor(1, "______ op_pix_clk_div(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x030a, &pll);
	dbg_sensor(1, "______ op_sys_clk_div(%x)\n", pll);

	fimc_is_sensor_read16(client, 0x030c, &pll);
	dbg_sensor(1, "______ secnd_pre_pll_clk_div(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x030e, &pll);
	dbg_sensor(1, "______ secnd_pll_multiplier(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x0340, &pll);
	dbg_sensor(1, "______ frame_length_lines(%x)\n", pll);
	fimc_is_sensor_read16(client, 0x0342, &pll);
	dbg_sensor(1, "______ line_length_pck(%x)\n", pll);
	}
#endif

	/* Sensor stream on */
	fimc_is_sensor_write16(client, 0x6028, 0x4000);
	fimc_is_sensor_write8(client, 0x0100, 0x01);

	cis_data->stream_on = true;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_stream_off(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
	if (ret < 0)
		err("group_param_hold_func failed at stream off");

	fimc_is_sensor_write16(client, 0x6028, 0x4000);

	/* during LEC mode, clear 0x0BCC before stream off for next frame
	 * After cancelled, restore value
	 */
	if (cis->long_term_mode.sen_strm_off_on_enable)
		fimc_is_sensor_write8(client, 0x0BCC, 0);
	else
		fimc_is_sensor_write8(client, 0x0BCC, 0x1);

	/* Sensor stream off */
	fimc_is_sensor_write8(client, 0x0100, 0x00);

	cis_data->stream_on = false;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_set_exposure_time(struct v4l2_subdev *subdev, struct ae_param *target_exposure)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 vt_pic_clk_freq_mhz = 0;
	u16 long_coarse_int = 0;
	u16 short_coarse_int = 0;
	u16 middle_coarse_int = 0;
	u32 line_length_pck = 0;
	u32 min_fine_int = 0;
	u64 numerator;
	u8 lte_shifter;
	u32 multiple_ratio = 1;
#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!target_exposure);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	if ((target_exposure->long_val <= 0) || (target_exposure->short_val <= 0)) {
		err("[%s] invalid target exposure(%d, %d)\n", __func__,
				target_exposure->long_val, target_exposure->short_val);
		return -EINVAL;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), target long(%d), short(%d), middle(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count,
			target_exposure->long_val, target_exposure->short_val, target_exposure->middle_val);

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;
	min_fine_int = cis_data->min_fine_integration_time;

	lte_shifter = cis->long_term_mode.sen_strm_off_on_enable ?
		GET_2X5SP_LTE_SHIFT_CNT(target_exposure->long_val) : 0;

	/* In 24M remosaic mode, set 4 times of exposure val */
	if (cis_data->sens_config_index_cur == SENSOR_2X5SP_5760X4320_24FPS) {
		multiple_ratio = 4;
		dbg_sensor(1, "[mod:d:%d] %s, Set 4 times of coarse_int for 24M mode\n", cis->id, __func__);
	}

	numerator = (u64)cis_data->pclk * target_exposure->long_val * multiple_ratio;
	long_coarse_int = (numerator - min_fine_int)
					/(1000 * 1000) / line_length_pck / (1 << lte_shifter);
	numerator = (u64)cis_data->pclk * target_exposure->short_val * multiple_ratio;
	short_coarse_int = (numerator - min_fine_int)
					/(1000 * 1000) / line_length_pck / (1 << lte_shifter);
	numerator = (u64)cis_data->pclk * target_exposure->middle_val * multiple_ratio;
	middle_coarse_int = (numerator - min_fine_int)
					/(1000 * 1000) / line_length_pck / (1 << lte_shifter);

	/* 4FDSUM mode should be set half of coarse integration */
	if (cis_data->sens_config_index_cur == SENSOR_2X5SP_2880X2160_30FPS
		|| cis_data->sens_config_index_cur == SENSOR_2X5SP_1920X1080_120FPS
		|| cis_data->sens_config_index_cur == SENSOR_2X5SP_1280X720_240FPS) {
		long_coarse_int /= 2;
		short_coarse_int /= 2;
		middle_coarse_int /= 2;
	}

	if (long_coarse_int > cis_data->max_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), long coarse(%d) max(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, long_coarse_int, cis_data->max_coarse_integration_time);
		long_coarse_int = cis_data->max_coarse_integration_time;
	}

	if (short_coarse_int > cis_data->max_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), short coarse(%d) max(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, short_coarse_int, cis_data->max_coarse_integration_time);
		short_coarse_int = cis_data->max_coarse_integration_time;
	}

	if (middle_coarse_int > cis_data->max_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), middle coarse(%d) max(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, middle_coarse_int, cis_data->max_coarse_integration_time);
		middle_coarse_int = cis_data->max_coarse_integration_time;
	}

	if (long_coarse_int < cis_data->min_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), long coarse(%d) min(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, long_coarse_int, cis_data->min_coarse_integration_time);
		long_coarse_int = cis_data->min_coarse_integration_time;
	}

	if (short_coarse_int < cis_data->min_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), short coarse(%d) min(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, short_coarse_int, cis_data->min_coarse_integration_time);
		short_coarse_int = cis_data->min_coarse_integration_time;
	}

	if (middle_coarse_int < cis_data->min_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), middle coarse(%d) min(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, middle_coarse_int, cis_data->min_coarse_integration_time);
		middle_coarse_int = cis_data->min_coarse_integration_time;
	}

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	/* Short exposure */
	ret = fimc_is_sensor_write16(client, 0x0202, short_coarse_int);
	if (ret < 0)
		goto p_err;

	/* Long exposure */
	if (cis_data->is_data.wdr_mode != CAMERA_WDR_OFF) {
		ret = fimc_is_sensor_write16(client, 0x0226, long_coarse_int);
		if (ret < 0)
			goto p_err;

		ret = fimc_is_sensor_write16(client, 0x022C, middle_coarse_int);
		if (ret < 0)
			goto p_err;
	}

	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), vt_pic_clk_freq_mhz (%d), line_length_pck(%d), min_fine_int (%d)\n",
		cis->id, __func__, cis_data->sen_vsync_count, vt_pic_clk_freq_mhz, line_length_pck, min_fine_int);
	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), frame_length_lines(%#x), coarse_int (L:%#x, S:%#x, M:%#x)\n",
		cis->id, __func__, cis_data->sen_vsync_count, cis_data->frame_length_lines,
		long_coarse_int, short_coarse_int, middle_coarse_int);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_get_min_exposure_time(struct v4l2_subdev *subdev, u32 *min_expo)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;
	cis_shared_data *cis_data = NULL;
	u32 min_integration_time = 0;
	u32 min_coarse = 0;
	u32 min_fine = 0;
	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!min_expo);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	cis_data = cis->cis_data;

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	if (vt_pic_clk_freq_mhz == 0) {
		pr_err("[MOD:D:%d] %s, Invalid vt_pic_clk_freq_mhz(%d)\n", cis->id, __func__, vt_pic_clk_freq_mhz);
		goto p_err;
	}
	line_length_pck = cis_data->line_length_pck;
	min_coarse = cis_data->min_coarse_integration_time;
	min_fine = cis_data->min_fine_integration_time;

	min_integration_time = ((line_length_pck * min_coarse) + min_fine) / vt_pic_clk_freq_mhz;
	*min_expo = min_integration_time;

	dbg_sensor(1, "[%s] min integration time %d\n", __func__, min_integration_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_2x5sp_cis_get_max_exposure_time(struct v4l2_subdev *subdev, u32 *max_expo)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;
	u32 max_integration_time = 0;
	u32 max_coarse_margin = 0;
	u32 max_fine_margin = 0;
	u32 max_coarse = 0;
	u32 max_fine = 0;
	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;
	u32 frame_length_lines = 0;
	u32 multiple_ratio = 1;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!max_expo);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	cis_data = cis->cis_data;

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	if (vt_pic_clk_freq_mhz == 0) {
		pr_err("[MOD:D:%d] %s, Invalid vt_pic_clk_freq_mhz(%d)\n", cis->id, __func__, vt_pic_clk_freq_mhz);
		goto p_err;
	}
	line_length_pck = cis_data->line_length_pck;
	frame_length_lines = cis_data->frame_length_lines;

	max_coarse_margin = cis_data->max_margin_coarse_integration_time;
	max_fine_margin = line_length_pck - cis_data->min_fine_integration_time;
	max_coarse = frame_length_lines - max_coarse_margin;
	max_fine = cis_data->max_fine_integration_time;

	max_integration_time = ((line_length_pck * max_coarse) + max_fine) / vt_pic_clk_freq_mhz;

	*max_expo = max_integration_time;

	/* In 24M remosaic mode, set 4 times of max_coarse_integration_time */
	if (cis_data->sens_config_index_cur == SENSOR_2X5SP_5760X4320_24FPS) {
		multiple_ratio = 4;
		dbg_sensor(1, "[mod:d:%d] %s, Set 4 times of coarse_int for 24M mode\n", cis->id, __func__);
	}

	/* TODO: Is this values update here? */
	cis_data->max_margin_fine_integration_time = max_fine_margin;
	cis_data->max_coarse_integration_time = max_coarse * multiple_ratio;

	dbg_sensor(1, "[%s] max integration time %d, max margin fine integration %d, max coarse integration %d\n",
			__func__, max_integration_time, cis_data->max_margin_fine_integration_time,
			cis_data->max_coarse_integration_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_2x5sp_cis_adjust_frame_duration(struct v4l2_subdev *subdev,
						u32 input_exposure_time,
						u32 *target_duration)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;
	u32 frame_length_lines = 0;
	u32 frame_duration = 0;
	u64 numerator;
	u8 lte_shifter;
	u32 multiple_ratio = 1;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!target_duration);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	cis_data = cis->cis_data;

	lte_shifter = cis->long_term_mode.sen_strm_off_on_enable ?
		GET_2X5SP_LTE_SHIFT_CNT(input_exposure_time) : 0;

	/* In 24M remosaic mode, set 4 times of frame_length_lines */
	if (cis_data->sens_config_index_cur == SENSOR_2X5SP_5760X4320_24FPS) {
		multiple_ratio = 4;
		dbg_sensor(1, "[mod:d:%d] %s, Set 4 times of coarse_int for 24M mode\n", cis->id, __func__);
	}

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;
	numerator = (u64)cis_data->pclk * input_exposure_time * multiple_ratio;
	frame_length_lines = (u16)((numerator / (1000 * 1000))/ line_length_pck / (1 << lte_shifter));
	frame_length_lines += cis_data->max_margin_coarse_integration_time;

	numerator = (u64)frame_length_lines * line_length_pck;
	frame_duration = (numerator << lte_shifter) / vt_pic_clk_freq_mhz;

	dbg_sensor(1, "[%s](vsync cnt = %d) input exp(%d), adj duration, frame duraion(%d), min_frame_us(%d)\n",
			__func__, cis_data->sen_vsync_count, input_exposure_time,
			frame_duration, cis_data->min_frame_us_time);
	dbg_sensor(1, "[%s](vsync cnt = %d) adj duration, frame duraion(%d), min_frame_us(%d)\n",
			__func__, cis_data->sen_vsync_count, frame_duration, cis_data->min_frame_us_time);

	*target_duration = MAX(frame_duration, cis_data->min_frame_us_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_2x5sp_cis_set_frame_duration(struct v4l2_subdev *subdev, u32 frame_duration)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 line_length_pck = 0;
	u16 frame_length_lines = 0;
	u64 numerator;
	u32 max_coarse_integration_time = 0;
	u8 lte_shifter;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;

	if (frame_duration < cis_data->min_frame_us_time) {
		dbg_sensor(1, "frame duration is less than min(%d)\n", frame_duration);
		frame_duration = cis_data->min_frame_us_time;
	}

	lte_shifter = cis->long_term_mode.sen_strm_off_on_enable ?
		GET_2X5SP_LTE_SHIFT_CNT(frame_duration) : 0;

	line_length_pck = cis_data->line_length_pck;
	numerator = (u64)cis_data->pclk * frame_duration;
	frame_length_lines = (u16)((numerator / line_length_pck) / (1000 * 1000) / (1 << lte_shifter));

	dbg_sensor(1, "[MOD:D:%d] %s, vt_pic_clk(%#x) frame_duration = %d us,"
		KERN_CONT "(line_length_pck%#x), frame_length_lines(%#x)\n",
		cis->id, __func__, cis_data->pclk, frame_duration, line_length_pck, frame_length_lines);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_write16(client, 0x0340, frame_length_lines);
	if (ret < 0)
		goto p_err;

	cis_data->cur_frame_us_time = frame_duration;
	cis_data->frame_length_lines = frame_length_lines;

	max_coarse_integration_time = cis_data->frame_length_lines - cis_data->max_margin_coarse_integration_time;
	cis_data->max_coarse_integration_time = max_coarse_integration_time;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_set_frame_rate(struct v4l2_subdev *subdev, u32 min_fps)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u32 frame_duration = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	cis_data = cis->cis_data;

	if (min_fps > cis_data->max_fps) {
		err("[MOD:D:%d] %s, request FPS is too high(%d), set to max(%d)\n",
			cis->id, __func__, min_fps, cis_data->max_fps);
		min_fps = cis_data->max_fps;
	}

	if (min_fps == 0) {
		err("[MOD:D:%d] %s, request FPS is 0, set to min FPS(1)\n",
			cis->id, __func__);
		min_fps = 1;
	}

	frame_duration = (1 * 1000 * 1000) / min_fps;

	dbg_sensor(1, "[MOD:D:%d] %s, set FPS(%d), frame duration(%d)\n",
			cis->id, __func__, min_fps, frame_duration);

	ret = sensor_2x5sp_cis_set_frame_duration(subdev, frame_duration);
	if (ret < 0) {
		err("[MOD:D:%d] %s, set frame duration is fail(%d)\n",
			cis->id, __func__, ret);
		goto p_err;
	}

	cis_data->min_frame_us_time = frame_duration;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:

	return ret;
}

int sensor_2x5sp_cis_adjust_analog_gain(struct v4l2_subdev *subdev, u32 input_again, u32 *target_permile)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u32 again_code = 0;
	u32 again_permile = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!target_permile);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	cis_data = cis->cis_data;

	again_code = sensor_cis_calc_again_code(input_again);

	if (again_code > cis_data->max_analog_gain[0])
		again_code = cis_data->max_analog_gain[0];
	else if (again_code < cis_data->min_analog_gain[0])
		again_code = cis_data->min_analog_gain[0];

	again_permile = sensor_cis_calc_again_permile(again_code);

	dbg_sensor(1, "[%s] min again(%d), max(%d), input_again(%d), code(%d), permile(%d)\n", __func__,
			cis_data->max_analog_gain[0],
			cis_data->min_analog_gain[0],
			input_again,
			again_code,
			again_permile);

	*target_permile = again_permile;

	return ret;
}

int sensor_2x5sp_cis_set_analog_gain(struct v4l2_subdev *subdev, struct ae_param *again)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	u16 analog_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	analog_gain = (u16)sensor_cis_calc_again_code(again->val);

	if (analog_gain < cis->cis_data->min_analog_gain[0])
		analog_gain = cis->cis_data->min_analog_gain[0];

	if (analog_gain > cis->cis_data->max_analog_gain[0])
		analog_gain = cis->cis_data->max_analog_gain[0];

	dbg_sensor(1, "[MOD:D:%d] %s(vsync cnt = %d), input_again = %d us, analog_gain(%#x)\n",
		cis->id, __func__, cis->cis_data->sen_vsync_count, again->long_val, analog_gain);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_write16(client, 0x0204, analog_gain);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_get_analog_gain(struct v4l2_subdev *subdev, u32 *again)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	u16 analog_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_read16(client, 0x0204, &analog_gain);
	if (ret < 0)
		goto p_err;

	*again = sensor_cis_calc_again_permile(analog_gain);

	dbg_sensor(1, "[MOD:D:%d] %s, cur_again = %d us, analog_gain(%#x)\n",
			cis->id, __func__, *again, analog_gain);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_get_min_analog_gain(struct v4l2_subdev *subdev, u32 *min_again)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!min_again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;
	cis_data->min_analog_gain[0] = 0x20;
	cis_data->min_analog_gain[1] = sensor_cis_calc_again_permile(cis_data->min_analog_gain[0]);

	*min_again = cis_data->min_analog_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__, cis_data->min_analog_gain[0], cis_data->min_analog_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_2x5sp_cis_get_max_analog_gain(struct v4l2_subdev *subdev, u32 *max_again)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!max_again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;
	cis_data->max_analog_gain[0] = 0x200;
	cis_data->max_analog_gain[1] = sensor_cis_calc_again_permile(cis_data->max_analog_gain[0]);

	*max_again = cis_data->max_analog_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__, cis_data->max_analog_gain[0], cis_data->max_analog_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_2x5sp_cis_set_digital_gain(struct v4l2_subdev *subdev, struct ae_param *dgain)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 long_gain = 0;
	u16 short_gain = 0;
	u16 middle_gain = 0;
	u16 dgains[4] = {0};

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;

	long_gain = (u16)sensor_cis_calc_dgain_code(dgain->long_val);
	short_gain = (u16)sensor_cis_calc_dgain_code(dgain->short_val);
	middle_gain = (u16)sensor_cis_calc_dgain_code(dgain->middle_val);

	if (long_gain < cis->cis_data->min_digital_gain[0])
		long_gain = cis->cis_data->min_digital_gain[0];

	if (long_gain > cis->cis_data->max_digital_gain[0])
		long_gain = cis->cis_data->max_digital_gain[0];

	if (short_gain < cis->cis_data->min_digital_gain[0])
		short_gain = cis->cis_data->min_digital_gain[0];

	if (short_gain > cis->cis_data->max_digital_gain[0])
		short_gain = cis->cis_data->max_digital_gain[0];

	if (middle_gain < cis->cis_data->min_digital_gain[0])
		middle_gain = cis->cis_data->min_digital_gain[0];

	if (middle_gain > cis->cis_data->max_digital_gain[0])
		middle_gain = cis->cis_data->max_digital_gain[0];

	dbg_sensor(1, "[MOD:D:%d] %s(vsync cnt = %d), input_dgain = %d/%d/%d us, gain(L:%#x, S:%#x, M:%#x)\n",
		cis->id, __func__, cis->cis_data->sen_vsync_count, dgain->long_val,
		dgain->short_val, dgain->middle_val, long_gain, short_gain, middle_gain);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	dgains[0] = dgains[1] = dgains[2] = dgains[3] = short_gain;
	/* Short digital gain */
	ret = fimc_is_sensor_write16_array(client, 0x020E, dgains, 4);
	if (ret < 0)
		goto p_err;

	/* Long & medium digital gain */
	if (cis_data->is_data.wdr_mode != CAMERA_WDR_OFF) {
		dgains[0] = dgains[1] = dgains[2] = dgains[3] = long_gain;
		/* long digital gain */
		ret = fimc_is_sensor_write16_array(client, 0x0230, dgains, 4);
		if (ret < 0)
			goto p_err;
		dgains[0] = dgains[1] = dgains[2] = dgains[3] = middle_gain;
		/* middle digital gain */
		ret = fimc_is_sensor_write16_array(client, 0x0238, dgains, 4);
		if (ret < 0)
			goto p_err;
	}

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_get_digital_gain(struct v4l2_subdev *subdev, u32 *dgain)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	u16 digital_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_read16(client, 0x020E, &digital_gain);
	if (ret < 0)
		goto p_err;

	*dgain = sensor_cis_calc_dgain_permile(digital_gain);

	dbg_sensor(1, "[MOD:D:%d] %s, cur_dgain = %d us, digital_gain(%#x)\n",
			cis->id, __func__, *dgain, digital_gain);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_get_min_digital_gain(struct v4l2_subdev *subdev, u32 *min_dgain)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!min_dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;
	cis_data->min_digital_gain[0] = 0x100;
	cis_data->min_digital_gain[1] = sensor_cis_calc_dgain_permile(cis_data->min_digital_gain[0]);

	*min_dgain = cis_data->min_digital_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__,
		cis_data->min_digital_gain[0], cis_data->min_digital_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_2x5sp_cis_get_max_digital_gain(struct v4l2_subdev *subdev, u32 *max_dgain)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!max_dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	cis_data = cis->cis_data;
	cis_data->max_digital_gain[0] = 0x8000;
	cis_data->max_digital_gain[1] = sensor_cis_calc_dgain_permile(cis_data->max_digital_gain[0]);

	*max_dgain = cis_data->max_digital_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__,
		cis_data->max_digital_gain[0], cis_data->max_digital_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_2x5sp_cis_set_wb_gain(struct v4l2_subdev *subdev, struct wb_gains wb_gains)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	u16 abs_gains[4] = {0, }; /* [0]=gr, [1]=r, [2]=b, [3]=gb */

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	if (!cis->use_wb_gain)
		return ret;

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	dbg_sensor(1, "[SEN:%d]%s:DDK vlaue: wb_gain_gr(%d), wb_gain_r(%d), wb_gain_b(%d)\n",
			cis->id, __func__, wb_gains.gr, wb_gains.r, wb_gains.b, wb_gains.gb);

	if (wb_gains.gr == 0 || wb_gains.r == 0 || wb_gains.b == 0 || wb_gains.gb == 0)
		return ret;

	abs_gains[0] = (u16)((wb_gains.r / 4) & 0xFFFF);
	abs_gains[1] = (u16)((wb_gains.gr / 4) & 0xFFFF);
	abs_gains[2] = (u16)((wb_gains.b / 4) & 0xFFFF);

	dbg_sensor(1, "[SEN:%d]%s, abs_gain_r(0x%4X), abs_gain_gr(0x%4X), abs_gain_b(0x%4X)\n",
			cis->id, __func__, abs_gains[0], abs_gains[1], abs_gains[2]);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	/* 0x40000D12 ~ 0x400000D16: api_rw_color_temperature_absolute_gain_red/green/blue */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x4000);
	ret |= fimc_is_sensor_write16_array(client, 0x0D12, abs_gains, 3);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_set_3hdr_roi(struct v4l2_subdev *subdev, struct roi_setting_t roi_control)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	u16 roi_val[4];
#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	if (!cis->use_3hdr)
		return ret;

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	dbg_sensor(1, "%s: [MOD:%d] roi_control (start_x:%d, start_y:%d, end_x:%d, end_y:%d)\n",
		__func__, cis->id,
		roi_control.roi_start_x, roi_control.roi_start_y,
		roi_control.roi_end_x, roi_control.roi_end_y);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	/* 0x2000_XXXX */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x2000);
	if (ret < 0)
		goto p_err;

	/* t_isp_rgby_hist_mem_cfg_active_window_percent_14bit */
	roi_val[0] = roi_control.roi_start_x; /* 0x20004B26: top_left_x */
	roi_val[1] = roi_control.roi_start_y; /* 0x20004B28: top_left_y */
	roi_val[2] = roi_control.roi_end_x; /* 0x20004B2A: bot_right_x */
	roi_val[3] = roi_control.roi_end_y; /* 0x20004B2C: bot_right_y */

	ret = fimc_is_sensor_write16_array(client, 0x4B26, roi_val, 4);
	if (ret < 0)
		goto p_err;

	/* t_isp_rgby_hist_grid_grid & thstat_grid_area */
	roi_val[0] = roi_control.roi_end_x - roi_control.roi_start_x; /* 0x20004BEE & 0x20004E44: width */
	roi_val[1] = roi_control.roi_end_y - roi_control.roi_start_y; /* 0x20004BF0 & 0x20004E46: height */
	roi_val[2] = (roi_control.roi_start_x + roi_control.roi_end_x) / 2; /* center_x */
	roi_val[3] = (roi_control.roi_start_y + roi_control.roi_end_y) / 2; /* center_y */

	ret = fimc_is_sensor_write16_array(client, 0x4BEE, roi_val, 4);
	if (ret < 0)
		goto p_err;

	ret = fimc_is_sensor_write16_array(client, 0x4E44, roi_val, 2);
	if (ret < 0)
		goto p_err;

	/* restore 0x4000_XXXX */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x4000);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_2x5sp_cis_set_3hdr_stat(struct v4l2_subdev *subdev, bool streaming, void *data)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	u16 weight[3];
	u16 low_gate_thr, high_gate_thr;
	struct roi_setting_t y_sum_roi;
	struct sensor_lsi_3hdr_stat_control_mode_change mode_change_stat;
	struct sensor_lsi_3hdr_stat_control_per_frame per_frame_stat;
#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!data);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	if (!cis->use_3hdr)
		return ret;

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		return -EINVAL;
	}

	I2C_MUTEX_LOCK(cis->i2c_lock);

	hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	if (streaming) {
		per_frame_stat = *(struct sensor_lsi_3hdr_stat_control_per_frame *)data;

		weight[0] = per_frame_stat.r_weight;
		weight[1] = per_frame_stat.g_weight;
		weight[2] = per_frame_stat.b_weight;
	} else {
		mode_change_stat = *(struct sensor_lsi_3hdr_stat_control_mode_change *)data;

		weight[0] = mode_change_stat.r_weight;
		weight[1] = mode_change_stat.g_weight;
		weight[2] = mode_change_stat.b_weight;

		low_gate_thr = mode_change_stat.low_gate_thr;
		high_gate_thr = mode_change_stat.high_gate_thr;

		y_sum_roi = mode_change_stat.y_sum_roi;
	}

	ret = fimc_is_sensor_write16(client, 0x6028, 0x2000);
	if (ret < 0)
		goto p_err;

	/* t_isp_rgby_hist_short_exp_weight */
	ret = fimc_is_sensor_write16_array(client, 0x4BBC, weight, 3);
	if (ret < 0)
		goto p_err;

	/* t_isp_rgby_hist_long_exp_weight */
	ret = fimc_is_sensor_write16_array(client, 0x4BCA, weight, 3);
	if (ret < 0)
		goto p_err;

	/* t_isp_rgby_hist_medium_exp_weight */
	ret = fimc_is_sensor_write16_array(client, 0x4BD8, weight, 3);
	if (ret < 0)
		goto p_err;

	/* t_isp_rgby_hist_mixed_exp_weight */
	ret = fimc_is_sensor_write16_array(client, 0x4BE6, weight, 3);
	if (ret < 0)
		goto p_err;

	/* t_isp_drc_thstat_rgb_weights */
	ret = fimc_is_sensor_write16_array(client, 0x4E2C, weight, 3);
	if (ret < 0)
		goto p_err;

	if (!streaming) {
		/* t_isp_drc_thstat_u_low_tresh_red */
		ret = fimc_is_sensor_write16(client, 0x4E1A, low_gate_thr);
		if (ret < 0)
			goto p_err;
		/* t_isp_drc_thstat_u_high_tresh_red */
		ret = fimc_is_sensor_write16(client, 0x4E1C, high_gate_thr);
		if (ret < 0)
			goto p_err;

		/* t_isp_drc_thstat_u_low_tresh_green */
		ret = fimc_is_sensor_write16(client, 0x4E1E, low_gate_thr);
		if (ret < 0)
			goto p_err;

		/* t_isp_drc_thstat_u_high_tresh_green */
		ret = fimc_is_sensor_write16(client, 0x4E20, high_gate_thr);
		if (ret < 0)
			goto p_err;

		/* t_isp_drc_thstat_u_low_tresh_blue */
		ret = fimc_is_sensor_write16(client, 0x4E22, low_gate_thr);
		if (ret < 0)
			goto p_err;

		/* t_isp_drc_thstat_u_high_tresh_blue */
		ret = fimc_is_sensor_write16(client, 0x4E24, high_gate_thr);
		if (ret < 0)
			goto p_err;

		/* t_isp_y_sum_top_left_x */
		ret = fimc_is_sensor_write16(client, 0x4E04, y_sum_roi.roi_start_x);
		if (ret < 0)
			goto p_err;
		ret = fimc_is_sensor_write16(client, 0x4E06, y_sum_roi.roi_start_y);
		if (ret < 0)
			goto p_err;
	} else {
		/* update 3hdr motion stat */
		ret = fimc_is_sensor_write16(client, 0x6028, 0x2001);
		if (ret < 0)
			goto p_err;

		ret |= fimc_is_sensor_write16(client, 0x602A, 0x29D8);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_indication);

		ret |= fimc_is_sensor_write16(client, 0x602A, 0x2A52);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_high_end_ty2ty1);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_high_end_ty3ty2);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_high_start_ty2ty1);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_high_start_ty3ty2);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_low_end_ty2ty1);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_low_end_ty3ty2);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_low_start_ty2ty1);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_low_start_ty3ty2);

		dbg_sensor(2, "[%s] motion idc(%d) high21(e:%d, s:%d), low21(e:%d, s:%d)\n",
				__func__,
				(u16)per_frame_stat.motion_indication,
				(u16)per_frame_stat.motion_high_end_ty2ty1,
				(u16)per_frame_stat.motion_high_start_ty2ty1,
				(u16)per_frame_stat.motion_low_end_ty2ty1,
				(u16)per_frame_stat.motion_low_start_ty2ty1);
		dbg_sensor(2, "[%s] motion high32(e:%d, s:%d), low32(e:%d, s:%d)\n",
				__func__,
				(u16)per_frame_stat.motion_high_end_ty3ty2,
				(u16)per_frame_stat.motion_high_start_ty3ty2,
				(u16)per_frame_stat.motion_low_end_ty3ty2,
				(u16)per_frame_stat.motion_low_start_ty3ty2);

		ret |= fimc_is_sensor_write16(client, 0x602A, 0x2A68);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.decision_thresh_override);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_abs_high_ty3ty2);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_abs_low_ty3ty2);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_abs_high_ty2ty1);
		ret |= fimc_is_sensor_write16(client, 0x6F12,
				(u16)per_frame_stat.motion_abs_low_ty2ty1);

		dbg_sensor(2, "[%s] motion DTO(%d), abs(h32:%d, l32:%d), abs(h21:%d, l21:%d)\n",
				__func__,
				(u16)per_frame_stat.decision_thresh_override,
				(u16)per_frame_stat.motion_abs_high_ty3ty2,
				(u16)per_frame_stat.motion_abs_low_ty3ty2,
				(u16)per_frame_stat.motion_abs_high_ty2ty1,
				(u16)per_frame_stat.motion_abs_low_ty2ty1);
	}

	/* restore 0x4000_XXXX */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x4000);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_2x5sp_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

void sensor_2x5sp_cis_check_wdr_mode(struct v4l2_subdev *subdev, u32 mode_idx)
{
	struct fimc_is_cis *cis;

	FIMC_BUG_VOID(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG_VOID(!cis);
	FIMC_BUG_VOID(!cis->cis_data);

	/* check wdr mode */
	if (mode_idx == SENSOR_2X5SP_2880X2160_30FPS_3DHDR)
		cis->cis_data->is_data.wdr_enable = true;
	else
		cis->cis_data->is_data.wdr_enable = false;

	dbg_sensor(1, "[%s] wdr_enable: %d\n", __func__,
				cis->cis_data->is_data.wdr_enable);
}

int sensor_2x5sp_cis_long_term_exposure(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct fimc_is_long_term_expo_mode *lte_mode;
	u8 shift_count;

	WARN_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	lte_mode = &cis->long_term_mode;

	I2C_MUTEX_LOCK(cis->i2c_lock);
	/* LTE mode or normal mode set */
	if (lte_mode->sen_strm_off_on_enable) {
		shift_count = GET_2X5SP_LTE_SHIFT_CNT(lte_mode->expo[0]);
		ret |= fimc_is_sensor_write16(cis->client, 0xFCFC, 0x4000);
		ret |= fimc_is_sensor_write8(cis->client, 0x0702, shift_count);
		ret |= fimc_is_sensor_write8(cis->client, 0x0704, shift_count);
	} else {
		ret |= fimc_is_sensor_write16(cis->client, 0xFCFC, 0x4000);
		ret |= fimc_is_sensor_write8(cis->client, 0x0702, 0);
		ret |= fimc_is_sensor_write8(cis->client, 0x0704, 0);
	}

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	info("%s enable(%d)", __func__, lte_mode->sen_strm_off_on_enable);

	if (ret < 0) {
		pr_err("ERR[%s]: LTE register setting fail\n", __func__);
		return ret;
	}

	return ret;
}

static struct fimc_is_cis_ops cis_ops = {
	.cis_init = sensor_2x5sp_cis_init,
	.cis_log_status = sensor_2x5sp_cis_log_status,
	.cis_group_param_hold = sensor_2x5sp_cis_group_param_hold,
	.cis_set_global_setting = sensor_2x5sp_cis_set_global_setting,
	.cis_mode_change = sensor_2x5sp_cis_mode_change,
	.cis_set_size = sensor_2x5sp_cis_set_size,
	.cis_stream_on = sensor_2x5sp_cis_stream_on,
	.cis_stream_off = sensor_2x5sp_cis_stream_off,
	.cis_set_exposure_time = sensor_2x5sp_cis_set_exposure_time,
	.cis_get_min_exposure_time = sensor_2x5sp_cis_get_min_exposure_time,
	.cis_get_max_exposure_time = sensor_2x5sp_cis_get_max_exposure_time,
	.cis_adjust_frame_duration = sensor_2x5sp_cis_adjust_frame_duration,
	.cis_set_frame_duration = sensor_2x5sp_cis_set_frame_duration,
	.cis_set_frame_rate = sensor_2x5sp_cis_set_frame_rate,
	.cis_adjust_analog_gain = sensor_2x5sp_cis_adjust_analog_gain,
	.cis_set_analog_gain = sensor_2x5sp_cis_set_analog_gain,
	.cis_get_analog_gain = sensor_2x5sp_cis_get_analog_gain,
	.cis_get_min_analog_gain = sensor_2x5sp_cis_get_min_analog_gain,
	.cis_get_max_analog_gain = sensor_2x5sp_cis_get_max_analog_gain,
	.cis_set_digital_gain = sensor_2x5sp_cis_set_digital_gain,
	.cis_get_digital_gain = sensor_2x5sp_cis_get_digital_gain,
	.cis_get_min_digital_gain = sensor_2x5sp_cis_get_min_digital_gain,
	.cis_get_max_digital_gain = sensor_2x5sp_cis_get_max_digital_gain,
	.cis_compensate_gain_for_extremely_br = sensor_cis_compensate_gain_for_extremely_br,
	.cis_wait_streamoff = sensor_cis_wait_streamoff,
	.cis_wait_streamon = sensor_cis_wait_streamon,
	.cis_set_initial_exposure = sensor_cis_set_initial_exposure,
	.cis_check_rev = sensor_2x5sp_cis_check_rev,
	.cis_factory_test = sensor_cis_factory_test,
	.cis_set_wb_gains = sensor_2x5sp_cis_set_wb_gain,
	.cis_set_roi_stat = sensor_2x5sp_cis_set_3hdr_roi,
	.cis_set_3hdr_stat = sensor_2x5sp_cis_set_3hdr_stat,
	.cis_check_wdr_mode = sensor_2x5sp_cis_check_wdr_mode,
	.cis_set_long_term_exposure = sensor_2x5sp_cis_long_term_exposure,
	.cis_mode_change_throttling = sensor_2x5sp_cis_mode_change_throttling,
};

static int cis_2x5sp_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0, i;
	struct fimc_is_core *core = NULL;
	struct v4l2_subdev *subdev_cis = NULL;
	struct fimc_is_cis *cis = NULL;
	struct fimc_is_device_sensor *device = NULL;
	struct fimc_is_device_sensor_peri *sensor_peri = NULL;
	u32 sensor_id = 0;
	char const *setfile;
	struct device *dev;
	struct device_node *dnode;

	FIMC_BUG(!client);
	FIMC_BUG(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &client->dev;
	dnode = dev->of_node;

	ret = of_property_read_u32(dnode, "id", &sensor_id);
	if (ret) {
		err("sensor id read is fail(%d)", ret);
		goto p_err;
	}

	probe_info("%s sensor id %d\n", __func__, sensor_id);

	device = &core->sensor[sensor_id];

	sensor_peri = find_peri_by_cis_id(device, SENSOR_NAME_S5K2X5SP);
	if (!sensor_peri) {
		probe_info("sensor peri is net yet probed");
		return -EPROBE_DEFER;
	}

	cis = &sensor_peri->cis;
	if (!cis) {
		err("cis is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_cis = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_cis) {
		err("subdev_cis NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	sensor_peri->subdev_cis = subdev_cis;

	cis->id = SENSOR_NAME_S5K2X5SP;
	cis->subdev = subdev_cis;
	cis->device = 0;
	cis->client = client;
	sensor_peri->module->client = cis->client;
	cis->ctrl_delay = N_PLUS_TWO_FRAME;

	cis->cis_data = kzalloc(sizeof(cis_shared_data), GFP_KERNEL);
	if (!cis->cis_data) {
		err("cis_data is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	cis->cis_ops = &cis_ops;

	/* belows are depend on sensor cis. MUST check sensor spec */
	cis->bayer_order = OTF_INPUT_ORDER_BAYER_GR_BG;

	if (of_property_read_bool(dnode, "sensor_f_number")) {
		ret = of_property_read_u32(dnode, "sensor_f_number", &cis->aperture_num);
		if (ret)
			warn("f-number read is fail(%d)", ret);
	} else {
		cis->aperture_num = F2_2;
	}

	probe_info("%s f-number %d\n", __func__, cis->aperture_num);

	cis->use_dgain = true;
	cis->hdr_ctrl_by_again = false;
	cis->use_wb_gain = true;
	cis->use_3hdr = true;

	ret = of_property_read_string(dnode, "setfile", &setfile);
	if (ret) {
		err("setfile index read fail(%d), take default setfile!!", ret);
		setfile = "default";
	}

	if (strcmp(setfile, "default") == 0 ||
			strcmp(setfile, "setA") == 0) {
		probe_info("%s setfile_A\n", __func__);
		sensor_2x5sp_global = sensor_2x5sp_setfile_A_Global;
		sensor_2x5sp_global_size = ARRAY_SIZE(sensor_2x5sp_setfile_A_Global);
		sensor_2x5sp_setfiles = sensor_2x5sp_setfiles_A;
		sensor_2x5sp_setfile_sizes = sensor_2x5sp_setfile_A_sizes;
		sensor_2x5sp_otp_initial = sensor_2x5sp_setfiles_A_otp_initial;
		sensor_2x5sp_otp_initial_size = ARRAY_SIZE(sensor_2x5sp_setfiles_A_otp_initial);
		sensor_2x5sp_pllinfos = sensor_2x5sp_pllinfos_A;
		sensor_2x5sp_max_setfile_num = ARRAY_SIZE(sensor_2x5sp_setfiles_A);
	} else if (strcmp(setfile, "setB") == 0) {
		probe_info("%s setfile_B\n", __func__);
		sensor_2x5sp_global = sensor_2x5sp_setfile_B_Global;
		sensor_2x5sp_global_size = ARRAY_SIZE(sensor_2x5sp_setfile_B_Global);
		sensor_2x5sp_setfiles = sensor_2x5sp_setfiles_B;
		sensor_2x5sp_setfile_sizes = sensor_2x5sp_setfile_B_sizes;
		sensor_2x5sp_otp_initial = sensor_2x5sp_setfiles_A_otp_initial;
		sensor_2x5sp_otp_initial_size = ARRAY_SIZE(sensor_2x5sp_setfiles_A_otp_initial);
		sensor_2x5sp_pllinfos = sensor_2x5sp_pllinfos_B;
		sensor_2x5sp_max_setfile_num = ARRAY_SIZE(sensor_2x5sp_setfiles_B);

		/* throttling setting */
		sensor_2x5sp_setfile_throttling = sensor_2x5sp_setfile_B_2880x2160_15fps;
		sensor_2x5sp_pllinfo_throttling = &sensor_2x5sp_pllinfo_B_2880x2160_15fps;
	} else {
		err("%s setfile index out of bound, take default (setfile_A)", __func__);
		sensor_2x5sp_global = sensor_2x5sp_setfile_A_Global;
		sensor_2x5sp_global_size = ARRAY_SIZE(sensor_2x5sp_setfile_A_Global);
		sensor_2x5sp_setfiles = sensor_2x5sp_setfiles_A;
		sensor_2x5sp_setfile_sizes = sensor_2x5sp_setfile_A_sizes;
		sensor_2x5sp_pllinfos = sensor_2x5sp_pllinfos_A;
		sensor_2x5sp_max_setfile_num = ARRAY_SIZE(sensor_2x5sp_setfiles_A);
	}

	cis->use_initial_ae = of_property_read_bool(dnode, "use_initial_ae");
	probe_info("%s use initial_ae(%d)\n", __func__, cis->use_initial_ae);

	device->use_otp_cal = of_property_read_bool(dnode, "use_otp_cal");
	probe_info("%s use otp_cal(%d)\n", __func__, device->use_otp_cal);

	for (i = 0; i < CAMERA_CRC_INDEX_MAX; i++)
		device->cal_status[i] = CRC_NO_ERROR;

	v4l2_i2c_subdev_init(subdev_cis, client, &subdev_ops);
	v4l2_set_subdevdata(subdev_cis, cis);
	v4l2_set_subdev_hostdata(subdev_cis, device);
	snprintf(subdev_cis->name, V4L2_SUBDEV_NAME_SIZE, "cis-subdev.%d", cis->id);

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static const struct of_device_id sensor_cis_2x5sp_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-cis-2x5sp",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_cis_2x5sp_match);

static const struct i2c_device_id sensor_cis_2x5sp_idt[] = {
	{ SENSOR_NAME, 0 },
	{},
};

static struct i2c_driver sensor_cis_2x5sp_driver = {
	.probe	= cis_2x5sp_probe,
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_cis_2x5sp_match,
		.suppress_bind_attrs = true,
	},
	.id_table = sensor_cis_2x5sp_idt
};

static int __init sensor_cis_2x5sp_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_cis_2x5sp_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_cis_2x5sp_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_cis_2x5sp_init);
