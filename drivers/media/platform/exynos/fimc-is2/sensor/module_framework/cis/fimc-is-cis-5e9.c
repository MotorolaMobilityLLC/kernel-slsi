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
#include "fimc-is-cis-5e9.h"
#include "fimc-is-cis-5e9-setA.h"
#include "fimc-is-cis-5e9-setB.h"
#include "fimc-is-cis-5e9-setC.h"

#include "fimc-is-helper-i2c.h"
#include "fimc-is-vender-specific.h"

#define SENSOR_NAME "S5K5E9"

static const struct v4l2_subdev_ops subdev_ops;

static const u32 *sensor_5e9_global;
static u32 sensor_5e9_global_size;
static const u32 **sensor_5e9_setfiles;
static const u32 *sensor_5e9_setfile_sizes;
static const struct sensor_pll_info **sensor_5e9_pllinfos;
static u32 sensor_5e9_max_setfile_num;

static const u32 *sensor_5e9_setfile_throttling;
static const struct sensor_pll_info *sensor_5e9_pllinfo_throttling;

static void sensor_5e9_cis_data_calculation(const struct sensor_pll_info *pll_info, cis_shared_data *cis_data)
{
	u32 pll_voc_a = 0, vt_pix_clk_hz = 0;
	u32 frame_rate = 0, max_fps = 0, frame_valid_us = 0;

	FIMC_BUG_VOID(!pll_info);

	/* 1. mipi data rate calculation (Mbps/Lane) */
	/* ToDo: using output Pixel Clock Divider Value */
	/* pll_voc_b = pll_info->ext_clk / pll_info->secnd_pre_pll_clk_div * pll_info->secnd_pll_multiplier * 2;
	op_sys_clk_hz = pll_voc_b / pll_info->op_sys_clk_div;
	if(gpsSensorExInfo) {
		gpsSensorExInfo->uiMIPISpeedBps = op_sys_clk_hz;
		gpsSensorExInfo->uiMCLK = sensorInfo.ext_clk;
	} */

	/* 2. pixel rate calculation (Mpps) */
	pll_voc_a = pll_info->ext_clk / pll_info->pre_pll_clk_div * pll_info->pll_multiplier;
	vt_pix_clk_hz = pll_voc_a * 2 / (pll_info->vt_sys_clk_div * pll_info->vt_pix_clk_div);

	dbg_sensor(1, "ext_clock(%d) / pre_pll_clk_div(%d) * pll_multiplier(%d) = pll_voc_a(%d)\n",
						pll_info->ext_clk, pll_info->pre_pll_clk_div,
						pll_info->pll_multiplier, pll_voc_a);
	dbg_sensor(1, "pll_voc_a(%d) / (vt_sys_clk_div(%d) * vt_pix_clk_div(%d)) = pixel clock (%d hz)\n",
						pll_voc_a, pll_info->vt_sys_clk_div,
						pll_info->vt_pix_clk_div, vt_pix_clk_hz);

	/* 3. the time of processing one frame calculation (us) */
	cis_data->min_frame_us_time = (pll_info->frame_length_lines * pll_info->line_length_pck
					/ (vt_pix_clk_hz / (1000 * 1000)));
	cis_data->cur_frame_us_time = cis_data->min_frame_us_time;

	/* 4. FPS calculation */
	frame_rate = vt_pix_clk_hz / (pll_info->frame_length_lines * pll_info->line_length_pck);

	/* calculate max fps */
	max_fps = (vt_pix_clk_hz * 10) / (pll_info->frame_length_lines * pll_info->line_length_pck);
	max_fps = (max_fps % 10 >= 5 ? frame_rate + 1 : frame_rate);

	cis_data->pclk = vt_pix_clk_hz;
	cis_data->max_fps = max_fps;
	cis_data->frame_length_lines = pll_info->frame_length_lines;
	cis_data->line_length_pck = pll_info->line_length_pck;
	cis_data->line_readOut_time = sensor_cis_do_div64((u64)cis_data->line_length_pck *
		(u64)(1000 * 1000 * 1000), cis_data->pclk);
	cis_data->rolling_shutter_skew = (cis_data->cur_height - 1) * cis_data->line_readOut_time;
	cis_data->stream_on = false;

	/* Frame valid time calcuration */
	frame_valid_us = sensor_cis_do_div64((u64)cis_data->cur_height * (u64)cis_data->line_length_pck *
		(u64)(1000 * 1000), cis_data->pclk);
	cis_data->frame_valid_us_time = (int)frame_valid_us;

	dbg_sensor(1, "%s\n", __func__);
	dbg_sensor(1, "Sensor size(%d x %d) setting: SUCCESS!\n",
					cis_data->cur_width, cis_data->cur_height);
	dbg_sensor(1, "Frame Valid(us): %d\n", frame_valid_us);
	dbg_sensor(1, "rolling_shutter_skew: %lld\n", cis_data->rolling_shutter_skew);

	dbg_sensor(1, "Fps: %d, max fps(%d)\n", frame_rate, cis_data->max_fps);
	dbg_sensor(1, "min_frame_time(%d us)\n", cis_data->min_frame_us_time);
	dbg_sensor(1, "Pixel rate(Mbps): %d\n", cis_data->pclk / 1000000);
	/* dbg_sensor(1, "Mbps/lane : %d Mbps\n", pll_voc_b / pll_info->op_sys_clk_div / 1000 / 1000); */

	/* Constant values */
	cis_data->min_fine_integration_time = SENSOR_5E9_FINE_INTEGRATION_TIME_MIN;
	cis_data->max_fine_integration_time = SENSOR_5E9_FINE_INTEGRATION_TIME_MAX;
	cis_data->min_coarse_integration_time = SENSOR_5E9_COARSE_INTEGRATION_TIME_MIN;
	cis_data->max_margin_coarse_integration_time = SENSOR_5E9_COARSE_INTEGRATION_TIME_MAX_MARGIN;
}

static int sensor_5e9_wait_stream_off_status(cis_shared_data *cis_data)
{
	int ret = 0;
	u32 timeout = 0;

	FIMC_BUG(!cis_data);

#define STREAM_OFF_WAIT_TIME 250
	while (timeout < STREAM_OFF_WAIT_TIME) {
		if (cis_data->is_active_area == false &&
				cis_data->stream_on == false) {
			pr_debug("actual stream off\n");
			break;
		}
		timeout++;
	}

	if (timeout == STREAM_OFF_WAIT_TIME) {
		pr_err("actual stream off wait timeout\n");
		ret = -1;
	}

	return ret;
}

int sensor_5e9_cis_check_rev(struct v4l2_subdev *subdev)
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
		ret = -EINVAL;
		return ret;
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

int sensor_5e9_cis_otp_check_awb_ratio(char *unit, char *golden, char *limit)
{
	int ret = 0;

	float r_g_min = (float)(limit[0]) / 1000;
	float r_g_max = (float)(limit[1]) / 1000;
	float b_g_min = (float)(limit[2]) / 1000;
	float b_g_max = (float)(limit[3]) / 1000;

	/* read by little endian */
	float rg = (float) ((unit[1] << 8) | (unit[0])) / 16384;
	float bg = (float) ((unit[3] << 8) | (unit[2])) / 16384;

	float golden_rg = (float) ((golden[1] << 8) | (golden[0])) / 16384;
	float golden_bg = (float) ((golden[3] << 8) | (golden[2])) / 16384;

	if (rg < (golden_rg - r_g_min) || rg > (golden_rg + r_g_max)) {
		err("%s(): Final RG calibration factors out of range! rg=0x%x golden_rg=0x%x",
			__func__, (unit[1] << 8 | unit[0]), (golden[1] << 8 | golden[0]));
		ret = 1;
	}

	if (bg < (golden_bg - b_g_min) || bg > (golden_bg + b_g_max)) {
		err("%s(): Final BG calibration factors out of range! bg=0x%x, golden_bg=0x%x",
			__func__, (unit[3] << 8 | unit[2]), (golden[3] << 8 | golden[2]));
		ret = 1;
	}

	return ret;
}

int sensor_5e9_cis_otp_check_crc(struct v4l2_subdev *subdev,
		struct fimc_is_device_sensor *device, int grp_offset)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	char *check_buf = (char *)&device->otp_cal_buf[0][0];
	int group = (grp_offset == 0) ? 1 : 2;

	/* ADDR CRC check */
	crc_value = ((check_buf[grp_offset + OTP_GRP_ADDR_CHKSUM + 1] << 8)
			| (check_buf[grp_offset + OTP_GRP_ADDR_CHKSUM]));
	crc16 = sensor_cis_otp_get_crc16(&check_buf[grp_offset + OTP_GRP_ADDR_CRC_START],
						OTP_GRP_ADDR_CRC_SIZE);
	if (crc_value != crc16) {
		err("GR%d: Error to ADDR CRC16 : 0x%x, cal buffer CRC: 0x%x", group, crc16, crc_value);
		ret = -EINVAL;
	} else
		info("GR%d: ADDR CRC16 : 0x%x, cal buffer CRC: 0x%x\n", group, crc16, crc_value);

	/* INFO CRC check */
	crc_value = ((check_buf[grp_offset + OTP_GRP_INFO_CHKSUM + 1] << 8)
			| (check_buf[grp_offset + OTP_GRP_INFO_CHKSUM]));
	crc16 = sensor_cis_otp_get_crc16(&check_buf[grp_offset + OTP_GRP_INFO_CRC_START],
						OTP_GRP_INFO_CRC_SIZE);
	if (crc_value != crc16) {
		err("GR%d: Error to INFO CRC16 : 0x%x, cal buffer CRC: 0x%x", group, crc16, crc_value);
		ret = -EINVAL;
	} else
		info("GR%d: INFO CRC16 : 0x%x, cal buffer CRC: 0x%x\n", group, crc16, crc_value);

	/* AWB CRC check */
	crc_value = ((check_buf[grp_offset + OTP_GRP_AWB_CHKSUM + 1] << 8)
			| (check_buf[grp_offset + OTP_GRP_AWB_CHKSUM]));
	crc16 = sensor_cis_otp_get_crc16(&check_buf[grp_offset + OTP_GRP_AWB_CRC_START],
						OTP_GRP_AWB_CRC_SIZE);
	if (crc_value != crc16) {
		err("GR%d: Error to AWB CRC16 : 0x%x, cal buffer CRC: 0x%x", group, crc16, crc_value);
		ret = -EINVAL;
	} else
		info("GR%d: AWB CRC16 : 0x%x, cal buffer CRC: 0x%x\n", group, crc16, crc_value);

	/* LSC_XTC CRC check */
	crc_value = ((check_buf[grp_offset + OTP_GRP_LSC_XTC_CHKSUM + 1] << 8)
			| (check_buf[grp_offset + OTP_GRP_LSC_XTC_CHKSUM]));
	crc16 = sensor_cis_otp_get_crc16(&check_buf[grp_offset + OTP_GRP_LSC_XTC_CRC_START],
						OTP_GRP_LSC_XTC_CRC_SIZE);
	if (crc_value != crc16) {
		err("GR%d: Error to LSC_XTC CRC16 : 0x%x, cal buffer CRC: 0x%x", group, crc16, crc_value);
		ret = -EINVAL;
	} else
		info("GR%d: LSC_XTC CRC16 : 0x%x, cal buffer CRC: 0x%x\n", group, crc16, crc_value);

	/* LSC_XTC CRC check */
	crc_value = ((check_buf[grp_offset + OTP_GRP_AE_SYNC_CHKSUM + 1] << 8)
			| (check_buf[grp_offset + OTP_GRP_AE_SYNC_CHKSUM]));
	crc16 = sensor_cis_otp_get_crc16(&check_buf[grp_offset + OTP_GRP_AE_SYNC_CRC_START],
						OTP_GRP_AE_SYNC_CRC_SIZE);
	if (crc_value != crc16) {
		err("GR%d: Error to AE_SYNC CRC16 : 0x%x, cal buffer CRC: 0x%x", group, crc16, crc_value);
		ret = -EINVAL;
	} else
		info("GR%d: AE_SYNC CRC16 : 0x%x, cal buffer CRC: 0x%x\n", group, crc16, crc_value);

	return ret;
}

int sensor_5e9_cis_otp_read(struct v4l2_subdev *subdev, struct fimc_is_device_sensor *device)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	u8 val, page;
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

	dbg_sensor(1, "%s, 2. sensor stream on", __func__);
	fimc_is_sensor_write8(client, 0x0100, 0x01);

	/* wait streamon */
	CALL_CISOPS(cis, cis_wait_streamon, subdev);

	dbg_sensor(1, "%s, 3. page select & read cal", __func__);
	for (page = OTP_PAGE_START; page <= OTP_PAGE_END; page++) {
		/* page select & read start */
		fimc_is_sensor_write8(client, OTP_PAGE_SELECT, page);
		fimc_is_sensor_write8(client, OTP_PAGE_CTRL, 0x01);
		usleep_range(1000, 1001);

		/* wait 0x0A01 == 1 [0]: read completed with no errors */
		retry = 500;
		while (retry > 0) {
			fimc_is_sensor_read8(client, OTP_PAGE_ERRCHK, &val);
			if (val == 1)
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
		fimc_is_sensor_write8(client, OTP_PAGE_CTRL, 0x04);
		fimc_is_sensor_write8(client, OTP_PAGE_CTRL, 0x00);
	}

	fimc_is_sensor_write8(client, 0x0100, 0x00);
	msleep(20);
	info("OTP end!!!!!\n");

	I2C_MUTEX_UNLOCK(cis->i2c_lock);

	return ret;
}

int sensor_5e9_cis_otp(struct v4l2_subdev *subdev, struct fimc_is_device_sensor *device)
{
	int ret = 0;
	int otp_group = 0x0;
	int offset = 0;
	char *otp_buf = (char *)&device->otp_cal_buf[0][0];
	char file_str[60];

	snprintf(file_str, sizeof(file_str), "%s%s", OTP_DATA_PATH, device->otp_filename);
	ret = sensor_cis_otp_read_file(file_str, (void *)device->otp_cal_buf, OTP_PAGE_SIZE * 64);
	if (ret) {
		/* OTP data read */
		ret = sensor_5e9_cis_otp_read(subdev, device);
		if (ret < 0) {
			err("Don't read to 5E9 OTP data");
			goto p_err;
		}

		/* Write to OTP data at file */
		ret = sensor_cis_otp_write_file(file_str, (void *)device->otp_cal_buf, OTP_PAGE_SIZE * 64);
		if (ret < 0) {
			err("5E9 OTP data don't file write");
			goto p_err;
		}
	}

	/* Need to first check GROUP2 */
	if (otp_buf[OTP_GRP_FLAG + OTP_GRP2_OFFSET * OTP_PAGE_SIZE] == OTP_DATA_VALID) {
		otp_group = OTP_GROUP_TWO;
		offset = OTP_GRP2_OFFSET * OTP_PAGE_SIZE;
	} else if (otp_buf[OTP_GRP_FLAG] == OTP_DATA_VALID) {
		otp_group = OTP_GROUP_ONE;
		offset = 0;
	} else {
		err("All OTP data are invalid, check module");
		goto p_err;
	}

	/* OTP CRC check */
	ret = sensor_5e9_cis_otp_check_crc(subdev, device, offset);
	if (ret < 0) {
		err("All OTP data CRC check fail, check module");

		device->cal_status[CAMERA_CRC_INDEX_AWB] = CRC_ERROR;
		goto p_err;
	} else {
		u8 *awb_base = &otp_buf[offset + OTP_GRP_AWB_CHKSUM];

		device->cal_status[CAMERA_CRC_INDEX_AWB] = CRC_NO_ERROR;
		ret = sensor_5e9_cis_otp_check_awb_ratio(&awb_base[CURRENT_RG_RATIO_OFFSET],
				&awb_base[MASTER_RG_RATIO_OFFSET],
				&awb_base[RG_MIN_LIMIT_OFFSET]);
		if (ret) {
			err("%s(): 5E9 OTP AWB Group%d ratio out of limit(%d)", __func__, otp_group, ret);
			device->cal_status[CAMERA_CRC_INDEX_AWB] = LIMIT_FAILURE;
			ret = -1;
		}
	}

p_err:
	return ret;
}

/* CIS OPS */
int sensor_5e9_cis_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device = NULL;
	u32 setfile_index = 0;
	cis_setting_info setinfo;

#if USE_OTP_AWB_CAL_DATA
	struct i2c_client *client = NULL;
	u8 selected_page;
	u16 data16[4];
	u8 cal_map_ver[4];
	bool skip_cal_write = false;
#endif

	setinfo.param = NULL;
	setinfo.return_value = 0;

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

#if USE_OTP_AWB_CAL_DATA
	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}
#endif

	BUG_ON(!cis->cis_data);

	cis->cis_data->cur_width = SENSOR_5E9_MAX_WIDTH;
	cis->cis_data->cur_height = SENSOR_5E9_MAX_HEIGHT;
	cis->cis_data->low_expo_start = 33000;
	cis->need_mode_change = false;

	sensor_5e9_cis_data_calculation(sensor_5e9_pllinfos[setfile_index], cis->cis_data);

#if USE_OTP_AWB_CAL_DATA
	/* Read AWB Cal Data from OTPROM */
	printk(KERN_INFO "%s 5E9 AWB Cal data read!\n", __func__);
	ret = fimc_is_sensor_write8(client, 0xA00, 0x04);
	if (unlikely(ret)) {
		err("failed to fimc_is_i2c_write (%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}
	ret = fimc_is_sensor_write8(client, 0xA02, 0x02);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_write8(client, 0xA00, 0x01);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);

	ret = fimc_is_sensor_read8(client, 0xA12, &selected_page);
	if (unlikely(ret)) {
		err("failed to fimc_is_i2c_read (%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}
	printk(KERN_INFO "Camera: otp_bank = %d\n", selected_page);
	if (selected_page == 0x3) {
		printk(KERN_INFO "Camera: OTP 3 page selected\n");
		ret = fimc_is_sensor_write8(client, 0xA00, 0x04);
		if (unlikely(ret))
			err("failed to fimc_is_i2c_read (%d)\n", ret);
		ret = fimc_is_sensor_write8(client, 0xA00, 0x00);
		if (unlikely(ret))
			err("failed to fimc_is_i2c_read (%d)\n", ret);

		usleep_range(1000, 1000);

		ret = fimc_is_sensor_write8(client, 0xA00, 0x04);
		if (unlikely(ret))
			err("failed to fimc_is_i2c_read (%d)\n", ret);
		ret = fimc_is_sensor_write8(client, 0xA02, 0x03);
		if (unlikely(ret))
			err("failed to fimc_is_i2c_read (%d)\n", ret);
		ret = fimc_is_sensor_write8(client, 0xA00, 0x01);
		if (unlikely(ret))
			err("failed to fimc_is_i2c_read (%d)\n", ret);
	}
	ret = fimc_is_sensor_read16(client, 0x0A04, &data16[0]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_read16(client, 0x0A06, &data16[1]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_read16(client, 0x0A08, &data16[2]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_read16(client, 0x0A0A, &data16[3]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);

	ret = fimc_is_sensor_read8(client, 0xA22, &cal_map_ver[0]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_read8(client, 0xA23, &cal_map_ver[1]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_read8(client, 0xA24, &cal_map_ver[2]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_read8(client, 0xA25, &cal_map_ver[3]);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);

	printk(KERN_INFO "5e9 cal map version %c %c %c %c\n",
		cal_map_ver[0], cal_map_ver[1], cal_map_ver[2], cal_map_ver[3]);

	if (cal_map_ver[0] != 0x56 || cal_map_ver[1] != 0x30
		|| cal_map_ver[2] != 0x30 || cal_map_ver[3] < 0x31) {
		printk(KERN_INFO "5e9 cal map version 0x%x 0x%x 0x%x 0x%x\n",
			cal_map_ver[0], cal_map_ver[1], cal_map_ver[2], cal_map_ver[3]);
		skip_cal_write = true;
	}

	ret = fimc_is_sensor_write8(client, 0xA00, 0x04);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);
	ret = fimc_is_sensor_write8(client, 0xA00, 0x00);
	if (unlikely(ret))
		err("failed to fimc_is_i2c_read (%d)\n", ret);

	printk(KERN_INFO "5e9 awb cal data %x %x %x %x\n",
		data16[0], data16[1], data16[2], data16[3]);

	/* Write AWB Cal Data to sensor */
	usleep_range(10000, 10000);

	if (skip_cal_write == false) {
		ret = fimc_is_sensor_write16_array(client, 0x020E, data16, 4);
		if (ret < 0) {
			printk(KERN_INFO "fimc_is_sensor_write16_array fail\n");
			ret = -EINVAL;
			goto p_err;
		}
	}

	cis->use_dgain = false;
#endif

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
		ret = sensor_5e9_cis_otp(subdev, device);
		if (ret < 0) {
			err("5E9 OTP data have problem, check module");
			ret = 0;
		}
	}

p_err:
	return ret;
}

int sensor_5e9_cis_log_status(struct v4l2_subdev *subdev)
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
		ret = -EINVAL;
		goto p_err;
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	pr_err("[SEN:DUMP] *******************************\n");
	ret = fimc_is_sensor_read16(client, 0x0000, &data16);
	if (unlikely(!ret))
		printk("[SEN:DUMP] model_id(%x)\n", data16);

	ret = fimc_is_sensor_read8(client, 0x0002, &data8);
	if (unlikely(!ret))
		printk("[SEN:DUMP] revision_number(%x)\n", data8);

	ret = fimc_is_sensor_read8(client, 0x0005, &data8);
	if (unlikely(!ret))
		printk("[SEN:DUMP] frame_count(%x)\n", data8);

	ret = fimc_is_sensor_read8(client, 0x0100, &data8);
	if (unlikely(!ret))
		pr_err("[SEN:DUMP] mode_select(%x)\n", data8);

	sensor_cis_dump_registers(subdev, sensor_5e9_setfiles[0], sensor_5e9_setfile_sizes[0]);

	pr_err("[SEN:DUMP] *******************************\n");

p_err:
	return ret;
}

#if USE_GROUP_PARAM_HOLD
static int sensor_5e9_cis_group_param_hold_func(struct v4l2_subdev *subdev, unsigned int hold)
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
static inline int sensor_5e9_cis_group_param_hold_func(struct v4l2_subdev *subdev, unsigned int hold)
{
	return 0;
}
#endif

/* Input
 *	hold : true - hold, flase - no hold
 * Output
 *      return: 0 - no effect(already hold or no hold)
 *		positive - setted by request
 *		negative - ERROR value
 */
int sensor_5e9_cis_group_param_hold(struct v4l2_subdev *subdev, bool hold)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	ret = sensor_5e9_cis_group_param_hold_func(subdev, hold);
	if (ret < 0)
		goto p_err;

p_err:
	return ret;
}

int sensor_5e9_cis_set_global_setting(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);

	/* setfile global setting is at camera entrance */
	ret = sensor_cis_set_registers(subdev, sensor_5e9_global, sensor_5e9_global_size);
	if (ret < 0) {
		err("sensor_5e9_set_registers fail!!");
		goto p_err;
	}

	dbg_sensor(1, "[%s] global setting done\n", __func__);

p_err:
	return ret;
}

int sensor_5e9_cis_mode_change(struct v4l2_subdev *subdev, u32 mode)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	if (mode > sensor_5e9_max_setfile_num) {
		err("invalid mode(%d)!!", mode);
		ret = -EINVAL;
		goto p_err;
	}

	sensor_5e9_cis_data_calculation(sensor_5e9_pllinfos[mode], cis->cis_data);

	ret = sensor_cis_set_registers(subdev, sensor_5e9_setfiles[mode], sensor_5e9_setfile_sizes[mode]);
	if (ret < 0) {
		err("sensor_5e9_set_registers fail!!");
		goto p_err;
	}

	cis->cis_data->frame_time = (cis->cis_data->line_readOut_time * cis->cis_data->cur_height / 1000);
	cis->cis_data->rolling_shutter_skew = (cis->cis_data->cur_height - 1) * cis->cis_data->line_readOut_time;
	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__,
		cis->cis_data->frame_time, cis->cis_data->rolling_shutter_skew);

	dbg_sensor(1, "[%s] mode changed(%d)\n", __func__, mode);

p_err:
	return ret;
}


int sensor_5e9_cis_mode_change_throttling(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	sensor_5e9_cis_data_calculation(sensor_5e9_pllinfo_throttling, cis->cis_data);

	I2C_MUTEX_LOCK(cis->i2c_lock);

	ret = sensor_cis_set_registers(subdev, sensor_5e9_setfile_throttling,
				sizeof(sensor_5e9_setfile_throttling) / sizeof(sensor_5e9_setfile_throttling[0]));
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

/* TODO: Sensor set size sequence(sensor done, sensor stop, 3AA done in FW case */
int sensor_5e9_cis_set_size(struct v4l2_subdev *subdev, cis_shared_data *cis_data)
{
	int ret = 0;
	bool binning = false;
	u32 ratio_w = 0, ratio_h = 0, start_x = 0, start_y = 0, end_x = 0, end_y = 0;
	u32 even_x = 0, odd_x = 0, even_y = 0, odd_y = 0;
	struct i2c_client *client = NULL;
	struct fimc_is_cis *cis = NULL;
#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif
	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	FIMC_BUG(!cis);

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	if (unlikely(!cis_data)) {
		err("cis data is NULL");
		if (unlikely(!cis->cis_data)) {
			ret = -EINVAL;
			goto p_err;
		} else {
			cis_data = cis->cis_data;
		}
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	/* Wait actual stream off */
	ret = sensor_5e9_wait_stream_off_status(cis_data);
	if (ret) {
		err("Must stream off\n");
		ret = -EINVAL;
		goto p_err;
	}

	binning = cis_data->binning;
	if (binning) {
		ratio_w = (SENSOR_5E9_MAX_WIDTH / cis_data->cur_width);
		ratio_h = (SENSOR_5E9_MAX_HEIGHT / cis_data->cur_height);
	} else {
		ratio_w = 1;
		ratio_h = 1;
	}

	if (((cis_data->cur_width * ratio_w) > SENSOR_5E9_MAX_WIDTH) ||
		((cis_data->cur_height * ratio_h) > SENSOR_5E9_MAX_HEIGHT)) {
		err("Config max sensor size over~!!\n");
		ret = -EINVAL;
		goto p_err;
	}

	/* 1. page_select */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x2000);
	if (ret < 0)
		goto p_err;

	/* 2. pixel address region setting */
	start_x = ((SENSOR_5E9_MAX_WIDTH - cis_data->cur_width * ratio_w) / 2) & (~0x1);
	start_y = ((SENSOR_5E9_MAX_HEIGHT - cis_data->cur_height * ratio_h) / 2) & (~0x1);
	end_x = start_x + (cis_data->cur_width * ratio_w - 1);
	end_y = start_y + (cis_data->cur_height * ratio_h - 1);

	if (!(end_x & (0x1)) || !(end_y & (0x1))) {
		err("Sensor pixel end address must odd\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = fimc_is_sensor_write16(client, 0x0344, start_x);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0346, start_y);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0348, end_x);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x034A, end_y);
	if (ret < 0)
		goto p_err;

	/* 3. output address setting */
	ret = fimc_is_sensor_write16(client, 0x034C, cis_data->cur_width);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x034E, cis_data->cur_height);
	if (ret < 0)
		goto p_err;

	/* If not use to binning, sensor image should set only crop */
	if (!binning) {
		dbg_sensor(1, "Sensor size set is not binning\n");
		goto p_err;
	}

	/* 4. sub sampling setting */
	even_x = 1;	/* 1: not use to even sampling */
	even_y = 1;
	odd_x = (ratio_w * 2) - even_x;
	odd_y = (ratio_h * 2) - even_y;

	ret = fimc_is_sensor_write16(client, 0x0380, even_x);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0382, odd_x);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0384, even_y);
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0386, odd_y);
	if (ret < 0)
		goto p_err;

	/* 5. binnig setting */
	ret = fimc_is_sensor_write8(client, 0x0900, binning);	/* 1:  binning enable, 0: disable */
	if (ret < 0)
		goto p_err;
	ret = fimc_is_sensor_write8(client, 0x0901, (ratio_w << 4) | ratio_h);
	if (ret < 0)
		goto p_err;

	/* 6. scaling setting: but not use */
	/* scaling_mode (0: No scaling, 1: Horizontal, 2: Full) */
	ret = fimc_is_sensor_write16(client, 0x0400, 0x0000);
	if (ret < 0)
		goto p_err;
	/* down_scale_m: 1 to 16 upwards (scale_n: 16(fixed)) */
	/* down scale factor = down_scale_m / down_scale_n */
	ret = fimc_is_sensor_write16(client, 0x0404, 0x0010);
	if (ret < 0)
		goto p_err;

	cis_data->frame_time = (cis_data->line_readOut_time * cis_data->cur_height / 1000);
	cis->cis_data->rolling_shutter_skew = (cis->cis_data->cur_height - 1) * cis->cis_data->line_readOut_time;
	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__,
		cis->cis_data->frame_time, cis->cis_data->rolling_shutter_skew);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_stream_on(struct v4l2_subdev *subdev)
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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	ret = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
	if (ret < 0)
		err("[%s] sensor_5e9_cis_group_param_hold_func fail\n", __func__);

	/* Sensor stream on */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x4000);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x6028, 0x4000, ret);
	ret = fimc_is_sensor_write8(client, 0x0100, 0x01);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x0100, 0x01, ret);

	/* WDR */
	if (fimc_is_vender_wdr_mode_on(cis_data))
		ret = fimc_is_sensor_write8(client, 0x0216, 0x01);
	else
		ret = fimc_is_sensor_write8(client, 0x0216, 0x00);

	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), ret(%d)\n", 0x216, ret);

	cis_data->stream_on = true;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_stream_off(struct v4l2_subdev *subdev)
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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	ret = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
	if (ret < 0)
		err("[%s] sensor_5e9_cis_group_param_hold_func fail\n", __func__);

	/* Sensor stream off */
	ret = fimc_is_sensor_write16(client, 0x6028, 0x4000);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x6028, 0x4000, ret);
	ret = fimc_is_sensor_write8(client, 0x0100, 0x00);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x0100, 0x00, ret);

	cis_data->stream_on = false;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_set_exposure_time(struct v4l2_subdev *subdev, struct ae_param *target_exposure)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 vt_pic_clk_freq_mhz = 0;
	u16 long_coarse_int = 0;
	u16 short_coarse_int = 0;
	u32 line_length_pck = 0;
	u32 min_fine_int = 0;

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
		ret = -EINVAL;
		goto p_err;
	}

	if ((target_exposure->long_val <= 0) || (target_exposure->short_val <= 0)) {
		err("[%s] invalid target exposure(%d, %d)\n", __func__,
				target_exposure->long_val, target_exposure->short_val);
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), target long(%d), short(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, target_exposure->long_val, target_exposure->short_val);

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;
	min_fine_int = cis_data->min_fine_integration_time;

	dbg_sensor(1, "[MOD:D:%d] %s, vt_pic_clk_freq_mhz (%d), line_length_pck(%d), min_fine_int (%d)\n",
		cis->id, __func__, vt_pic_clk_freq_mhz, line_length_pck, min_fine_int);

	long_coarse_int = ((target_exposure->long_val * vt_pic_clk_freq_mhz) - min_fine_int) / line_length_pck;
	short_coarse_int = ((target_exposure->short_val * vt_pic_clk_freq_mhz) - min_fine_int) / line_length_pck;

	if (long_coarse_int > cis_data->max_coarse_integration_time) {
		long_coarse_int = cis_data->max_coarse_integration_time;
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), long coarse(%d) max\n", cis->id, __func__,
			cis_data->sen_vsync_count, long_coarse_int);
	}

	if (short_coarse_int > cis_data->max_coarse_integration_time) {
		short_coarse_int = cis_data->max_coarse_integration_time;
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), short coarse(%d) max\n", cis->id, __func__,
			cis_data->sen_vsync_count, short_coarse_int);
	}

	if (long_coarse_int < cis_data->min_coarse_integration_time) {
		long_coarse_int = cis_data->min_coarse_integration_time;
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), long coarse(%d) min\n", cis->id, __func__,
			cis_data->sen_vsync_count, long_coarse_int);
	}

	if (short_coarse_int < cis_data->min_coarse_integration_time) {
		short_coarse_int = cis_data->min_coarse_integration_time;
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), short coarse(%d) min\n", cis->id, __func__,
			cis_data->sen_vsync_count, short_coarse_int);
	}

	dbg_sensor(1, "[MOD:D:%d] %s, frame_length_lines(%#x), long_coarse_int %#x, short_coarse_int %#x\n",
		cis->id, __func__, cis_data->frame_length_lines, long_coarse_int, short_coarse_int);

	hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	/* Short exposure */
	ret = fimc_is_sensor_write16(client, 0x0202, short_coarse_int);
	if (ret < 0)
		goto p_err;

	/* Long exposure */
	if (fimc_is_vender_wdr_mode_on(cis_data)) {
		ret = fimc_is_sensor_write16(client, 0x021E, long_coarse_int);
		if (ret < 0)
			goto p_err;
	}

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_5e9_cis_get_min_exposure_time(struct v4l2_subdev *subdev, u32 *min_expo)
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
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_get_max_exposure_time(struct v4l2_subdev *subdev, u32 *max_expo)
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

	/* TODO: Is this values update hear? */
	cis_data->max_margin_fine_integration_time = max_fine_margin;
	cis_data->max_coarse_integration_time = max_coarse;

	dbg_sensor(1, "[%s] max integration time %d, max margin fine integration %d, max coarse integration %d\n",
			__func__, max_integration_time, cis_data->max_margin_fine_integration_time,
			cis_data->max_coarse_integration_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_adjust_frame_duration(struct v4l2_subdev *subdev,
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

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;
	frame_length_lines = ((vt_pic_clk_freq_mhz * input_exposure_time) / line_length_pck);
	frame_length_lines += cis_data->max_margin_coarse_integration_time;

	frame_duration = (frame_length_lines * line_length_pck) / vt_pic_clk_freq_mhz;

	dbg_sensor(1, "[%s] input exp(%d), adj duration, frame duraion(%d), min_frame_us(%d)\n",
			__func__, input_exposure_time, frame_duration, cis_data->min_frame_us_time);
	dbg_sensor(1, "[%s] adj duration, frame duraion(%d), min_frame_us(%d)\n",
			__func__, frame_duration, cis_data->min_frame_us_time);

	*target_duration = MAX(frame_duration, cis_data->min_frame_us_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_5e9_cis_set_frame_duration(struct v4l2_subdev *subdev, u32 frame_duration)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 line_length_pck = 0;
	u16 frame_length_lines = 0;
	u64 numerator;
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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	if (frame_duration < cis_data->min_frame_us_time) {
		dbg_sensor(1, "frame duration is less than min(%d)\n", frame_duration);
		frame_duration = cis_data->min_frame_us_time;
	}

	line_length_pck = cis_data->line_length_pck;
	numerator = (u64)cis_data->pclk * frame_duration;
	frame_length_lines = (u16)((numerator / line_length_pck) / (1000 * 1000));

	dbg_sensor(1, "[MOD:D:%d] %s, vt_pic_clk(%#x) frame_duration = %d us,"
		KERN_CONT "(line_length_pck%#x), frame_length_lines(%#x)\n",
		cis->id, __func__, cis_data->pclk, frame_duration, line_length_pck, frame_length_lines);

	hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_write16(client, 0x0340, frame_length_lines);
	if (ret < 0)
		goto p_err;

	cis_data->cur_frame_us_time = frame_duration;
	cis_data->frame_length_lines = frame_length_lines;
	cis_data->max_coarse_integration_time = cis_data->frame_length_lines
		- cis_data->max_margin_coarse_integration_time;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_5e9_cis_set_frame_rate(struct v4l2_subdev *subdev, u32 min_fps)
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

	ret = sensor_5e9_cis_set_frame_duration(subdev, frame_duration);
	if (ret < 0) {
		err("[MOD:D:%d] %s, set frame duration is fail(%d)\n",
			cis->id, __func__, ret);
		goto p_err;
	}

	cis_data->min_frame_us_time = frame_duration;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:

	return ret;
}

int sensor_5e9_cis_adjust_analog_gain(struct v4l2_subdev *subdev, u32 input_again, u32 *target_permile)
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

int sensor_5e9_cis_set_analog_gain(struct v4l2_subdev *subdev, struct ae_param *again)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 analog_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;

	do_gettimeofday(&st);
#endif

	FIMC_BUG(!subdev);
	FIMC_BUG(!again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);
	FIMC_BUG(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	analog_gain = (u16)sensor_cis_calc_again_code(again->val);

	if (analog_gain < cis->cis_data->min_analog_gain[0])
		analog_gain = cis->cis_data->min_analog_gain[0];

	if (analog_gain > cis->cis_data->max_analog_gain[0])
		analog_gain = cis->cis_data->max_analog_gain[0];

	dbg_sensor(1, "[MOD:D:%d] %s, input_again = %d us, analog_gain(%#x)\n",
			cis->id, __func__, again->val, analog_gain);

	hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_write16(client, 0x0204, analog_gain);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_5e9_cis_get_analog_gain(struct v4l2_subdev *subdev, u32 *again)
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
		ret = -EINVAL;
		goto p_err;
	}

	hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x01);
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
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_5e9_cis_get_min_analog_gain(struct v4l2_subdev *subdev, u32 *min_again)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 read_value = 0;

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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	ret = fimc_is_sensor_read16(client, 0x0084, &read_value);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x0084, read_value, ret);

	cis_data->min_analog_gain[0] = read_value;

	cis_data->min_analog_gain[1] = sensor_cis_calc_again_permile(read_value);

	*min_again = cis_data->min_analog_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__,
		cis_data->min_analog_gain[0], cis_data->min_analog_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_get_max_analog_gain(struct v4l2_subdev *subdev, u32 *max_again)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 read_value = 0;

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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	ret = fimc_is_sensor_read16(client, 0x0086, &read_value);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x0086, read_value, ret);

	cis_data->max_analog_gain[0] = read_value;

	cis_data->max_analog_gain[1] = sensor_cis_calc_again_permile(read_value);

	*max_again = cis_data->max_analog_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__,
		cis_data->max_analog_gain[0], cis_data->max_analog_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

#if USE_OTP_AWB_CAL_DATA
// Do nothing ! Digital gains are used to compensate for the AWB M2M (module to mudule) variation
int sensor_5e9_cis_set_digital_gain(struct v4l2_subdev *subdev, struct ae_param *dgain)
{
	return 0;
}
#else
int sensor_5e9_cis_set_digital_gain(struct v4l2_subdev *subdev, struct ae_param *dgain)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 long_gain = 0;
	u16 short_gain = 0;
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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	long_gain = (u16)sensor_cis_calc_dgain_code(dgain->long_val);
	short_gain = (u16)sensor_cis_calc_dgain_code(dgain->short_val);

	if (long_gain < cis->cis_data->min_digital_gain[0])
		long_gain = cis->cis_data->min_digital_gain[0];
	if (long_gain > cis->cis_data->max_digital_gain[0])
		long_gain = cis->cis_data->max_digital_gain[0];
	if (short_gain < cis->cis_data->min_digital_gain[0])
		short_gain = cis->cis_data->min_digital_gain[0];
	if (short_gain > cis->cis_data->max_digital_gain[0])
		short_gain = cis->cis_data->max_digital_gain[0];

	dbg_sensor(1, "[MOD:D:%d] %s, input_dgain = %d/%d us, long_gain(%#x), short_gain(%#x)\n",
			cis->id, __func__, dgain->long_val, dgain->short_val, long_gain, short_gain);

	hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	dgains[0] = dgains[1] = dgains[2] = dgains[3] = short_gain;
	/* Short digital gain */
	ret = fimc_is_sensor_write16_array(client, 0x020E, dgains, 4);
	if (ret < 0)
		goto p_err;

	/* Long digital gain */
	if (fimc_is_vender_wdr_mode_on(cis_data)) {
		ret = fimc_is_sensor_write16(client, 0x305C, long_gain);
		if (ret < 0)
			goto p_err;
	}

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}
#endif

int sensor_5e9_cis_get_digital_gain(struct v4l2_subdev *subdev, u32 *dgain)
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
		ret = -EINVAL;
		goto p_err;
	}

	hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x01);
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
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_5e9_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_5e9_cis_get_min_digital_gain(struct v4l2_subdev *subdev, u32 *min_dgain)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 read_value = 0;

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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	ret = fimc_is_sensor_read16(client, 0x1084, &read_value);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x1084, read_value, ret);

	cis_data->min_digital_gain[0] = read_value;

	cis_data->min_digital_gain[1] = sensor_cis_calc_dgain_permile(read_value);

	*min_dgain = cis_data->min_digital_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__,
		cis_data->min_digital_gain[0], cis_data->min_digital_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_5e9_cis_get_max_digital_gain(struct v4l2_subdev *subdev, u32 *max_dgain)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 read_value = 0;

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
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	ret = fimc_is_sensor_read16(client, 0x1086, &read_value);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x1086, read_value, ret);

	cis_data->max_digital_gain[0] = read_value;

	cis_data->max_digital_gain[1] = sensor_cis_calc_dgain_permile(read_value);

	*max_dgain = cis_data->max_digital_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__,
		cis_data->max_digital_gain[0], cis_data->max_digital_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

static int sensor_5e9_cis_set_dual_master_setting(struct fimc_is_cis *cis)
{
	int ret = 0;
	struct i2c_client *client;

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
	       err("client is NULL");
	       ret = -EINVAL;
	       goto p_err;
	}

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	/* Vsync out */
	ret = fimc_is_sensor_write8(client, 0x3C03, 0x0F);
	if (unlikely(ret))
	       err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x3C03, 0x0F, ret);

p_err:
	return ret;
}

static int sensor_5e9_cis_set_dual_slave_setting(struct fimc_is_cis *cis)
{
	int ret = 0;
	struct i2c_client *client;

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	/* Vsync Input Source Select */
	ret = fimc_is_sensor_write8(client, 0x3C02, 0x01);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x3C02, 0x01, ret);
	/* gpio1 Input Option */
	ret = fimc_is_sensor_write8(client, 0x3C05, 0x1D);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x3C02, 0x1D, ret);
	/* Dual Sync Slave enable */
	ret = fimc_is_sensor_write8(client, 0x3500, 0x03);
	if (unlikely(ret))
		err("i2c treansfer fail addr(%x), val(%x), ret(%d)\n", 0x3500, 0x03, ret);

p_err:
	return ret;
}

int sensor_5e9_cis_set_dual_setting(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	FIMC_BUG(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	switch (cis->dual_sync_mode) {
	case DUAL_SYNC_MASTER:
		ret = sensor_5e9_cis_set_dual_master_setting(cis);
		if (ret)
			err("5e9 dual master setting fail");
		break;
	case DUAL_SYNC_SLAVE:
		ret = sensor_5e9_cis_set_dual_slave_setting(cis);
		if (ret)
			err("5e9 dual slave setting fail");
		break;
	default:
		err("invalid cis->dual_sync_mode(%d)\n", cis->dual_sync_mode);
		ret = -EINVAL;
	}

p_err:
	return ret;
}

static struct fimc_is_cis_ops cis_ops = {
	.cis_init = sensor_5e9_cis_init,
	.cis_log_status = sensor_5e9_cis_log_status,
	.cis_group_param_hold = sensor_5e9_cis_group_param_hold,
	.cis_set_global_setting = sensor_5e9_cis_set_global_setting,
	.cis_mode_change = sensor_5e9_cis_mode_change,
	.cis_set_size = sensor_5e9_cis_set_size,
	.cis_stream_on = sensor_5e9_cis_stream_on,
	.cis_stream_off = sensor_5e9_cis_stream_off,
	.cis_set_exposure_time = sensor_5e9_cis_set_exposure_time,
	.cis_get_min_exposure_time = sensor_5e9_cis_get_min_exposure_time,
	.cis_get_max_exposure_time = sensor_5e9_cis_get_max_exposure_time,
	.cis_adjust_frame_duration = sensor_5e9_cis_adjust_frame_duration,
	.cis_set_frame_duration = sensor_5e9_cis_set_frame_duration,
	.cis_set_frame_rate = sensor_5e9_cis_set_frame_rate,
	.cis_adjust_analog_gain = sensor_5e9_cis_adjust_analog_gain,
	.cis_set_analog_gain = sensor_5e9_cis_set_analog_gain,
	.cis_get_analog_gain = sensor_5e9_cis_get_analog_gain,
	.cis_get_min_analog_gain = sensor_5e9_cis_get_min_analog_gain,
	.cis_get_max_analog_gain = sensor_5e9_cis_get_max_analog_gain,
	.cis_set_digital_gain = sensor_5e9_cis_set_digital_gain,
	.cis_get_digital_gain = sensor_5e9_cis_get_digital_gain,
	.cis_get_min_digital_gain = sensor_5e9_cis_get_min_digital_gain,
	.cis_get_max_digital_gain = sensor_5e9_cis_get_max_digital_gain,
	.cis_compensate_gain_for_extremely_br = sensor_cis_compensate_gain_for_extremely_br,
	.cis_wait_streamoff = sensor_cis_wait_streamoff,
	.cis_wait_streamon = sensor_cis_wait_streamon,
	.cis_set_initial_exposure = sensor_cis_set_initial_exposure,
	.cis_check_rev = sensor_5e9_cis_check_rev,
	.cis_factory_test = sensor_cis_factory_test,
	.cis_set_dual_setting = sensor_5e9_cis_set_dual_setting,
	.cis_mode_change_throttling = sensor_5e9_cis_mode_change_throttling,
};

static int cis_5e9_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	int i;
	struct fimc_is_core *core = NULL;
	struct v4l2_subdev *subdev_cis = NULL;
	struct fimc_is_cis *cis = NULL;
	struct fimc_is_device_sensor *device = NULL;
	struct fimc_is_device_sensor_peri *sensor_peri = NULL;
	u32 sensor_id = 0;
	char const *setfile;
	struct device *dev;
	struct device_node *dnode;
#if defined(CONFIG_USE_DIRECT_IS_CONTROL) && defined(CONFIG_CAMERA_OTPROM_SUPPORT_FRONT)
	struct fimc_is_vender_specific *specific = NULL;
#endif

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
		err("sensor_id read is fail(%d)", ret);
		goto p_err;
	}

	probe_info("%s sensor_id %d\n", __func__, sensor_id);

	device = &core->sensor[sensor_id];

	sensor_peri = find_peri_by_cis_id(device, SENSOR_NAME_S5K5E9);
	if (!sensor_peri) {
		probe_info("sensor peri is not yet probed");
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
		ret = -ENOMEM;
		goto p_err;
	}
	sensor_peri->subdev_cis = subdev_cis;

	cis->id = SENSOR_NAME_S5K5E9;
	cis->subdev = subdev_cis;
	cis->device = 0;
	cis->client = client;
	sensor_peri->module->client = cis->client;
	cis->ctrl_delay = N_PLUS_ONE_FRAME;
#if defined(CONFIG_USE_DIRECT_IS_CONTROL) && defined(CONFIG_CAMERA_OTPROM_SUPPORT_FRONT)
	specific = core->vender.private_data;
	specific->front_cis_client = client;
#endif

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

	if (of_property_read_bool(dnode, "dual_sync_mode")) {
		ret = of_property_read_u32(dnode, "dual_sync_mode", &cis->dual_sync_mode);
		if (ret)
			warn("dual_sync_mode read is fail(%d)", ret);
	} else {
		cis->dual_sync_mode = DUAL_SYNC_NONE;
	}

	probe_info("%s dual_sync_mode %d\n", __func__, cis->dual_sync_mode);

	cis->use_dgain = true;
	cis->hdr_ctrl_by_again = false;

	ret = of_property_read_string(dnode, "setfile", &setfile);
	if (ret) {
		err("setfile index read fail(%d), take default setfile!!", ret);
		setfile = "default";
	}

	if (strcmp(setfile, "default") == 0 ||
			strcmp(setfile, "setA") == 0) {
		probe_info("%s setfile_A\n", __func__);
		sensor_5e9_global = sensor_5e9_setfile_A_Global;
		sensor_5e9_global_size = ARRAY_SIZE(sensor_5e9_setfile_A_Global);
		sensor_5e9_setfiles = sensor_5e9_setfiles_A;
		sensor_5e9_setfile_sizes = sensor_5e9_setfile_A_sizes;
		sensor_5e9_pllinfos = sensor_5e9_pllinfos_A;
		sensor_5e9_max_setfile_num = ARRAY_SIZE(sensor_5e9_setfiles_A);
	} else if (strcmp(setfile, "setB") == 0) {
		probe_info("%s setfile_B\n", __func__);
		sensor_5e9_global = sensor_5e9_setfile_B_Global;
		sensor_5e9_global_size = ARRAY_SIZE(sensor_5e9_setfile_B_Global);
		sensor_5e9_setfiles = sensor_5e9_setfiles_B;
		sensor_5e9_setfile_sizes = sensor_5e9_setfile_B_sizes;
		sensor_5e9_pllinfos = sensor_5e9_pllinfos_B;
		sensor_5e9_max_setfile_num = ARRAY_SIZE(sensor_5e9_setfiles_B);
	} else if (strcmp(setfile, "setC") == 0) {
		probe_info("%s setfile_C\n", __func__);
		sensor_5e9_global = sensor_5e9_setfile_C_Global;
		sensor_5e9_global_size = ARRAY_SIZE(sensor_5e9_setfile_C_Global);
		sensor_5e9_setfiles = sensor_5e9_setfiles_C;
		sensor_5e9_setfile_sizes = sensor_5e9_setfile_C_sizes;
		sensor_5e9_pllinfos = sensor_5e9_pllinfos_C;
		sensor_5e9_max_setfile_num = ARRAY_SIZE(sensor_5e9_setfiles_C);

		/* throttling setting */
		sensor_5e9_setfile_throttling = sensor_5e9_setfile_C_2592x1944_15fps;
		sensor_5e9_pllinfo_throttling = &sensor_5e9_pllinfo_C_2592x1944_15fps;
	} else {
		err("%s setfile index out of bound, take default (setfile_A)", __func__);
		sensor_5e9_global = sensor_5e9_setfile_A_Global;
		sensor_5e9_global_size = ARRAY_SIZE(sensor_5e9_setfile_A_Global);
		sensor_5e9_setfiles = sensor_5e9_setfiles_A;
		sensor_5e9_setfile_sizes = sensor_5e9_setfile_A_sizes;
		sensor_5e9_pllinfos = sensor_5e9_pllinfos_A;
		sensor_5e9_max_setfile_num = ARRAY_SIZE(sensor_5e9_setfiles_A);
	}

	cis->use_initial_ae = of_property_read_bool(dnode, "use_initial_ae");
	probe_info("%s use initial_ae(%d)\n", __func__, cis->use_initial_ae);

	device->use_otp_cal = of_property_read_bool(dnode, "use_otp_cal");
	probe_info("%s use otp_cal(%d)\n", __func__, device->use_otp_cal);
	if (device->use_otp_cal) {
		ret = of_property_read_string(dnode, "otp_filename", &device->otp_filename);
		if (ret) {
			err("OTP filename read fail(%d), take default name", ret);
			device->otp_filename = "5e9_otp_cal_data.bin";
		}
		probe_info("%s otp_filename(%s)\n", __func__, device->otp_filename);
	}

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

static const struct of_device_id sensor_cis_5e9_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-cis-5e9",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_cis_5e9_match);

static const struct i2c_device_id sensor_cis_5e9_idt[] = {
	{ SENSOR_NAME, 0 },
	{},
};

static struct i2c_driver sensor_cis_5e9_driver = {
	.probe	= cis_5e9_probe,
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_cis_5e9_match,
		.suppress_bind_attrs = true,
	},
	.id_table = sensor_cis_5e9_idt
};

static int __init sensor_cis_5e9_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_cis_5e9_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_cis_5e9_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_cis_5e9_init);
