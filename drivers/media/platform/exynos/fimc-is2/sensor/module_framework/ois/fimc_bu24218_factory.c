/*
 * Copyright (C) 2019 Motorola Mobility LLC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "fimc_bu24218_factory.h"

#define check_result(ret) {if (ret < 0) goto ERROR;}

int fimc_is_factory_ois_get_fw_rev(struct v4l2_subdev *subdev, uint32_t *result)
{
	int ret = 0;
	char version[4] = {0, };
	struct fimc_is_ois *ois = NULL;

	FIMC_BUG(!subdev);

	info("%s: E", __func__);

	ois = (struct fimc_is_ois *)v4l2_get_subdevdata(subdev);
	if(!ois) {
		err("%s: ois subdev is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	WARN_ON(!ois);

	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_FW_VERSION, &version[0]);
	check_result(ret);

	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_FW_VERSION + 1, &version[1]);
	check_result(ret);

	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_FW_VERSION + 2, &version[2]);
	check_result(ret);

	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_FW_VERSION + 3, &version[3]);
	check_result(ret);

	info("%s: version %d %d %d %d",  __func__, version[0], version[1], version[2], version[3]);
	*result = version[0] | (version[1] << 8) | (version[2] << 16) | (version[3] << 24);

	return 0;
ERROR:
	err("%s: Failed with %d", __func__, ret);
	return -1;
}


int fimc_is_factory_ois_get_hea(struct v4l2_subdev  *subdev, struct fimc_is_ois_hea_parameters *result)
{
	int ret = 0;
	uint32_t data[4] = {0, 0, 0, 0};
	struct fimc_is_ois *ois = NULL;

	FIMC_BUG(!subdev);

	info("%s: E", __func__);

	ois = (struct fimc_is_ois *)v4l2_get_subdevdata(subdev);
	if(!ois) {
		err("%s: ois subdev is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	WARN_ON(!ois);

	// 1. change to calibration mode
	ret = fimc_is_factory_ois_calibration_mode(ois->client);
	check_result(ret);
	usleep_range(500,500);
	
	// 2. set to zero position
	ret = fimc_is_factory_ois_set_zero_pos(ois->client);
	check_result(ret);
	usleep_range(20000,20000);
	
	// 3. set x max position and read ADC
	ret = fimc_is_factory_ois_set_x_pos(ois->client, MOT_BU24218_VCM_DRV_MAX_HI, MOT_BU24218_VCM_DRV_MAX_LO);
	check_result(ret);
	usleep_range(20000,20000);
	ret = fimc_is_factory_ois_get_x_adc(ois->client,&data[0]);
	check_result(ret);
	
	// 4. set x min position and read ADC
	ret = fimc_is_factory_ois_set_x_pos(ois->client, MOT_BU24218_VCM_DRV_MIN_HI, MOT_BU24218_VCM_DRV_MIN_LO);
	check_result(ret);
	usleep_range(20000,20000);
	ret = fimc_is_factory_ois_get_x_adc(ois->client,&data[1]);
	check_result(ret);
	
	// 5. set to zero position
	ret = fimc_is_factory_ois_set_zero_pos(ois->client);
	check_result(ret);
	usleep_range(20000,20000);
	
	// 6. set y max position and read ADC
	ret = fimc_is_factory_ois_set_y_pos(ois->client, MOT_BU24218_VCM_DRV_MAX_HI, MOT_BU24218_VCM_DRV_MAX_LO);
	check_result(ret);
	usleep_range(20000,20000);
	ret = fimc_is_factory_ois_get_y_adc(ois->client,&data[2]);
	check_result(ret);
	
	// 7. set y min position and read ADC
	ret = fimc_is_factory_ois_set_y_pos(ois->client, MOT_BU24218_VCM_DRV_MIN_HI, MOT_BU24218_VCM_DRV_MIN_LO);
	check_result(ret);
	usleep_range(20000,20000);
	ret = fimc_is_factory_ois_get_y_adc(ois->client,&data[3]);
	check_result(ret);
	
	result->x_max = data[0];
	result->x_min = data[1];
	result->y_max = data[2];
	result->y_min = data[3];

	info("%s: result %d %d %d %d", __func__,
		result->x_max, result->x_min, result->y_max, result->y_min);
	
	return 0;

ERROR:
	err("%s: Failed with %d", __func__, ret);
	return -1;
}

int fimc_is_factory_ois_poll(struct i2c_client *client,
		u16 addr, u8 expdata)
{
	int ret = 0, i = 0;
	uint8_t data = 0;
	do {
		usleep_range(200, 200);
		ret = fimc_is_ois_read(client, addr, &data);
		check_result(ret);
	} while (data != expdata && i++ < 100);

	if (i >= 100 || data != expdata) {
		err("%s addr %x expdata=%d act data=%x tries=%d", __func__,
		(unsigned int)addr, (unsigned int)expdata, (unsigned int)data, i);
		return -1;
	}
	return 0;
ERROR:
	err("%s: Failed with %d", __func__, ret);
	return -1;
}


int fimc_is_factory_ois_calibration_mode(struct i2c_client *client)
{
	int ret = 0;
	
	fimc_is_ois_write(client, MOT_BU24218_REG_MODE, 0x01);
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at calib 01");
		return ret;
	}
	fimc_is_ois_write(client, MOT_BU24218_REG_LINEARITY, 0x00);
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_LINEARITY, 0x00);
	if (ret < 0) {
		err("fail at linearity");
		return ret;
	}
	fimc_is_ois_write(client, MOT_BU24218_REG_GYRO_ACCESS, 0x02);
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at gyro access");
		return ret;
	}
	fimc_is_ois_write(client, MOT_BU24218_REG_MODE, 0x00);
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at calib 00");
		return ret;
	}
	fimc_is_ois_write(client, MOT_BU24218_REG_MODE, 0x04);
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at calib 04");
		return ret;
	}
	
	return ret;
}

int fimc_is_factory_ois_set_zero_pos(struct i2c_client *client)
{
	int ret = 0;
	fimc_is_factory_ois_set_x_pos(client, MOT_BU24218_VCM_DRV_ZERO_HI, MOT_BU24218_VCM_DRV_ZERO_LO);
	fimc_is_factory_ois_set_y_pos(client, MOT_BU24218_VCM_DRV_ZERO_HI, MOT_BU24218_VCM_DRV_ZERO_LO);
	
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at set to zero");
	}
    return ret;
}

int fimc_is_factory_ois_set_x_pos(struct i2c_client *client, uint8_t hi, uint8_t lo)
{
	int ret = 0;
	fimc_is_ois_write(client, MOT_BU24218_REG_POS_XCH,     hi);
	fimc_is_ois_write(client, MOT_BU24218_REG_POS_XCH + 1, lo);	
	
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at %s",__func__);
	}
    return ret;
}

int fimc_is_factory_ois_set_y_pos(struct i2c_client *client, uint8_t hi, uint8_t lo)
{
	int ret = 0;
	fimc_is_ois_write(client, MOT_BU24218_REG_POS_YCH,     hi);
	fimc_is_ois_write(client, MOT_BU24218_REG_POS_YCH + 1, lo);	
	
	ret = fimc_is_factory_ois_poll(client, MOT_BU24218_REG_STATUS, 0x01);
	if (ret < 0) {
		err("fail at %s",__func__);
	}
    return ret;
}

int fimc_is_factory_ois_get_x_adc(struct i2c_client *client, uint32_t* data)
{
	int ret = 0;
	uint8_t raw_adc_hi = 0, raw_adc_lo = 0;

    fimc_is_ois_write(client, MOT_BU24218_REG_ADC_LATCH, MOT_BU24218_ADC_LATCH_X);
	usleep_range(11000,11000);

    ret = fimc_is_ois_read(client, MOT_BU24218_REG_ADC_VAL, &raw_adc_hi);
	if (ret < 0) {
		err("fail at %s",__func__);
		return ret;
	}
	usleep_range(500,500);
    ret = fimc_is_ois_read(client, MOT_BU24218_REG_ADC_VAL+1, &raw_adc_lo);
	if (ret < 0) {
		err("fail at %s",__func__);
		return ret;
	}
	usleep_range(500,500);

	*data = (uint32_t)(raw_adc_hi << 8) + raw_adc_lo;
	return ret;
}

int fimc_is_factory_ois_get_y_adc(struct i2c_client *client, uint32_t* data)
{
	int ret = 0;
	uint8_t raw_adc_hi = 0, raw_adc_lo = 0;

    fimc_is_ois_write(client, MOT_BU24218_REG_ADC_LATCH, MOT_BU24218_ADC_LATCH_Y);
	usleep_range(11000,11000);

    ret = fimc_is_ois_read(client, MOT_BU24218_REG_ADC_VAL, &raw_adc_hi);
	if (ret < 0) {
		err("fail at %s",__func__);
		return ret;
	}
	usleep_range(500,500);
    ret = fimc_is_ois_read(client, MOT_BU24218_REG_ADC_VAL+1, &raw_adc_lo);
	if (ret < 0) {
		err("fail at %s",__func__);
		return ret;
	}
	usleep_range(500,500);

	*data = (uint32_t)(raw_adc_hi << 8) + raw_adc_lo;
    return ret;
}
