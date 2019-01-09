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
		err("%s, ois subdev is NULL", __func__);
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

	info("%s: fw version:%d, %d, %d, %d",  __func__, version[0], version[1], version[2], version[3]);
	*result = version[0] | (version[1] << 8) | (version[2] << 16) | (version[3] << 24);

	return 0;
ERROR:
	err("%s: Failed with %d", __func__, ret);
	return -1;
}


int fimc_is_factory_ois_get_hea(struct v4l2_subdev  *subdev, uint32_t *result)
{
	int ret = 0;
	char data[4] = {0, 0, 0, 0};
	struct fimc_is_ois *ois = NULL;

	FIMC_BUG(!subdev);

	info("%s: E", __func__);

	ois = (struct fimc_is_ois *)v4l2_get_subdevdata(subdev);
	if(!ois) {
		err("%s, ois subdev is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	WARN_ON(!ois);

	// 1. change to calibration mode
	ret = fimc_is_factory_ois_poll(ois->client, MOT_BU24218_REG_STATUS, 0x01);
	check_result(ret);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_MODE, 0x01);
	ret = fimc_is_factory_ois_poll(ois->client, MOT_BU24218_REG_STATUS, 0x01);
	check_result(ret);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_MODE, 0x00);
	ret = fimc_is_factory_ois_poll(ois->client, MOT_BU24218_REG_STATUS, 0x01);
	check_result(ret);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_MODE, 0x04);
	ret = fimc_is_factory_ois_poll(ois->client, MOT_BU24218_REG_STATUS, 0x01);
	check_result(ret);
	usleep_range(500, 500);

	// 2. set max postion
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_XCH,     (uint8_t) MOT_BU24218_VCM_DRV_MAX_HI);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_XCH + 1, (uint8_t) MOT_BU24218_VCM_DRV_MAX_LO);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_YCH,     (uint8_t) MOT_BU24218_VCM_DRV_MAX_HI);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_YCH + 1, (uint8_t) MOT_BU24218_VCM_DRV_MAX_LO);
	usleep_range(200, 200);

	// 3. check status reg
	ret = fimc_is_factory_ois_poll(ois->client, MOT_BU24218_REG_STATUS, 0x01);
	check_result(ret);

	// 4. read lens position
	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_LENS_XCH, &data[0]);
	check_result(ret);
	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_LENS_YCH, &data[1]);
	check_result(ret);

	// 5. set min postion
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_XCH,     (uint8_t) MOT_BU24218_VCM_DRV_MIN_HI);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_XCH + 1, (uint8_t) MOT_BU24218_VCM_DRV_MIN_LO);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_YCH,     (uint8_t) MOT_BU24218_VCM_DRV_MIN_HI);
	fimc_is_ois_write(ois->client, MOT_BU24218_REG_POS_YCH + 1, (uint8_t) MOT_BU24218_VCM_DRV_MIN_LO);
	usleep_range(200, 200);

	// 6. check status reg
	ret = fimc_is_factory_ois_poll(ois->client, MOT_BU24218_REG_STATUS, 0x01);
	check_result(ret);

	usleep_range(200, 200);
	// 7. read lens position
	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_LENS_XCH, &data[2]);
	check_result(ret);
	ret = fimc_is_ois_read(ois->client, MOT_BU24218_REG_LENS_YCH, &data[3]);
	check_result(ret);

	info("%s: actual pos:(%d, %d, %d, %d)", __func__,
		data[0], data[2], data[1], data[3]);
	// follow the format, Xmax, Xmin, Ymax, Ymin
	*result = data[0] | (data[2] << 8) | (data[1] << 16) | (data[3] << 24);

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
