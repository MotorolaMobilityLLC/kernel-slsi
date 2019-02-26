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
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <exynos-fimc-is-sensor.h>
#include "fimc-is-sensor-eeprom-16885c.h"
#include "fimc-is-sensor-eeprom.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-core.h"

#define SENSOR_EEPROM_NAME "16885C"

int fimc_is_eeprom_16885c_check_all_crc(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_module_enum *module;
	struct fimc_is_eeprom *eeprom = NULL;
	struct fimc_is_device_sensor *sensor = NULL;

	FIMC_BUG(!subdev);

	module = (struct fimc_is_module_enum *)v4l2_get_subdev_hostdata(subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);
	FIMC_BUG(!module);

	sensor = (struct fimc_is_device_sensor *)v4l2_get_subdev_hostdata(module->subdev);
	if (!sensor) {
		err("device sensor is NULL");
		ret = -ENODEV;
		return ret;
	}

	/* Check CRC to Address cal data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_address, subdev);
	if (ret) {
		err("%s(): 16885C EEPROM Address section CRC check fail(%d)", __func__, ret);

		/* All calibration data is zero set only Address section is invalid CRC */
		fimc_is_eeprom_cal_data_set(eeprom->data, "all",
				EEPROM_ADD_CRC_SEC, EEPROM_DATA_SIZE, 0xff);

		/*Set all cal_status to ERROR if Address cal data invalid*/
		for (int i = 0; i < CAMERA_CRC_INDEX_MAX; i++)
		    sensor->cal_status[i] = CRC_ERROR;

		return ret;
	} else
		info("16885C EEPROM Address section CRC check success\n");

	/* Check CRC to Information cal data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_info, subdev);
	if (ret) {
		err("%s(): 16885C EEPROM Information CRC section check fail(%d)", __func__, ret);

		/* All calibration data is 0xff set but exception Address section */
		fimc_is_eeprom_cal_data_set(eeprom->data, "Information - End",
				EEPROM_INFO_CRC_SEC, EEPROM_ADD_CAL_SIZE, 0xff);

		sensor->cal_status[CAMERA_CRC_INDEX_MNF] = CRC_ERROR;

	} else {
		info("16885C EEPROM Informaion section CRC check success\n");

		sensor->cal_status[CAMERA_CRC_INDEX_MNF] = CRC_NO_ERROR;
	}

	/* Check CRC to AWB cal data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_awb, subdev);
	if (ret) {
		err("%s(): 16885C EEPROM AWB section CRC check fail(%d)", __func__, ret);

		fimc_is_eeprom_cal_data_set(eeprom->data, "AWB",
				EEPROM_AWB_CRC_SEC, EEPROM_AWB_CAL_SIZE, 0xff);

		sensor->cal_status[CAMERA_CRC_INDEX_AWB] = CRC_ERROR;

	} else {
		info("16885C EEPROM AWB section CRC check success\n");

		sensor->cal_status[CAMERA_CRC_INDEX_AWB] = CRC_NO_ERROR;

		ret = fimc_is_sensor_eeprom_check_awb_ratio(&eeprom->data[EEPROM_AWB_UNIT_OFFSET],
			&eeprom->data[EEPROM_AWB_GOLDEN_OFFSET],&eeprom->data[EEPROM_AWB_LIMIT_OFFSET]);
		if (ret) {
			err("%s(): 16885C EEPROM AWB ratio out of limit(%d)", __func__, ret);

			sensor->cal_status[CAMERA_CRC_INDEX_AWB] = LIMIT_FAILURE;
		}
	}

	/* Check CRC to LSC cal data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_lsc, subdev);
	if (ret) {
		err("%s(): 16885C EEPROM LSC section CRC check fail(%d)", __func__, ret);

		fimc_is_eeprom_cal_data_set(eeprom->data, "LSC",
				EEPROM_LSC_CRC_SEC, EEPROM_LSC_CAL_SIZE, 0xff);

	} else
		info("16885C EEPROM LSC section CRC check success\n");

	/* Check CRC to SFR cal data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_sfr, subdev);
	if (ret) {
		err("%s(): EEPROM SFR section CRC check fail(%d)", __func__, ret);

		fimc_is_eeprom_cal_data_set(eeprom->data, "SFR",
				EEPROM_SFR_CRC_SEC, EEPROM_SFR_CAL_SIZE, 0xff);

	} else
		info("16885C EEPROM SFR section CRC check success\n");

	/* Write file to serial number of Information calibration data */
	ret = fimc_is_eeprom_file_write(EEPROM_SERIAL_NUM_DATA_PATH,
			(void *)&eeprom->data[EEPROM_INFO_SERIAL_NUM_START], EEPROM_INFO_SERIAL_NUM_SIZE);
	if (ret < 0)
		err("%s(), DUAL cal file write fail(%d)", __func__, ret);

	return ret;
}

static int fimc_is_eeprom_16885c_check_address(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc_value = ((eeprom->data[EEPROM_ADD_CRC_SEC] << 8) | (eeprom->data[EEPROM_ADD_CRC_FST]));

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_ADD_CRC_CHK_START], EEPROM_ADD_CRC_CHK_SIZE);
	if (crc_value != crc16) {
		err("Error to ADD CRC16: 0x%x, cal_buffer CRC: 0x%x", crc16, crc_value);

		ret = -EINVAL;
	} else
		info("ADD CRC16: 0x%x, cal_buffer CRC: 0x%x\n", crc16, crc_value);

	return ret;
}

static int fimc_is_eeprom_16885c_check_info(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc_value = ((eeprom->data[EEPROM_INFO_CRC_SEC] << 8) | (eeprom->data[EEPROM_INFO_CRC_FST]));

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_INFO_CRC_CHK_START], EEPROM_INFO_CRC_CHK_SIZE);
	if (crc_value != crc16)
		err("Error to INFO CRC16: 0x%x, cal_buffer CRC: 0x%x", crc16, crc_value);
	else
		info("INFO CRC16: 0x%x, cal_buffer CRC: 0x%x\n", crc16, crc_value);

	return ret;
}

static int fimc_is_eeprom_16885c_check_awb(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc_value = ((eeprom->data[EEPROM_AWB_CRC_SEC] << 8) | (eeprom->data[EEPROM_AWB_CRC_FST]));

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_AWB_CRC_CHK_START], EEPROM_AWB_CRC_CHK_SIZE);
	if (crc_value != crc16)
		err("Error to AWB CRC16: 0x%x, cal_buffer CRC: 0x%x", crc16, crc_value);
	else
		info("AWB CRC16: 0x%x, cal_buffer CRC: 0x%x\n", crc16, crc_value);

	return ret;
}

static int fimc_is_eeprom_16885c_check_lsc(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc_value = ((eeprom->data[EEPROM_LSC_CRC_SEC] << 8) | (eeprom->data[EEPROM_LSC_CRC_FST]));

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_LSC_CRC_CHK_START], EEPROM_LSC_CRC_CHK_SIZE);
	if (crc_value != crc16)
		err("Error to LSC CRC16: 0x%x, cal_buffer CRC: 0x%x", crc16, crc_value);
	else
		info("LSC CRC16: 0x%x, cal_buffer CRC: 0x%x\n", crc16, crc_value);

	return ret;
}

static int fimc_is_eeprom_16885c_check_sfr(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc_value = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc_value = ((eeprom->data[EEPROM_SFR_CRC_SEC] << 8) | (eeprom->data[EEPROM_SFR_CRC_FST]));

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_SFR_CRC_CHK_START], EEPROM_SFR_CRC_CHK_SIZE);
	if (crc_value != crc16)
		err("Error to SFR CRC16: 0x%x, cal_buffer CRC: 0x%x", crc16, crc_value);
	else
		info("SFR CRC16: 0x%x, cal_buffer CRC: 0x%x\n", crc16, crc_value);

	return ret;
}

int fimc_is_eeprom_16885c_get_cal_data(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_eeprom *eeprom;
	struct i2c_client *client;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	client = eeprom->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	/*
	 * If already read at EEPROM data in module
	 * don't again read at EEPROM but there isn't file or
	 * data is NULL read EEPROM data
	 */
	ret = fimc_is_eeprom_file_read(EEPROM_DATA_PATH, (void *)eeprom->data, EEPROM_DATA_SIZE);
	if (ret) {
		/* I2C read to Sensor EEPROM cal data */
		ret = fimc_is_eeprom_module_read(client, EEPROM_ADD_CRC_FST, eeprom->data, EEPROM_DATA_SIZE);
		if (ret < 0) {
			err("%s(): eeprom i2c read failed(%d)\n", __func__, ret);
			return ret;
		}

		/* CRC check to each section cal data */
		ret = CALL_EEPROMOPS(eeprom, eeprom_check_all_crc, subdev);
		if (ret < 0)
			err("%s(): eeprom data invalid(%d)\n", __func__, ret);

		/* Write file to Cal data */
		ret = fimc_is_eeprom_file_write(EEPROM_DATA_PATH, (void *)eeprom->data, EEPROM_DATA_SIZE);
		if (ret < 0) {
			err("%s(), eeprom file write fail(%d)\n", __func__, ret);
			return ret;
		}
	} else {
		/* CRC check to each section cal data */
		ret = CALL_EEPROMOPS(eeprom, eeprom_check_all_crc, subdev);
		if (ret < 0)
			err("%s(): eeprom data invalid(%d)\n", __func__, ret);
	}

	return ret;
}

static struct fimc_is_eeprom_ops sensor_eeprom_ops = {
	.eeprom_read = fimc_is_eeprom_16885c_get_cal_data,
	.eeprom_check_all_crc = fimc_is_eeprom_16885c_check_all_crc,
	.eeprom_check_address = fimc_is_eeprom_16885c_check_address,
	.eeprom_check_info = fimc_is_eeprom_16885c_check_info,
	.eeprom_check_awb = fimc_is_eeprom_16885c_check_awb,
	.eeprom_check_lsc = fimc_is_eeprom_16885c_check_lsc,
	.eeprom_check_sfr = fimc_is_eeprom_16885c_check_sfr,
};

static int sensor_eeprom_16885c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0, i;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_eeprom = NULL;
	struct fimc_is_eeprom *eeprom = NULL;
	struct fimc_is_device_sensor *device;
	struct device *dev;
	struct device_node *dnode;
	u32 sensor_id = 0;

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
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	device = &core->sensor[sensor_id];
	if (!device) {
		err("sensor device is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	eeprom = kzalloc(sizeof(struct fimc_is_eeprom), GFP_KERNEL);
	if (!eeprom) {
		err("eeprom is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_eeprom = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_eeprom) {
		probe_err("subdev_cis NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	eeprom->data = kzalloc(EEPROM_DATA_SIZE, GFP_KERNEL);
	if (!eeprom->data) {
		err("data is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	eeprom->id = EEPROM_NAME_16885C;
	eeprom->subdev = subdev_eeprom;
	eeprom->device = sensor_id;
	eeprom->client = client;
	eeprom->i2c_lock = NULL;
	eeprom->total_size = EEPROM_DATA_SIZE;
	eeprom->eeprom_ops = &sensor_eeprom_ops;

	for (i = 0; i < CAMERA_CRC_INDEX_MAX; i++)
		device->cal_status[i] = CRC_NO_ERROR;

	device->subdev_eeprom = subdev_eeprom;
	device->eeprom = eeprom;

	v4l2_set_subdevdata(subdev_eeprom, eeprom);
	v4l2_set_subdev_hostdata(subdev_eeprom, device);

	snprintf(subdev_eeprom->name, V4L2_SUBDEV_NAME_SIZE, "eeprom-subdev.%d", eeprom->id);

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static const struct of_device_id sensor_eeprom_16885c_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-sensor-eeprom-16885c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_eeprom_16885c_match);

static const struct i2c_device_id sensor_eeprom_16885c_idt[] = {
	{ SENSOR_EEPROM_NAME, 0 },
	{},
};

static struct i2c_driver sensor_eeprom_16885c_driver = {
	.probe  = sensor_eeprom_16885c_probe,
	.driver = {
		.name	= SENSOR_EEPROM_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_eeprom_16885c_match,
		.suppress_bind_attrs = true,
	},
	.id_table = sensor_eeprom_16885c_idt
};

static int __init sensor_eeprom_16885c_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_eeprom_16885c_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_eeprom_16885c_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_eeprom_16885c_init);
