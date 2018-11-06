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
#include "fimc-is-sensor-eeprom-gm1.h"
#include "fimc-is-sensor-eeprom.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-core.h"

#define SENSOR_EEPROM_NAME "GM1SP"

int fimc_is_eeprom_gm1_check_all_crc(struct v4l2_subdev *subdev)
{
	int ret =0;
	int flags = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	/* check crc to information data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_info, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}

#if 0
	/* check crc to AWB data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_awb, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}
	/* check crc to AF data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_af, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}
	/* check crc to LSC data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_lsc, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}
	/* check crc to PDAF data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_pdaf, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}
	/* check crc to OIS data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_ois, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}
	/* check crc to AE Sync data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_ae, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}

	/* check crc to Dual camera data */
	ret = CALL_EEPROMOPS(eeprom, eeprom_check_dual, subdev);
	if (ret) {
		err("%s(): EEPROM information section check fail(%d)\n", __func__, ret);
		flags++;
	}
#endif

	return ret;
}

static int fimc_is_eeprom_gm1_check_info(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_INFO_CRC_START], EEPROM_INFO_CRC_SIZE);
	info("INFO CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_INFO_CRC_START], eeprom->data[EEPROM_INFO_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_awb(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_AWB_CRC_START], EEPROM_AWB_CRC_SIZE);
	info("AWB CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_AWB_CRC_START], eeprom->data[EEPROM_AWB_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_af(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_AF_CRC_START], EEPROM_AF_CRC_SIZE);
	info("AF CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_AF_CRC_START], eeprom->data[EEPROM_AF_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_ae(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_AE_CRC_START], EEPROM_AE_CRC_SIZE);
	info("AE CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_AE_CRC_START], eeprom->data[EEPROM_AE_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_lsc(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_LSC_CRC_START], EEPROM_LSC_CRC_SIZE);
	info("LSC CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_LSC_CRC_START], eeprom->data[EEPROM_LSC_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_ois(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_OIS_CRC_START], EEPROM_OIS_CRC_SIZE);
	info("OIS CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_OIS_CRC_START], eeprom->data[EEPROM_OIS_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_pdaf(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_PDAF_CRC_START], EEPROM_PDAF_CRC_SIZE);
	info("PDAF CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_PDAF_CRC_START], eeprom->data[EEPROM_PDAF_CRC_END]);

	return ret;
}

static int fimc_is_eeprom_gm1_check_dual(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u16 crc16 = 0;
	struct fimc_is_eeprom *eeprom = NULL;

	FIMC_BUG(!subdev);

	eeprom = (struct fimc_is_eeprom *)v4l2_get_subdevdata(subdev);

	FIMC_BUG(!eeprom);

	crc16 = fimc_is_sensor_eeprom_check_crc(&eeprom->data[EEPROM_DUAL_CRC_START], EEPROM_DUAL_CRC_SIZE);
	info("DUAL CRC16: %x, cal_buffer CRC: %x%x\n", crc16,
			eeprom->data[EEPROM_DUAL_CRC_START], eeprom->data[EEPROM_DUAL_CRC_END]);

	return ret;
}

int fimc_is_sensor_get_eeprom_data(struct v4l2_subdev *subdev)
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
		ret = fimc_is_eeprom_module_read(client, EEPROM_DATA_PATH, eeprom->data, EEPROM_DATA_SIZE);
		if (ret < 0) {
			err("%s(): eeprom i2c read failed(%d)\n", __func__, ret);
			return ret;
		}

		ret = fimc_is_eeprom_file_write(EEPROM_DATA_PATH, (void *)eeprom->data, EEPROM_DATA_SIZE);
		if (ret < 0) {
			err("%s(), eeprom file write fail(%d)\n", __func__, ret);
			return ret;
		}
	}

	ret = CALL_EEPROMOPS(eeprom, eeprom_check_all_crc, subdev);
	if (ret) {
		err("%s(): eeprom data invalid(%d)\n", __func__, ret);
		return ret;
	}

	return ret;
}

static struct fimc_is_eeprom_ops sensor_eeprom_ops = {
	.eeprom_read = fimc_is_sensor_get_eeprom_data,
	.eeprom_check_all_crc = fimc_is_eeprom_gm1_check_all_crc,
	.eeprom_check_info = fimc_is_eeprom_gm1_check_info,
	.eeprom_check_awb = fimc_is_eeprom_gm1_check_awb,
	.eeprom_check_af = fimc_is_eeprom_gm1_check_af,
	.eeprom_check_ae = fimc_is_eeprom_gm1_check_ae,
	.eeprom_check_lsc = fimc_is_eeprom_gm1_check_lsc,
	.eeprom_check_ois = fimc_is_eeprom_gm1_check_ois,
	.eeprom_check_pdaf = fimc_is_eeprom_gm1_check_pdaf,
	.eeprom_check_dual = fimc_is_eeprom_gm1_check_dual,
};

static int sensor_eeprom_gm1_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
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

	eeprom->id = EEPROM_NAME_GM1;
	eeprom->subdev = subdev_eeprom;
	eeprom->device = sensor_id;
	eeprom->client = client;
	eeprom->i2c_lock = NULL;
	eeprom->total_size = EEPROM_DATA_SIZE;
	eeprom->eeprom_ops = &sensor_eeprom_ops;

	device->subdev_eeprom = subdev_eeprom;
	device->eeprom = eeprom;

	v4l2_set_subdevdata(subdev_eeprom, eeprom);
	v4l2_set_subdev_hostdata(subdev_eeprom, device);

	snprintf(subdev_eeprom->name, V4L2_SUBDEV_NAME_SIZE, "eeprom-subdev.%d", eeprom->id);

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static const struct of_device_id sensor_eeprom_gm1sp_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-sensor-eeprom-gm1",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_eeprom_gm1sp_match);

static const struct i2c_device_id sensor_eeprom_gm1sp_idt[] = {
	{ SENSOR_EEPROM_NAME, 0 },
	{},
};

static struct i2c_driver sensor_eeprom_gm1sp_driver = {
	.probe  = sensor_eeprom_gm1_probe,
	.driver = {
		.name	= SENSOR_EEPROM_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_eeprom_gm1sp_match,
		.suppress_bind_attrs = true,
	},
	.id_table = sensor_eeprom_gm1sp_idt
};

static int __init sensor_eeprom_gm1sp_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_eeprom_gm1sp_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_eeprom_gm1sp_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_eeprom_gm1sp_init);
