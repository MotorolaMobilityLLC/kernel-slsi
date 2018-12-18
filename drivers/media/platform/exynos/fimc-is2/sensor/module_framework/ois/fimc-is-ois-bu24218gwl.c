/*
 * Copyright (C) 2018 Motorola Mobility LLC.
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

#include "fimc-is-core.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-helper-ois-i2c.h"
#include "fimc-is-ois-bu24218gwl.h"
#include "fimc-is-ois.h"

#define OIS_NAME "OIS_ROHM_BU24218GWL"
#define OIS_FW_1_NAME		"bu24218_Rev1.2_S_data1.bin"
#define OIS_FW_2_NAME		"bu24218_Rev1.2_S_data2.bin"
#define OIS_FW_NUM		2
#define OIS_FW_ADDR_1		0x0000
#define OIS_FW_ADDR_2		0x1C00
#define OIS_FW_CHECK_SUM	0x02D806

#define OIS_CAL_DATA_PATH		"/data/camera/gm1_eeprom_data.bin"
#define OIS_CAL_DATA_PATH_DEFAULT	"/vendor/firmware/bu24218_cal_data_default.bin"
#define OIS_CAL_DATA_CRC_OFFSET	0x1DB0
#define OIS_CAL_DATA_CRC_SIZE		2
#define OIS_CAL_DATA_OFFSET		0x1DB4
#define OIS_CAL_DATA_SIZE		0x4C
#define OIS_CAL_DATA_OFFSET_DEFAULT	0
#define OIS_CAL_DATA_SIZE_DEFAULT	0x28
#define OIS_CAL_ADDR			0x1DC0
#define OIS_CAL_ACTUAL_DL_SIZE		0x28

//#define OIS_DEBUG

static u8 ois_fw_data[OIS_FW_NUM][OIS_FW_SIZE] = {{0},};
static int ois_fw_data_size[OIS_FW_NUM] = {0,};
static u8 ois_cal_data[OIS_CAL_DATA_SIZE] = {0,};
static int ois_cal_data_size = 0;
static int ois_previous_mode = 0;

static const struct v4l2_subdev_ops subdev_ops;

#ifdef CONFIG_OIS_DIRECT_FW_CONTROL
int fimc_is_ois_fw_ver_copy(struct fimc_is_ois *ois, u8 *buf, long size)
{
	int ret = 0;
	static int load_cnt = 0;

	FIMC_BUG(!ois);

	memcpy(ois_fw_data[load_cnt], (void *)buf, size);
	ois_fw_data_size[load_cnt++] = size;
	info("%s copy size:%d bytes", __func__, size);

	return ret;
}

int fimc_is_ois_check_crc(char *data, size_t size)
{
	char *tmp = data;
	u32 crc[16];
	int i, j;
	u16 crc16 = 0;

	memset(crc, 0, sizeof(crc));
	for (i = 0; i < size; i++) {
		for (j = 7; j >= 0; j--) {
			/* isolate the bit in the byte */
			u32 doInvert = *tmp & (1 << j);

			// shift the bit to LSB in the byte
			doInvert = doInvert >> j;

			// XOR required?
			doInvert = doInvert ^ crc[15];

			crc[15] = crc[14] ^ doInvert;
			crc[14] = crc[13];
			crc[13] = crc[12];
			crc[12] = crc[11];
			crc[11] = crc[10];
			crc[10] = crc[9];
			crc[9] = crc[8];
			crc[8] = crc[7];
			crc[7] = crc[6];
			crc[6] = crc[5];
			crc[5] = crc[4];
			crc[4] = crc[3];
			crc[3] = crc[2];
			crc[2] = crc[1] ^ doInvert;
			crc[1] = crc[0];
			crc[0] = doInvert;
		}
		tmp++;
	}

	/* convert bits to CRC word */
	for (i = 0; i < 16; i++)
		crc16 = crc16 + (crc[i] << i);

	return crc16;
}

int fimc_is_ois_cal_open(struct fimc_is_ois *ois, char *name, int offset,int size, int crc_enable)
{
	int ret = 0;
	long fsize, nread;
	mm_segment_t old_fs;
	struct file *fp;
	u8 *buf = NULL;
	u16 crc_value = 0;
	u16 crc16 = 0;
	int i = 0;

	FIMC_BUG(!ois);

	info("%s: E", __func__);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(name, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(fp)) {
		ret = PTR_ERR(fp);
		err("filp_open(%s) fail(%d)!!\n", name, ret);
		goto p_err;
	}

	buf = vmalloc(size);
	if (!buf) {
		err("failed to allocate memory");
		ret = -ENOMEM;
		goto p_err;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;
	if (fsize < offset + size) {
		err("ois cal data not exist");
		ret = -EIO;
		goto p_err;
	}

	if (crc_enable) {
		fp->f_pos = OIS_CAL_DATA_CRC_OFFSET;
		err("ois f_pos set offset %x", fp->f_pos);
		nread = vfs_read(fp, (char __user *)buf, OIS_CAL_DATA_CRC_SIZE , &fp->f_pos);
		if (nread != OIS_CAL_DATA_CRC_SIZE) {
			err("failed to read ois cal crc data from file, (%ld) Bytes", nread);
			ret = -EIO;
			goto p_err;
		}
		crc_value = ((buf[0] << 8) | (buf[1]));
	}

	fp->f_pos = offset;
	err("ois f_pos set offset %x", fp->f_pos);
	nread = vfs_read(fp, (char __user *)buf, size , &fp->f_pos);
	if (nread != size) {
		err("failed to read ois cal data from file, (%ld) Bytes", nread);
		ret = -EIO;
		goto p_err;
	}

	for(i = 0; i < size/4; i++) {
		info("ois cal data (%d): 0x%0x,%0x,%0x,%0x", i, buf[4*i+0], buf[4*i+1], buf[4*i+2], buf[4*i+3]);
	}

	if (crc_enable) {
		crc16 = fimc_is_ois_check_crc(buf, OIS_CAL_DATA_SIZE);
		if (crc_value != crc16) {
			err("Error to OIS CRC16: 0x%x, cal_buffer CRC: 0x%x", crc16, crc_value);
			ret = -EIO;
			goto p_err;
		}
		else {
			info("OIS CRC16: 0x%x, cal_buffer CRC: 0x%x\n", crc16, crc_value);
		}
	}
	/* Cal data save */
	memcpy(ois_cal_data, (void *)buf, OIS_CAL_ACTUAL_DL_SIZE);
	ois_cal_data_size = OIS_CAL_ACTUAL_DL_SIZE;
	info("%s cal data copy size:%d bytes", __func__, OIS_CAL_ACTUAL_DL_SIZE);

p_err:
	if (buf)
		vfree(buf);

	if (!IS_ERR_OR_NULL(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);

	info("%s: X", __func__);
	return ret;
}

int fimc_is_ois_fw_download(struct fimc_is_ois *ois)
{
	int ret = 0;
	int retry = 3;
	u8 check_sum[4] = {0};
	int sum = 0;
	u8 send_data[256];
	int position = 0;
	int i, quotienti, remainder;
	u8 ois_status = 0;
	int fw_num = 0;
	int fw_download_start_addr[OIS_FW_NUM] = {OIS_FW_ADDR_1, OIS_FW_ADDR_2};

	FIMC_BUG(!ois);

	info("%s: E", __func__);

	/* Step 1 Enable download*/
	I2C_MUTEX_LOCK(ois->i2c_lock);
	ret = fimc_is_ois_write(ois->client, 0xF010, 0x00);
	if (ret < 0) {
		err("ois start download write is fail");
		ret = 0;
		I2C_MUTEX_UNLOCK(ois->i2c_lock);
		goto p_err;
	}
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	/* wait over 200us */
	usleep_range(200,200);
	ret = fimc_is_ois_read(ois->client, 0x6024, &ois_status);
	info("ois status:0x%x", ois_status);

	/* Step 2 Download FW*/
	for(fw_num = 0; fw_num < OIS_FW_NUM; fw_num++) {
		quotienti = 0;
		remainder = 0;
		position = 0;
		quotienti = ois_fw_data_size[fw_num]/FW_TRANS_SIZE;
		remainder = ois_fw_data_size[fw_num]%FW_TRANS_SIZE;

		for(i = 0; i < quotienti ; i++) {
			memcpy(send_data, &ois_fw_data[fw_num][position], (size_t)FW_TRANS_SIZE);
			I2C_MUTEX_LOCK(ois->i2c_lock);
			ret = fimc_is_ois_write_multi(ois->client, fw_download_start_addr[fw_num]+position,
				send_data, FW_TRANS_SIZE + 2);
			if (ret < 0) {
				err("ois fw download is fail");
				ret = 0;
				I2C_MUTEX_UNLOCK(ois->i2c_lock);
				goto p_err;
			}
			I2C_MUTEX_UNLOCK(ois->i2c_lock);

			position += FW_TRANS_SIZE;
		}
		if(remainder) {
			I2C_MUTEX_LOCK(ois->i2c_lock);
			memcpy(send_data, &ois_fw_data[fw_num][position], (size_t)remainder);
			ret = fimc_is_ois_write_multi(ois->client, fw_download_start_addr[fw_num]+position,
				send_data, remainder + 2);
			I2C_MUTEX_UNLOCK(ois->i2c_lock);
		}
		if (ret < 0) {
			err("ois fw download is fail");
			ret = 0;
			goto p_err;
		}
		info("ois fw %d download size:%d", fw_num, position + remainder);
	}

	/* Step 3 Sum Check*/
	I2C_MUTEX_LOCK(ois->i2c_lock);
	ret = fimc_is_ois_read_multi(ois->client, 0xF008, check_sum, 4);
	if (ret < 0) {
		err("ois read check sum fail");
		ret = 0;
		I2C_MUTEX_UNLOCK(ois->i2c_lock);
		goto p_err;
	}
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	sum = ((check_sum[0]<<24) | (check_sum[1]<<16) | (check_sum[2]<<8) | (check_sum[3]));
	info("ois check sum value:0x%0x, expected value:0x%0x", sum, OIS_FW_CHECK_SUM);
	if (OIS_FW_CHECK_SUM != sum) {
		err("ois check sum fail, force return");
		ret = 0;
		goto p_err;
	}

	/* Step 4 Calibration data download */
	info("ois download cal data");
	I2C_MUTEX_LOCK(ois->i2c_lock);
	ret = fimc_is_ois_write_multi(ois->client, OIS_CAL_ADDR, ois_cal_data, ois_cal_data_size + 2);
	if (ret < 0) {
		err("ois cal data download is fail");
		ret = 0;
		I2C_MUTEX_UNLOCK(ois->i2c_lock);
		goto p_err;
	}
	info("ois cal data download size :%d", ois_cal_data_size);
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	/* Step 5 OIS download complete */
	I2C_MUTEX_LOCK(ois->i2c_lock);
	ret = fimc_is_ois_write(ois->client, 0xF006, 0x00);
	if (ret < 0) {
		err("ois write download complete is fail");
		ret = 0;
		I2C_MUTEX_UNLOCK(ois->i2c_lock);
		goto p_err;
	}
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	/* wait 18ms */
	usleep_range(18000,18000);

	/* OIS status */
	ret = fimc_is_ois_read(ois->client, 0x6024, &ois_status);
	info("ois status after D/L complete :0x%x", ois_status);

	while ((ois_status == 0) && (retry-- > 0)) {
		usleep_range(4000, 4000);
		I2C_MUTEX_LOCK(ois->i2c_lock);
		ret = fimc_is_ois_read(ois->client, 0x6024, &ois_status);
		I2C_MUTEX_UNLOCK(ois->i2c_lock);
		info("ois status :0x%x", ois_status);
	}
	if (ois_status != 1) {
		err("ois_status is 0,force return error");
		ret = 0;
		goto p_err;
	}

	info("%s: ois fw download success\n", __func__);
p_err:

	return ret;
}

int fimc_is_ois_fw_update(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_ois *ois = NULL;
	static int is_first_load = 1;

	FIMC_BUG(!subdev);

	info("%s: E", __func__);

	ois = (struct fimc_is_ois *)v4l2_get_subdevdata(subdev);
	if (!ois) {
		err("ois is NULL");
		ret = -EINVAL;
		return ret;
	}

	/* OIS Firmware load*/
	if (1 == is_first_load) {
		ret = fimc_is_ois_fw_open(ois, OIS_FW_1_NAME);
		if (ret < 0) {
			err("OIS %s load is fail\n", OIS_FW_1_NAME);
			return 0;
		}
		ret = fimc_is_ois_fw_open(ois, OIS_FW_2_NAME);
		if (ret < 0) {
			err("OIS %s load is fail\n", OIS_FW_2_NAME);
			return 0;
		}
		ret = fimc_is_ois_cal_open(ois, OIS_CAL_DATA_PATH, OIS_CAL_DATA_OFFSET, OIS_CAL_DATA_SIZE, 1);
//		if (ret < 0) {
			info(" switch to load default OIS Cal Data %s \n", OIS_CAL_DATA_PATH_DEFAULT);
			ret = fimc_is_ois_cal_open(ois, OIS_CAL_DATA_PATH_DEFAULT, OIS_CAL_DATA_OFFSET_DEFAULT,
				OIS_CAL_DATA_SIZE_DEFAULT, 0);
			if (ret < 0) {
				err("OIS %s load is fail\n", OIS_CAL_DATA_PATH_DEFAULT);
				return 0;
			}
//		}
		is_first_load = 0;
	}

	/* OIS Firmware download */
	ret = fimc_is_ois_fw_download(ois);
	if (ret < 0) {
		err("OIS Firmware download fail");
		return 0;
	}

	info("%s: X", __func__);
	return ret;
}

#endif

int fimc_is_set_ois_mode(struct v4l2_subdev *subdev, int mode)
{
	int ret = 0;
	struct fimc_is_ois *ois;
	struct i2c_client *client = NULL;
#ifdef OIS_DEBUG
	u8 ois_mode = 0;
	u8 ois_gyro_data[2] = {0};
#endif

	FIMC_BUG(!subdev);

	ois = (struct fimc_is_ois *)v4l2_get_subdevdata(subdev);
	if (!ois) {
		err("ois is NULL");
		ret = -EINVAL;
		return ret;
	}

	client = ois->client;
	if (!client) {
		err("client is NULL");
		ret = -EINVAL;
		return ret;
	}

	I2C_MUTEX_LOCK(ois->i2c_lock);

#ifdef OIS_DEBUG
	ret != fimc_is_ois_read(ois->client, 0x6021, &ois_mode);
	info("last ois mode is 0x%x", ois_mode);
#endif

	if (ois_previous_mode == mode) {
#ifdef OIS_DEBUG
		info("skip set same ois mode:%d", mode);
#endif
		goto p_err;
	} else {
		info("set ois mode:%d", mode);
		switch(mode) {
		case OPTICAL_STABILIZATION_MODE_STILL:
			fimc_is_ois_write(ois->client, 0x6020, 0x01);
			usleep_range(100000, 100000);
			fimc_is_ois_write(client, 0x6021, 0x7b); // ZSL
			fimc_is_ois_write(client, 0x6020, 0x02);
			break;
		case OPTICAL_STABILIZATION_MODE_STILL_ZOOM:
			fimc_is_ois_write(ois->client, 0x6020, 0x01);
			usleep_range(100000, 100000);
			fimc_is_ois_write(client, 0x6021, 0x03); // Exposure/Shake
			fimc_is_ois_write(client, 0x6020, 0x02);
			break;
		case OPTICAL_STABILIZATION_MODE_VIDEO:
			fimc_is_ois_write(ois->client, 0x6020, 0x01);
			usleep_range(100000, 100000);
			fimc_is_ois_write(client, 0x6021, 0x61);
			fimc_is_ois_write(client, 0x6020, 0x02);
			break;
		case OPTICAL_STABILIZATION_MODE_CENTERING:
			fimc_is_ois_write(ois->client, 0x6020, 0x01);
			usleep_range(100000, 100000);
			fimc_is_ois_write(client, 0x6021, 0x63);
			fimc_is_ois_write(client, 0x6020, 0x02); // Servo ON
			break;
		default:
			err("%s: invalid ois_mode value(%d)\n", __func__, mode);
			break;
		}
		ois_previous_mode = mode;
	}

#ifdef OIS_DEBUG
	ret != fimc_is_ois_read_multi(ois->client, 0x6040, ois_gyro_data, 2);
	info("ois Gyro output1 is 0x%x%x", ois_gyro_data[0], ois_gyro_data[1]);
	ret != fimc_is_ois_read_multi(ois->client, 0x6042, ois_gyro_data, 2);
	info("ois Gyro output2 is 0x%x%x", ois_gyro_data[0], ois_gyro_data[1]);
#endif
p_err:
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	return ret;
}

int fimc_is_ois_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	int retry = 3;
	u8 ois_status = 0;
	struct fimc_is_ois *ois = NULL;

	FIMC_BUG(!subdev);

	info("%s: E", __func__);

	ois = (struct fimc_is_ois *)v4l2_get_subdevdata(subdev);
	if(!ois) {
		err("%s, ois subdev is NULL", __func__);
		ret = -EINVAL;
		return ret;
	}

	I2C_MUTEX_LOCK(ois->i2c_lock);

	/* Servo ON for OIS */
	ret = fimc_is_ois_write(ois->client, 0x6020, 0x01);
	if (ret < 0) {
		err("ois servo on write is fail");
		ret = 0;
		goto p_err;
	}
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	/* wait 100ms */
	usleep_range(100000, 100000);

	I2C_MUTEX_LOCK(ois->i2c_lock);
	/* Gyro ON for OIS */
	ret |= fimc_is_ois_write(ois->client, 0x6023, 0x02);
	ret |= fimc_is_ois_write(ois->client, 0x602C, 0x76);
	ret != fimc_is_ois_write(ois->client, 0x602D, 0x02);
	ret != fimc_is_ois_write(ois->client, 0x602C, 0x44);
	ret != fimc_is_ois_write(ois->client, 0x602D, 0x02);
	ret != fimc_is_ois_write(ois->client, 0x602C, 0x45);
	ret != fimc_is_ois_write(ois->client, 0x602D, 0x58);
	I2C_MUTEX_UNLOCK(ois->i2c_lock);
	usleep_range(20000, 20000);
	I2C_MUTEX_LOCK(ois->i2c_lock);
	ret != fimc_is_ois_write(ois->client, 0x6023, 0x00);
	ret != fimc_is_ois_write(ois->client, 0x614F, 0x01);
	ret != fimc_is_ois_write(ois->client, 0x6021, 0x7B);
	usleep_range(300, 300);
	ret != fimc_is_ois_read(ois->client, 0x6024, &ois_status);
	info("ois status is 0x%x", ois_status);
	while ((ois_status == 0) && (retry-- > 0)) {
		usleep_range(3000, 3000);
		ret != fimc_is_ois_read(ois->client, 0x6024, &ois_status);
		info("retry ois status is 0x%x", ois_status);
	}
	if (ois_status != 1) {
		err("ois_status is 0, force return error");
		ret = 0;
		goto p_err;
	}
	ret != fimc_is_ois_write(ois->client, 0x6020, 0x02);
	if (ret < 0) {
		err("ois gyro on write is fail");
		ret = 0;
		goto p_err;
	}

	usleep_range(200, 200);
	ret != fimc_is_ois_read(ois->client, 0x6024, &ois_status);
	info("ois status after OIS ON is 0x%x", ois_status);
	while ((ois_status == 0) && (retry-- > 0)) {
		usleep_range(300, 300);
		ret != fimc_is_ois_read(ois->client, 0x6024, &ois_status);
		info("ois status after OIS ON is 0x%x", ois_status);
	}
	if (ois_status != 1) {
		err("ois_status is 0, force return error");
		ret = 0;
		goto p_err;
	}
	ois_previous_mode = OPTICAL_STABILIZATION_MODE_STILL;

#ifdef OIS_DEBUG
	ret != fimc_is_ois_read(ois->client, 0x6021, &ois_status);
	info("ois mode is 0x%x", ois_status);
	ret != fimc_is_ois_read(ois->client, 0x6023, &ois_status);
	info("ois Gyro mode is 0x%x", ois_status);
#endif

	info("%s: X", __func__);
p_err:
	I2C_MUTEX_UNLOCK(ois->i2c_lock);

	return ret;
}

static struct fimc_is_ois_ops ois_ops = {
	.ois_init = fimc_is_ois_init,
	.ois_set_mode = fimc_is_set_ois_mode,
#ifdef CONFIG_OIS_DIRECT_FW_CONTROL
	.ois_fw_update = fimc_is_ois_fw_update,
#endif
};

static int sensor_ois_bu24218gwl_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct fimc_is_core *core= NULL;
	struct v4l2_subdev *subdev_ois = NULL;
	struct fimc_is_device_sensor *device = NULL;
	struct fimc_is_device_sensor_peri *sensor_peri = NULL;
	struct fimc_is_ois *ois;
	struct device *dev;
	struct device_node *dnode;
	u32 sensor_id = 0;

	FIMC_BUG(!fimc_is_dev);
	FIMC_BUG(!client);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core device is not yet probed");
		ret = -EPROBE_DEFER;
		goto p_err;
	}

	dev = &client->dev;
	dnode = dev->of_node;

	ret = of_property_read_u32(dnode, "id", &sensor_id);
	if (ret) {
		err("id read is fail(%d)", ret);
		goto p_err;
	}

	probe_info("%s sensor_id %d\n", __func__, sensor_id);

	device = &core->sensor[sensor_id];
	if (!device) {
		err("sensor device is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	sensor_peri = find_peri_by_ois_id(device, OIS_NAME_ROHM_BU24218GWL);
	if (!sensor_peri) {
		probe_info("sensor peri is not yet probed");
		return -EPROBE_DEFER;
	}

	ois = kzalloc(sizeof(struct fimc_is_ois), GFP_KERNEL);
	if (!ois) {
		err("ois is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	sensor_peri->ois = ois;

	ois->ois_ops = &ois_ops;

	subdev_ois = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_ois) {
		err("subdev_ois is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	sensor_peri->subdev_ois = subdev_ois;

	ois->id = OIS_NAME_ROHM_BU24218GWL;
	ois->subdev = subdev_ois;
	ois->device = sensor_id;
	ois->client = client;
	device->subdev_ois = subdev_ois;
	device->ois = ois;

	v4l2_i2c_subdev_init(subdev_ois, client, &subdev_ops);
	v4l2_set_subdevdata(subdev_ois, ois);
	v4l2_set_subdev_hostdata(subdev_ois, device);

	set_bit(FIMC_IS_SENSOR_OIS_AVAILABLE, &sensor_peri->peri_state);
	snprintf(subdev_ois->name, V4L2_SUBDEV_NAME_SIZE, "ois->subdev.%d", ois->id);

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static int sensor_ois_bu24218gwl_remove(struct i2c_client *client)
{
	int ret = 0;
	return ret;
};

static const struct of_device_id sensor_ois_bu24218gwl_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-ois-bu24218gwl",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_ois_bu24218gwl_match);

static const struct i2c_device_id sensor_ois_bu24218gwl_idt[] = {
	{ OIS_NAME, 0 },
	{},
};

static struct i2c_driver sensor_ois_bu24218gwl_driver = {
	.probe	= sensor_ois_bu24218gwl_probe,
	.driver = {
		.name	= OIS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_ois_bu24218gwl_match
	},
	.id_table = sensor_ois_bu24218gwl_idt,
	.remove = sensor_ois_bu24218gwl_remove,
};

static int __init sensor_ois_bu24218gwl_init(void)
{
	int ret;

	ret = i2c_add_driver(&sensor_ois_bu24218gwl_driver);
	if (ret)
		err("failed to add %s driver: %d\n",
			sensor_ois_bu24218gwl_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_ois_bu24218gwl_init);
