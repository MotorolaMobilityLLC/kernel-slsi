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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "fimc-is-sensor-eeprom.h"
#include "fimc-is-helper-i2c.h"
#include "fimc-is-core.h"

void fimc_is_eeprom_cal_data_set(char *data, char *name,
		u32 addr, u32 size, u32 value)
{
	int i;

	/* value setting to (name) cal data section */
	for (i = addr; i < size; i++)
		data[i] = value;

	info("%s() Done: %s calibration data is %d set\n", __func__, name, value);
}

int fimc_is_eeprom_file_write(const char *file_name, const void *data,
		unsigned long size)
{
	int ret = 0;
	struct file *fp;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(file_name, O_WRONLY | O_CREAT | O_TRUNC | O_SYNC, 0666);
	if (IS_ERR_OR_NULL(fp)) {
		ret = PTR_ERR(fp);
		err("%s(): open file error(%s), error(%d)\n", __func__, file_name, ret);
		goto p_err;
	}

	ret = vfs_write(fp, (const char *)data, size, &fp->f_pos);
	if (ret < 0) {
		err("%s(): file write fail(%s) to EEPROM data(%d)", __func__,
				file_name, ret);
		goto p_err;
	}

	info("%s(): wirte to file(%s)\n", __func__, file_name);
p_err:
	if (!IS_ERR_OR_NULL(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);

	return 0;
}

int fimc_is_eeprom_file_read(const char *file_name, const void *data,
		unsigned long size)
{
	int ret = 0;
	long fsize, nread;
	mm_segment_t old_fs;
	struct file *fp;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(file_name, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(fp)) {
		ret = PTR_ERR(fp);
		err("filp_open(%s) fail(%d)!!\n", file_name, ret);
		goto p_err;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;

	nread = vfs_read(fp, (char __user *)data, size, &fp->f_pos);
	if (nread != size) {
		err("failed to read eeprom file, (%ld) Bytes", nread);
		ret = nread;
		goto p_err;
	}

	info("%s(): read to file(%s)\n", __func__, file_name);
p_err:
	if (!IS_ERR_OR_NULL(fp))
		filp_close(fp, NULL);

	set_fs(old_fs);

	return ret;
}

int fimc_is_sensor_eeprom_check_crc(char *data, size_t size)
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

int fimc_is_sensor_eeprom_check_awb_ratio(char *unit, char *golden, char *limit)
{
	int ret = 0;

	float r_g_min = (float)(limit[0]) / 1000;
	float r_g_max = (float)(limit[1]) / 1000;
	float b_g_min = (float)(limit[2]) / 1000;
	float b_g_max = (float)(limit[3]) / 1000;

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

int fimc_is_eeprom_module_read(struct i2c_client *client, u32 addr,
		char *data, unsigned long size)
{
	int ret = 0;

	/* Read EEPROM cal data in module */
	ret = fimc_is_sensor_read8_size(client, &data[0], addr, size);
	if (ret < 0) {
		err("%s(), i2c read failed(%d)\n", __func__, ret);
		return ret;
	}

	info("EEPROM module read done!!\n");

	return ret;
}

