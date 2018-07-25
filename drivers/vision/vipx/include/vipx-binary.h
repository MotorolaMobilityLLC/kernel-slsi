/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_BINARY_H_
#define VIPX_BINARY_H_

#include <linux/device.h>
#include <linux/firmware.h>

#define VIPX_FW_PATH1 		"/data/"
#define VIPX_FW_PATH2		"/vendor/firmware/"

#define VIPX_FW_DRAM_NAME	"CC_DRAM_CODE_FLASH.bin"
#define VIPX_FW_ITCM_NAME	"CC_ITCM_CODE_FLASH.bin"
#define VIPX_FW_DTCM_NAME	"CC_DTCM_CODE_FLASH.bin"

#define VIPX_FW_NAME_LEN 	100
#define VIPX_VERSION_SIZE	42

struct vipx_binary {
	struct device		*dev;
};

int vipx_binary_init(struct vipx_binary *binary, struct device *dev);

int vipx_binary_read(struct vipx_binary *binary,
	char *path,
	char *name,
	void *target,
	size_t target_size);
int vipx_binary_write(struct vipx_binary *binary,
	char *path,
	char *name,
	void *target,
	size_t target_size);

#endif
