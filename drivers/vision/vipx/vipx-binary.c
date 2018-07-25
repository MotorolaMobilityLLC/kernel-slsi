/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/syscalls.h>
#include <linux/vmalloc.h>

#include "vipx-config.h"
#include "vipx-binary.h"
#include "vipx-memory.h"

static noinline_for_stack long __get_file_size(struct file *file)
{
	struct kstat st;
	u32 request_mask = (STATX_MODE | STATX_SIZE);
    
	if (vfs_getattr(&file->f_path, &st, request_mask, KSTAT_QUERY_FLAGS))
		return -1;
	if (!S_ISREG(st.mode))
		return -1;
	if (st.size != (long)st.size)
		return -1;

	return st.size;
}

int vipx_binary_init(struct vipx_binary *binary, struct device *dev)
{
	int ret = 0;

	BUG_ON(!binary);
	BUG_ON(!dev);

	binary->dev = dev;

	return ret;
}

int vipx_binary_read(struct vipx_binary *binary,
	char *path,
	char *name,
	void *target,
	size_t target_size)
{
	int ret = 0;
    loff_t pos = 0;
	const struct firmware *fw_blob;
	u8 *buf = NULL;
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	char fname[VIPX_FW_NAME_LEN];

	BUG_ON(!binary);

	snprintf(fname, sizeof(fname), "%s%s", path, name);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(fname, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(fp)) {
		set_fs(old_fs);
		goto request_fw;
	}

	fsize = __get_file_size(fp);
	if (fsize <= 0) {
		vipx_err("__get_file_size is fail(%ld)\n", fsize);
		ret = -EBADF;
		goto p_err;
	}

	buf = vmalloc(fsize);
	if (!buf) {
		vipx_err("vmalloc is fail\n");
		ret = -ENOMEM;
		goto p_err;
	}

	nread = kernel_read(fp, buf, fsize, &pos);
	if (nread != fsize) {
		vipx_err("kernel_read is fail(%ld != %ld)\n", nread, fsize);
		ret = -EIO;
		goto p_err;
	}

	if (fsize > target_size) {
		vipx_err("image size is over(%ld > %ld)\n", fsize, target_size);
		ret = -EIO;
		goto p_err;
	}

	/* no cache operation, because target is sram of vipx */
#ifdef CONFIG_EXYNOS_VIPX_HARDWARE
	memcpy(target, (void *)buf, fsize);
#endif
	vipx_info("FW(%s, %ld) were applied successfully.\n", fname, fsize);

p_err:
	if (buf)
		vfree(buf);

	filp_close(fp, current->files);
	set_fs(old_fs);

	return ret;

request_fw:
	ret = request_firmware(&fw_blob, name, binary->dev);
	if (ret) {
		vipx_err("request_firmware(%s) is fail(%d)", name, ret);
		ret = -EINVAL;
		goto request_err;
	}

	if (!fw_blob) {
		vipx_err("fw_blob is NULL\n");
		ret = -EINVAL;
		goto request_err;
	}

	if (!fw_blob->data) {
		vipx_err("fw_blob->data is NULL\n");
		ret = -EINVAL;
		goto request_err;
	}

	if (fw_blob->size > target_size) {
		vipx_err("image size is over(%ld > %ld)\n", fw_blob->size, target_size);
		ret = -EIO;
		goto request_err;
	}

#ifdef CONFIG_EXYNOS_VIPX_HARDWARE
	memcpy(target, fw_blob->data, fw_blob->size);
#endif
	vipx_info("Binay(%s, %ld) were applied successfully.\n", name, fw_blob->size);

request_err:
	release_firmware(fw_blob);
	return ret;
}

int vipx_binary_write(struct vipx_binary *binary,
	char *path,
	char *name,
	void *target,
	size_t target_size)
{
	int ret = 0;
	struct file *fp;
	mm_segment_t old_fs;
	long nwrite;
    loff_t pos = 0;
	char fname[VIPX_FW_NAME_LEN];

	BUG_ON(!binary);

	snprintf(fname, sizeof(fname), "%s%s", path, name);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(fname, O_RDWR | O_CREAT, 0);
	if (IS_ERR_OR_NULL(fp)) {
		set_fs(old_fs);
		vipx_err("filp_open is fail(%p)\n", fp);
		ret = -EBADF;
		goto p_err;
	}

    vipx_info("debug kva(0x%p), size(%ld)\n", target, target_size);

    
	nwrite = kernel_write(fp, target, target_size, &pos);
	if (nwrite != target_size) {
		filp_close(fp, current->files);
		set_fs(old_fs);
		vipx_err("kernel_write is fail(%ld != %ld)\n", nwrite, target_size);
		ret = -EIO;
		goto p_err;
	}

	vipx_info("Binay(%s, %ld) were applied successfully.\n", fname, target_size);

	filp_close(fp, current->files);
	set_fs(old_fs);

p_err:
	return ret;
}
