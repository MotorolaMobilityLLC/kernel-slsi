/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/firmware.h>
#include <linux/uaccess.h>

#include "vipx-log.h"
#include "vipx-system.h"
#include "vipx-binary.h"

static noinline_for_stack loff_t __vipx_binary_get_file_size(struct file *file)
{
	int ret;
	struct kstat st;
	unsigned int request_mask = (STATX_MODE | STATX_SIZE);

	vipx_enter();
	ret = vfs_getattr(&file->f_path, &st, request_mask, KSTAT_QUERY_FLAGS);
	if (ret) {
		vipx_err("vfs_getattr failed (%d)\n", ret);
		goto p_err;
	}

	if (!S_ISREG(st.mode)) {
		ret = -EINVAL;
		vipx_err("file mode is not S_ISREG\n");
		goto p_err;
	}

	vipx_leave();
	return st.size;
p_err:
	return (loff_t)ret;
}

static int __vipx_binary_file_read(struct vipx_binary *bin, const char *path,
		const char *name, void *target, size_t size)
{
	int ret;
	mm_segment_t old_fs;
	char fname[VIPX_FW_NAME_LEN];
	struct file *fp;
	loff_t fsize, pos = 0;
	ssize_t nread;

	vipx_enter();
	snprintf(fname, sizeof(fname), "%s/%s", path, name);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(fname, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		set_fs(old_fs);
		ret = PTR_ERR(fp);
		vipx_err("filp_open(%s) is failed (%d)\n", fname, ret);
		goto p_err_open;
	}

	fsize = __vipx_binary_get_file_size(fp);
	if (fsize <= 0) {
		ret = (int)fsize;
		goto p_err_size;
	}

	if (fsize > size) {
		ret = -EIO;
		vipx_err("file(%s) size is over (%lld > %zu)\n",
				fname, fsize, size);
		goto p_err_size;
	}

	nread = kernel_read(fp, target, fsize, &pos);
	filp_close(fp, current->files);
	set_fs(old_fs);

	if (nread < 0) {
		ret = (int)nread;
		vipx_err("kernel_read(%s) is fail (%d)\n", fname, ret);
		goto p_err_read;
	}

	vipx_leave();
	return 0;
p_err_read:
p_err_size:
p_err_open:
	return ret;
}

static int __vipx_binary_firmware_load(struct vipx_binary *bin,
		const char *name, void *target, size_t size)
{
	int ret;
	const struct firmware *fw_blob;

	vipx_enter();
	if (!target) {
		ret = -EINVAL;
		vipx_err("binary(%s) memory is NULL\n", name);
		goto p_err_target;
	}

	ret = request_firmware(&fw_blob, name, bin->dev);
	if (ret) {
		vipx_err("request_firmware(%s) is fail (%d)\n", name, ret);
		goto p_err_req;
	}

	if (fw_blob->size > size) {
		ret = -EIO;
		vipx_err("binary(%s) size is over (%ld > %ld)\n",
				name, fw_blob->size, size);
		goto p_err_size;
	}

	memcpy(target, fw_blob->data, fw_blob->size);
	release_firmware(fw_blob);

	vipx_leave();
	return 0;
p_err_size:
	release_firmware(fw_blob);
p_err_req:
p_err_target:
	return ret;
}

int vipx_binary_read(struct vipx_binary *bin, const char *path,
		const char *name, void *target, size_t size)
{
	int ret;

	vipx_enter();

	if (path)
		ret = __vipx_binary_file_read(bin, path, name, target, size);
	else
		ret = __vipx_binary_firmware_load(bin, name, target, size);

	vipx_leave();
	return ret;
}

int vipx_binary_write(struct vipx_binary *bin, const char *path,
		const char *name, void *target, size_t size)
{
	int ret;
	char fname[VIPX_FW_NAME_LEN];
	mm_segment_t old_fs;
	struct file *fp;
	ssize_t nwrite;
	loff_t pos = 0;

	vipx_enter();
	snprintf(fname, sizeof(fname), "%s/%s", path, name);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(fname, O_RDWR | O_CREAT, 0);
	if (IS_ERR(fp)) {
		set_fs(old_fs);
		ret = PTR_ERR(fp);
		vipx_err("filp_open(%s) is fail (%d)\n", fname, ret);
		goto p_err;
	}

	nwrite = kernel_write(fp, target, size, &pos);
	filp_close(fp, current->files);
	set_fs(old_fs);

	if (nwrite < 0) {
		ret = (int)nwrite;
		vipx_err("kernel_write(%s) is fail (%d)\n", fname, ret);
		goto p_err;
	}

	vipx_leave();
	return 0;
p_err:
	return ret;
}

int vipx_binary_init(struct vipx_system *sys)
{
	struct vipx_binary *bin;

	vipx_enter();
	bin = &sys->binary;
	bin->dev = sys->dev;
	vipx_leave();
	return 0;
}

void vipx_binary_deinit(struct vipx_binary *bin)
{
	vipx_enter();
	vipx_leave();
}
