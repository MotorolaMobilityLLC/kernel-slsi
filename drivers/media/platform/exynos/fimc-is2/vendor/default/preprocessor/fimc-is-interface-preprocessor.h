/*
 * Samsung EXYNOS FIMC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_PREPROCESSOR_INTERFACE_H
#define FIMC_IS_PREPROCESSOR_INTERFACE_H
struct fimc_is_preprocessor_ops {
	int (*preprocessor_stream_on)(struct v4l2_subdev *subdev);
	int (*preprocessor_stream_off)(struct v4l2_subdev *subdev);
	int (*preprocessor_mode_change)(struct v4l2_subdev *subdev, struct fimc_is_device_sensor *device);
	int (*preprocessor_debug)(struct v4l2_subdev *subdev);
	int (*preprocessor_wait_s_input)(struct v4l2_subdev *subdev);
	int (*preprocessor_deinit)(struct v4l2_subdev *subdev);
	int (*preprocessor_s_format)(struct v4l2_subdev *subdev, struct fimc_is_device_sensor *device);
	int (*preprocessor_set_le_mode)(struct v4l2_subdev *subdev, void *param);
};

struct fimc_is_preprocessor_interface {
	u32						magic;
};

int init_preprocessor_interface(struct fimc_is_preprocessor_interface *itf);
#endif