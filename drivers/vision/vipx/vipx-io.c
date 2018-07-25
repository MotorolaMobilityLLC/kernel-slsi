/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/slab.h>

#include "vipx-io.h"
void *mem2iocpy(void *dst,void *src, u32 size)
{
	u8  *src8;
	u8  *dst8;

	src8 = (u8 *)(src);
	dst8 = (u8 *)(dst);
	/* first copy byte by byte till the source first alignment
	 * this step is necessary to ensure we do not even try to access
	 * data which is before the source buffer, hence it is not ours.
	 */
	/* complete the left overs */
	while (size--)
	{
		IOW8(*dst8, *src8);
		dst8++;
		src8++;
	}
	return dst;
}

void *io2memcpy(void *dst,void *src, u32 size)
{
	u8  *src8;
	u8  *dst8;

	src8 = (u8 *)(src);
	dst8 = (u8 *)(dst);

	while (size--)
	{
		*dst8 = IOR8(*src8);
		dst8++;
		src8++;
	}

	return dst;
}
