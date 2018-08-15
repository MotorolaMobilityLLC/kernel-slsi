/*
 * Copyright (c) 2009-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SOCINFO_H_
#define _SOCINFO_H_

#define SOCINFO_VERSION_MAJOR(ver) (((ver) & 0xffff0000) >> 16)
#define SOCINFO_VERSION_MINOR(ver) ((ver) & 0x0000ffff)
#define SOCINFO_VERSION(maj, min)  ((((maj) & 0xffff) << 16)|((min) & 0xffff))


uint32_t socinfo_get_id(void)
{
	return 0;
}
uint32_t socinfo_get_version(void)
{
	return 0;
}
uint32_t socinfo_get_raw_id(void)
{
	return 0;
}
uint32_t socinfo_get_raw_version(void)
{
	return 0;
}
uint32_t socinfo_get_platform_type(void)
{
	return 0;
}
uint32_t socinfo_get_platform_subtype(void)
{
	return 0;
}
uint32_t socinfo_get_platform_version(void)
{
	return 0;
}

#endif
