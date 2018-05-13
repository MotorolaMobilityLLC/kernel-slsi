/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Debug-SnapShot: Debug Framework for Ramdump based debugging method
 * The original code is Exynos-Snapshot for Exynos SoC
 *
 * Author: Hosung Kim <hosung0.kim@samsung.com>
 * Author: Changki Kim <changki.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef DEBUG_SNAPSHOT_SOC_H
#define DEBUG_SNAPSHOT_SOC_H

/* To Support Samsung SoC */
#include <dt-bindings/soc/samsung/debug-snapshot-table.h>
#ifdef CONFIG_DEBUG_SNAPSHOT_PMU
#include <soc/samsung/cal-if.h>
#endif

/* SoC Dependent Header */
#define DSS_REG_MCT_ADDR	(0)
#define DSS_REG_MCT_SIZE	(0)
#define DSS_REG_UART_ADDR	(0)
#define DSS_REG_UART_SIZE	(0)

struct dbg_snapshot_ops {
        int (*pd_status)(unsigned int id);
};

extern struct dbg_snapshot_ops dss_ops;
#endif
