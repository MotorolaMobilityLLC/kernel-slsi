/*
 * Samsung Exynos SoC series VIPX driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "vipx-log.h"
#include "vipx-system.h"
#include "platform/vipx-clk.h"

enum vipx_clk_id {
	VIPX_CLK_UMUX_CLKCMU_VIPX1_BUS,
	VIPX_CLK_GATE_VIPX1_QCH,
	VIPX_CLK_UMUX_CLKCMU_VIPX2_BUS,
	VIPX_CLK_GATE_VIPX2_QCH,
	VIPX_CLK_GATE_VIPX2_QCH_LOCAL,
	VIPX_CLK_MAX
};

static struct vipx_clk vipx_exynos9610_clk_array[] = {
	{ NULL, "UMUX_CLKCMU_VIPX1_BUS" },
	{ NULL, "GATE_VIPX1_QCH"        },
	{ NULL, "UMUX_CLKCMU_VIPX2_BUS" },
	{ NULL, "GATE_VIPX2_QCH"        },
	{ NULL, "GATE_VIPX2_QCH_LOCAL"  },
};

static int vipx_exynos9610_clk_init(struct vipx_system *sys)
{
	int ret;
	int index;
	const char *name;
	struct clk *clk;

	vipx_enter();
	if (ARRAY_SIZE(vipx_exynos9610_clk_array) != VIPX_CLK_MAX) {
		ret = -EINVAL;
		vipx_err("clock array size is invalid (%zu/%d)\n",
				ARRAY_SIZE(vipx_exynos9610_clk_array),
				VIPX_CLK_MAX);
		goto p_err;
	}

	for (index = 0; index < VIPX_CLK_MAX; ++index) {
		name = vipx_exynos9610_clk_array[index].name;
		if (!name) {
			ret = -EINVAL;
			vipx_err("clock name is NULL (%d)\n", index);
			goto p_err;
		}

		clk = devm_clk_get(sys->dev, name);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			vipx_err("Failed to get clock(%s/%d) (%d)\n",
					name, index, ret);
			goto p_err;
		}

		vipx_exynos9610_clk_array[index].clk = clk;
	}

	vipx_leave();
	return 0;
p_err:
	return ret;
}

static void vipx_exynos9610_clk_deinit(struct vipx_system *sys)
{
	vipx_enter();
	vipx_leave();
}

static int vipx_exynos9610_clk_cfg(struct vipx_system *sys)
{
	vipx_enter();
	vipx_leave();
	return 0;
}

static int vipx_exynos9610_clk_on(struct vipx_system *sys)
{
	int ret;
	int index;
	const char *name;
	struct clk *clk;

	vipx_enter();
	for (index = 0; index < VIPX_CLK_MAX; ++index) {
		name = vipx_exynos9610_clk_array[index].name;
		clk = vipx_exynos9610_clk_array[index].clk;

		ret = clk_prepare_enable(clk);
		if (ret) {
			vipx_err("Failed to enable clock(%s/%d) (%d)\n",
					name, index, ret);
			goto p_err;
		}
	}

	vipx_leave();
	return 0;
p_err:
	return ret;
}

static int vipx_exynos9610_clk_off(struct vipx_system *sys)
{
	int index;
	const char *name;
	struct clk *clk;

	vipx_enter();
	for (index = VIPX_CLK_MAX - 1; index >= 0; --index) {
		name = vipx_exynos9610_clk_array[index].name;
		clk = vipx_exynos9610_clk_array[index].clk;

		clk_disable_unprepare(clk);
	}

	vipx_leave();
	return 0;
}

static int vipx_exynos9610_clk_dump(struct vipx_system *sys)
{
	int index;
	const char *name;
	struct clk *clk;
	unsigned long freq;

	vipx_enter();
	for (index = 0; index < VIPX_CLK_MAX; ++index) {
		name = vipx_exynos9610_clk_array[index].name;
		clk = vipx_exynos9610_clk_array[index].clk;

		freq = clk_get_rate(clk);
		vipx_info("%30s(%d) : %9lu Hz\n", name, index, freq);
	}

	vipx_leave();
	return 0;
}

const struct vipx_clk_ops vipx_clk_ops = {
	.clk_init	= vipx_exynos9610_clk_init,
	.clk_deinit	= vipx_exynos9610_clk_deinit,
	.clk_cfg	= vipx_exynos9610_clk_cfg,
	.clk_on		= vipx_exynos9610_clk_on,
	.clk_off	= vipx_exynos9610_clk_off,
	.clk_dump	= vipx_exynos9610_clk_dump
};
