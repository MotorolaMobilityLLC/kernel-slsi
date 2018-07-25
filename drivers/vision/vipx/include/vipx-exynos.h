/*
 * Samsung Exynos SoC series VIPX driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_EXYNOS_H_
#define VIPX_EXYNOS_H_

#include <linux/clk.h>
#include <linux/device.h>

struct vipx_exynos;

enum {
	VIPX_REG_CPU_SS1 = 0,
	VIPX_REG_CPU_SS2 = 1,
	VIPX_REG_MAX_CNT,
};

struct vipx_regblock {
	unsigned int		offset;
	unsigned int		blocks;
	char			*name;
};

enum  regdata_type {
	/* read write */
	RW			= 0,
	/* read only */
	RO			= 1,
	/* write only */
	WO			= 2,
	/* write input */
	WI			= 2,
	/* clear after read */
	RAC			= 3,
	/* write 1 -> clear */
	W1C			= 4,
	/* write read input */
	WRI			= 5,
	/* write input */
	RWI			= 5,
	/* only scaler */
	R_W			= 6,
	/* read & write for clear */
	RWC			= 7,
	/* read & write as dual setting */
	RWS
};

struct vipx_reg {
	unsigned int		offset;
	char			*name;
};

struct vipx_field {
	char			*name;
	unsigned int		bit_start;
	unsigned int		bit_width;
	enum regdata_type	type;
};

struct vipx_clk {
	const char	*name;
	struct clk	*clk;
};

struct vipx_clk_ops {
	int (*clk_cfg)(struct vipx_exynos *exynos);
	int (*clk_on)(struct vipx_exynos *exynos);
	int (*clk_off)(struct vipx_exynos *exynos);
	int (*clk_dump)(struct vipx_exynos *exynos);
};

struct vipx_ctl_ops {
	int (*ctl_reset)(struct vipx_exynos *exynos, bool hold);
	int (*ctl_dump)(struct vipx_exynos *exynos, u32 instance);
	void * (*ctl_remap)(struct vipx_exynos *exynos, u32 instance);
	int (*ctl_unmap)(struct vipx_exynos *exynos, u32 instance, void *base);
	int (*ctl_trigger)(struct vipx_exynos *exynos, u32 chain_id);
	int (*ctl_start)(struct vipx_exynos *exynos, u32 chain_id);
};

struct vipx_exynos {
	struct device			*dev;
	void				**regbase;
	void				*ram0base;
	void				*ram1base;
	struct pinctrl			*pinctrl;
	const struct vipx_clk_ops	*clk_ops;
	const struct vipx_ctl_ops	*ctl_ops;
};

int vipx_exynos_probe(struct vipx_exynos *exynos, struct device *dev,
	void **regs, void *ram0, void *ram1);

void vipx_readl(void __iomem *base_addr, struct vipx_reg *reg, u32 *val);
void vipx_writel(void __iomem *base_addr, struct vipx_reg *reg, u32 val);

#define CLK_OP(exynos, op) (exynos->clk_ops ? exynos->clk_ops->op(exynos) : 0)
#define CTL_OP(exynos, op, ...) (exynos->ctl_ops ? exynos->ctl_ops->op(exynos, ##__VA_ARGS__) : 0)

#endif
