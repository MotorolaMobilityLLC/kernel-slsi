/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_FPSIMD_H
#define FIMC_IS_FPSIMD_H

#include <asm/fpsimd.h>

#ifdef CONFIG_KERNEL_MODE_NEON

void fimc_is_kernel_neon_begin(void)
{
	fpsimd_save_state(&current->thread.fpsimd_state);
}

void fimc_is_kernel_neon_end(void)
{
	fpsimd_load_state(&current->thread.fpsimd_state);
}

#endif /* CONFIG_KERNEL_MODE_NEON */
#endif
