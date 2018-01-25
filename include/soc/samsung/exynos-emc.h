/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - Mode Changer for boosting
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_MODE_CHANGER_H
#define __EXYNOS_MODE_CHANGER_H __FILE__

#ifdef CONFIG_EXYNOS_MODE_CHANGER
void exynos_emc_update(int cpu);
int exynos_emc_update_cpu_pwr(unsigned int cpu, bool on);
#else
static inline void exynos_emc_update(int cpu) {};
static inline int exynos_emc_update_cpu_pwr(unsigned int cpu, bool on) { return 0; };
#endif

#endif /* __EXYNOS_MODE_CHANGER_H */

