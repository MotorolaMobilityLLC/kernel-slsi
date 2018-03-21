/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - CPU CONTROL support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_CPU_HP_H
#define __EXYNOS_CPU_HP_H __FILE__

#define FAST_HP		0xFA57

extern int exynos_cpuhp_unregister(char *name, struct cpumask mask, int type);
extern int exynos_cpuhp_register(char *name, struct cpumask mask, int type);
extern int exynos_cpuhp_request(char *name, struct cpumask mask, int type);

#ifdef EXYNOS_FAST_HP
int cpus_up(struct cpumask *mask);
int cpus_down(struct cpumask *mask);
#else
extern inline int cpus_up(struct cpumask *mask) { return 0; };
extern inline int cpus_down(struct cpumask *mask) {return 0; };
#endif

#endif /* __EXYNOS_CPU_HP_H */
