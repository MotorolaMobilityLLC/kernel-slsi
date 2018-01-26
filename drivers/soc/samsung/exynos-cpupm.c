/*
 * Copyright (c) 2018 Park Bumgyu, Samsung Electronics Co., Ltd <bumgyu.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Exynos CPU Power Management driver implementation
 */

#include <linux/cpumask.h>
#include <linux/slab.h>

#include <soc/samsung/cal-if.h>

/******************************************************************************
 *                                CAL interfaces                              *
 ******************************************************************************/
static void cpu_enable(unsigned int cpu)
{
	cal_cpu_enable(cpu);
}

static void cpu_disable(unsigned int cpu)
{
	cal_cpu_disable(cpu);
}

/******************************************************************************
 *                            CPU idle management                             *
 ******************************************************************************/
/*
 * State of CPUPM objects
 * All CPUPM objects have 2 states, RUN and POWERDOWN.
 *
 * @RUN
 * a state in which the power domain referred to by the object is turned on.
 *
 * @POWERDOWN
 * a state in which the power domain referred to by the object is turned off.
 * However, the power domain is not necessarily turned off even if the object
 * is in POEWRDOWN state because the cpu may be booting or executing power off
 * sequence.
 */
enum {
	CPUPM_STATE_RUN = 0,
	CPUPM_STATE_POWERDOWN,
};

/* Macros for CPUPM state */
#define set_state_run(object)		(object)->state = CPUPM_STATE_RUN
#define set_state_powerdown(object)	(object)->state = CPUPM_STATE_POWERDOWN

/*
 * Main struct of CPUPM
 * Each cpu has its own data structure and main purpose of this struct is to
 * manage the state of the cpu.
 */
struct exynos_cpupm {
	/* cpu state, RUN or POWERDOWN */
	int			state;
};

static DEFINE_PER_CPU(struct exynos_cpupm, cpupm);

/*
 * State of each cpu is managed by a structure declared by percpu, so there
 * is no need for protection for synchronization. However, when entering
 * the power mode, it is necessary to set the critical section to check the
 * state of cpus in the power domain, cpupm_lock is used for it.
 */
static spinlock_t cpupm_lock;

/*
 * Exynos cpuidle driver call exynos_cpu_pm_enter() and exynos_cpu_pm_exit()
 * to handle platform specific configuration to control cpu power domain.
 */
int exynos_cpu_pm_enter(int cpu, int index)
{
	struct exynos_cpupm *pm;

	spin_lock(&cpupm_lock);
	pm = &per_cpu(cpupm, cpu);

	/* Configure PMUCAL to power down core */
	cpu_disable(cpu);

	/* Set cpu state to POWERDOWN */
	set_state_powerdown(pm);

	spin_unlock(&cpupm_lock);

	return index;
}

void exynos_cpu_pm_exit(int cpu, int cancel)
{
	struct exynos_cpupm *pm;

	spin_lock(&cpupm_lock);
	pm = &per_cpu(cpupm, cpu);

	/* Set cpu state to RUN */
	set_state_run(pm);

	/* Configure PMUCAL to power up core */
	cpu_enable(cpu);

	spin_unlock(&cpupm_lock);
}

/******************************************************************************
 *                                Initialization                              *
 ******************************************************************************/
static int __init exynos_cpupm_init(void)
{
	spin_lock_init(&cpupm_lock);

	return 0;
}
arch_initcall(exynos_cpupm_init);
