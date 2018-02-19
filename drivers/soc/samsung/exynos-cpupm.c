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
#include <linux/tick.h>
#include <linux/psci.h>
#include <linux/cpuhotplug.h>

#include <soc/samsung/exynos-cpupm.h>
#include <soc/samsung/exynos-powermode.h>
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

static void cluster_enable(unsigned int cluster_id)
{
	cal_cluster_enable(cluster_id);
}

static void cluster_disable(unsigned int cluster_id)
{
	cal_cluster_disable(cluster_id);
}

/******************************************************************************
 *                            CPU idle management                             *
 ******************************************************************************/
#define IS_NULL(object)		(object == NULL)

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
#define check_state_run(object)		((object)->state == CPUPM_STATE_RUN)
#define check_state_powerdown(object)	((object)->state == CPUPM_STATE_POWERDOWN)

/* Length of power mode name */
#define NAME_LEN	32

/*
 * Power modes
 * In CPUPM, power mode controls the power domain consisting of cpu and enters
 * the power mode by cpuidle. Basically, to enter power mode, all cpus in power
 * domain must be in POWERDOWN state, and sleep length of cpus must be smaller
 * than target_residency.
 */
struct power_mode {
	/* name of power mode, it is declared in device tree */
	char		name[NAME_LEN];

	/* power mode state, RUN or POWERDOWN */
	int		state;

	/* sleep length criterion of cpus to enter power mode */
	int		target_residency;

	/* PSCI index for entering power mode */
	int		psci_index;

	/* type according to h/w configuration of power domain */
	int		type;

	/* cpus belonging to the power domain */
	struct cpumask	siblings;

	/*
	 * Among siblings, the cpus that can enter the power mode.
	 * Due to H/W constraint, only certain cpus need to enter power mode.
	 */
	struct cpumask	entry_allowed;

	/* disable count */
	atomic_t	disable;

	/*
	 * Some power modes can determine whether to enter power mode
	 * depending on system idle state
	 */
	bool		system_idle;
};

/* Maximum number of power modes manageable per cpu */
#define MAX_MODE	5

/* Iterator for power mode */
#define for_each_mode(mode, array, pos)			\
	for ((pos) = 0, (mode) = (array)[0];		\
		(mode) = (array)[(pos)],		\
		(pos) < MAX_MODE; (pos)++)

/*
 * Main struct of CPUPM
 * Each cpu has its own data structure and main purpose of this struct is to
 * manage the state of the cpu and the power modes containing the cpu.
 */
struct exynos_cpupm {
	/* cpu state, RUN or POWERDOWN */
	int			state;

	/* array to manage the power mode that contains the cpu */
	struct power_mode *	modes[MAX_MODE];
};

static DEFINE_PER_CPU(struct exynos_cpupm, cpupm);

/*
 * State of each cpu is managed by a structure declared by percpu, so there
 * is no need for protection for synchronization. However, when entering
 * the power mode, it is necessary to set the critical section to check the
 * state of cpus in the power domain, cpupm_lock is used for it.
 */
static spinlock_t cpupm_lock;

/* Nop function to awake cpus */
static void do_nothing(void *unused)
{
}

static void awake_cpus(const struct cpumask *cpus)
{
	struct cpumask mask;

	cpumask_and(&mask, cpus, cpu_online_mask);
	smp_call_function_many(&mask, do_nothing, NULL, 0);
}

/*
 * disable_power_mode/enable_power_mode
 * It provides "disable" function to enable/disable power mode as required by
 * user or device driver. To handle multiple disable requests, it use the
 * atomic disable count, and disable the mode that contains the given cpu.
 */
void disable_power_mode(int cpu, int type)
{
	struct exynos_cpupm *pm;
	struct power_mode *mode;
	int pos;

	spin_lock(&cpupm_lock);
	pm = &per_cpu(cpupm, cpu);

	for_each_mode(mode, pm->modes, pos) {
		if (IS_NULL(mode))
			break;

		if (mode->type == type) {
			/*
			 * The first mode disable request wakes the cpus to
			 * exit power mode
			 */
			if (atomic_inc_return(&mode->disable) == 1)
				awake_cpus(&mode->siblings);
		}
	}
	spin_unlock(&cpupm_lock);
}

void enable_power_mode(int cpu, int type)
{
	struct exynos_cpupm *pm;
	struct power_mode *mode;
	int pos;

	spin_lock(&cpupm_lock);
	pm = &per_cpu(cpupm, cpu);

	for_each_mode(mode, pm->modes, pos) {
		if (IS_NULL(mode))
			break;

		if (mode->type == type)
			atomic_dec(&mode->disable);
	}
	spin_unlock(&cpupm_lock);
}

/* get sleep length of given cpu from tickless framework */
static s64 get_sleep_length(int cpu)
{
	return ktime_to_us(tick_nohz_get_sleep_length_cpu(cpu));
}

static int cpus_busy(int target_residency, const struct cpumask *cpus)
{
	int cpu;

	/*
	 * If there is even one cpu which is not in POWERDOWN state or has
	 * the smaller sleep length than target_residency, CPUPM regards
	 * it as BUSY.
	 */
	for_each_cpu_and(cpu, cpu_online_mask, cpus) {
		struct exynos_cpupm *pm = &per_cpu(cpupm, cpu);

		if (check_state_run(pm))
			return -EBUSY;

		if (get_sleep_length(cpu) < target_residency)
			return -EBUSY;
	}

	return 0;
}

static int system_busy(void)
{
	return 0;
}

/*
 * In order to enter the power mode, the following conditions must be met:
 * 1. power mode should not be disabled
 * 2. the cpu attempting to enter must be a cpu that is allowed to enter the
 *    power mode.
 * 3. all cpus in the power domain must be in POWERDOWN state and the sleep
 *    length of the cpus must be less than target_residency.
 */
static int can_enter_power_mode(int cpu, struct power_mode *mode)
{
	if (atomic_read(&mode->disable))
		return 0;

	if (!cpumask_test_cpu(cpu, &mode->entry_allowed))
		return 0;

	if (cpus_busy(mode->target_residency, &mode->siblings))
		return 0;

	if (mode->system_idle && system_busy())
		return 0;

	return 1;
}

extern struct cpu_topology cpu_topology[NR_CPUS];

static int try_to_enter_power_mode(int cpu, struct power_mode *mode)
{
	/* Check whether mode can be entered */
	if (!can_enter_power_mode(cpu, mode)) {
		/* fail to enter power mode */
		return 0;
	}

	/*
	 * From this point on, it has succeeded in entering the power mode.
	 * It prepares to enter power mode, and makes the corresponding
	 * setting according to type of power mode.
	 */
	switch (mode->type) {
	case POWERMODE_TYPE_CLUSTER:
		cluster_disable(cpu_topology[cpu].cluster_id);
		break;
	case POWERMODE_TYPE_SYSTEM:
		if (unlikely(exynos_system_idle_enter()))
			return 0;
		break;
	}

	exynos_ss_cpuidle(mode->name, 0, 0, ESS_FLAG_IN);
	set_state_powerdown(mode);

	return 1;
}

static void exit_mode(int cpu, struct power_mode *mode, int cancel)
{
	/*
	 * Configure settings to exit power mode. This is executed by the
	 * first cpu exiting from power mode.
	 */
	set_state_run(mode);
	exynos_ss_cpuidle(mode->name, 0, 0, ESS_FLAG_OUT);

	switch (mode->type) {
	case POWERMODE_TYPE_CLUSTER:
		cluster_enable(cpu_topology[cpu].cluster_id);
		break;
	case POWERMODE_TYPE_SYSTEM:
		exynos_system_idle_exit(cancel);
		break;
	}
}

/*
 * Exynos cpuidle driver call exynos_cpu_pm_enter() and exynos_cpu_pm_exit()
 * to handle platform specific configuration to control cpu power domain.
 */
int exynos_cpu_pm_enter(int cpu, int index)
{
	struct exynos_cpupm *pm;
	struct power_mode *mode;
	int pos;

	spin_lock(&cpupm_lock);
	pm = &per_cpu(cpupm, cpu);

	/* Configure PMUCAL to power down core */
	cpu_disable(cpu);

	/* Set cpu state to POWERDOWN */
	set_state_powerdown(pm);

	/* Try to enter power mode */
	for_each_mode(mode, pm->modes, pos) {
		if (IS_NULL(mode))
			break;

		if (try_to_enter_power_mode(cpu, mode))
			index |= mode->psci_index;
	}

	spin_unlock(&cpupm_lock);

	return index;
}

void exynos_cpu_pm_exit(int cpu, int cancel)
{
	struct exynos_cpupm *pm;
	struct power_mode *mode;
	int pos;

	spin_lock(&cpupm_lock);
	pm = &per_cpu(cpupm, cpu);

	/* Make settings to exit from mode */
	for_each_mode(mode, pm->modes, pos) {
		if (IS_NULL(mode))
			break;

		if (check_state_powerdown(mode))
			exit_mode(cpu, mode, cancel);
	}

	/* Set cpu state to RUN */
	set_state_run(pm);

	/* Configure PMUCAL to power up core */
	cpu_enable(cpu);

	spin_unlock(&cpupm_lock);
}

/******************************************************************************
 *                               CPU HOTPLUG                                  *
 ******************************************************************************/
static int cpuhp_cpupm_online(unsigned int cpu)
{
	struct cpumask mask;

	cpumask_and(&mask, cpu_coregroup_mask(cpu), cpu_online_mask);
	if (cpumask_weight(&mask) == 0)
		cluster_enable(cpu);

	cpu_enable(cpu);

	return 0;
}

static int cpuhp_cpupm_offline(unsigned int cpu)
{
	struct cpumask mask;

	cpu_disable(cpu);

	cpumask_and(&mask, cpu_coregroup_mask(cpu), cpu_online_mask);
	if (cpumask_weight(&mask) == 0)
		cluster_disable(cpu);

	return 0;
}

/******************************************************************************
 *                                Initialization                              *
 ******************************************************************************/
static void __init
add_mode(struct power_mode **modes, struct power_mode *new)
{
	struct power_mode *mode;
	int pos;

	for_each_mode(mode, modes, pos) {
		if (IS_NULL(mode)) {
			modes[pos] = new;
			return;
		}
	}

	pr_warn("The number of modes exceeded\n");
}

static int __init cpu_power_mode_init(void)
{
	struct device_node *dn = NULL;
	struct power_mode *mode;
	const char *buf;

	while ((dn = of_find_node_by_type(dn, "cpupm"))) {
		int cpu;

		/*
		 * Power mode is dynamically generated according to what is defined
		 * in device tree.
		 */
		mode = kzalloc(sizeof(struct power_mode), GFP_KERNEL);
		if (!mode) {
			pr_warn("%s: No memory space!\n", __func__);
			continue;
		}

		strncpy(mode->name, dn->name, NAME_LEN);

		of_property_read_u32(dn, "target-residency", &mode->target_residency);
		of_property_read_u32(dn, "psci-index", &mode->psci_index);
		of_property_read_u32(dn, "type", &mode->type);

		if (of_property_read_bool(dn, "system-idle"))
			mode->system_idle = true;

		if (!of_property_read_string(dn, "siblings", &buf))
			cpulist_parse(buf, &mode->siblings);

		if (!of_property_read_string(dn, "entry-allowed", &buf))
			cpulist_parse(buf, &mode->entry_allowed);

		atomic_set(&mode->disable, 0);

		/* Connect power mode to the cpus in the power domain */
		for_each_cpu(cpu, &mode->siblings)
			add_mode(per_cpu(cpupm, cpu).modes, mode);
	}

	return 0;
}

static int __init exynos_cpupm_init(void)
{
	cpu_power_mode_init();

	spin_lock_init(&cpupm_lock);

	return 0;
}
arch_initcall(exynos_cpupm_init);

static int __init exynos_cpupm_early_init(void)
{
	cpuhp_setup_state(CPUHP_AP_EXYNOS_CPU_UP_POWER_CONTROL,
				"AP_EXYNOS_CPU_UP_POWER_CONTROL",
				cpuhp_cpupm_online, NULL);
	cpuhp_setup_state(CPUHP_AP_EXYNOS_CPU_DOWN_POWER_CONTROL,
				"AP_EXYNOS_CPU_DOWN_POWER_CONTROL",
				NULL, cpuhp_cpupm_offline);

	return 0;
}
early_initcall(exynos_cpupm_early_init);
