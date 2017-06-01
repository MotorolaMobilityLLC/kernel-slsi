#include <linux/cgroup.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/ems.h>

#include <trace/events/sched.h>

#include "sched.h"
#include "tune.h"

bool schedtune_initialized = false;
extern struct reciprocal_value schedtune_spc_rdiv;

struct group_balancer {
	/* sum of task utilization in group */
	unsigned long util;

	/* group balancing threshold */
	unsigned long threshold;

	/* imbalance ratio by heaviest task */
	unsigned int imbalance_ratio;

	/* balance ratio by heaviest task */
	unsigned int balance_ratio;

	/* heaviest task utilization in group */
	unsigned long heaviest_util;

	/* group utilization update interval */
	unsigned long update_interval;

	/* next group utilization update time */
	unsigned long next_update_time;

	/*
	 * group imbalance time = imbalance_count * update_interval
	 * imbalance_count >= imbalance_duration -> need balance
	 */
	unsigned int imbalance_duration;
	unsigned int imbalance_count;

	/* utilization tracking window size */
	unsigned long window;

	/* group balancer locking */
	raw_spinlock_t lock;

	/* need group balancing? */
	bool need_balance;
};

/*
 * EAS scheduler tunables for task groups.
 */

/* SchdTune tunables for a group of tasks */
struct schedtune {
	/* SchedTune CGroup subsystem */
	struct cgroup_subsys_state css;

	/* Boost group allocated ID */
	int idx;

	/* Boost value for tasks on that SchedTune CGroup */
	int boost;

	/* Hint to bias scheduling of tasks on that SchedTune CGroup
	 * towards idle CPUs */
	int prefer_idle;

	/* Hint to bias scheduling of tasks on that SchedTune CGroup
	 * towards high performance CPUs */
	int prefer_perf;

	/* SchedTune group balancer */
	struct group_balancer gb;
};

static inline struct schedtune *css_st(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct schedtune, css) : NULL;
}

static inline struct schedtune *task_schedtune(struct task_struct *tsk)
{
	return css_st(task_css(tsk, schedtune_cgrp_id));
}

static inline struct schedtune *parent_st(struct schedtune *st)
{
	return css_st(st->css.parent);
}

/*
 * SchedTune root control group
 * The root control group is used to defined a system-wide boosting tuning,
 * which is applied to all tasks in the system.
 * Task specific boost tuning could be specified by creating and
 * configuring a child control group under the root one.
 * By default, system-wide boosting is disabled, i.e. no boosting is applied
 * to tasks which are not into a child control group.
 */
static struct schedtune
root_schedtune = {
	.boost	= 0,
	.prefer_idle = 0,
	.prefer_perf = 0,
};

/*
 * Maximum number of boost groups to support
 * When per-task boosting is used we still allow only limited number of
 * boost groups for two main reasons:
 * 1. on a real system we usually have only few classes of workloads which
 *    make sense to boost with different values (e.g. background vs foreground
 *    tasks, interactive vs low-priority tasks)
 * 2. a limited number allows for a simpler and more memory/time efficient
 *    implementation especially for the computation of the per-CPU boost
 *    value
 */
#define BOOSTGROUPS_COUNT 5

/* Array of configured boostgroups */
static struct schedtune *allocated_group[BOOSTGROUPS_COUNT] = {
	&root_schedtune,
	NULL,
};

/* SchedTune boost groups
 * Keep track of all the boost groups which impact on CPU, for example when a
 * CPU has two RUNNABLE tasks belonging to two different boost groups and thus
 * likely with different boost values.
 * Since on each system we expect only a limited number of boost groups, here
 * we use a simple array to keep track of the metrics required to compute the
 * maximum per-CPU boosting value.
 */
struct boost_groups {
	/* Maximum boost value for all RUNNABLE tasks on a CPU */
	bool idle;
	int boost_max;
	struct {
		/* The boost for tasks on that boost group */
		int boost;
		/* Count of RUNNABLE tasks on that boost group */
		unsigned tasks;
	} group[BOOSTGROUPS_COUNT];
	/* CPU's boost group locking */
	raw_spinlock_t lock;
};

/* Boost groups affecting each CPU in the system */
DEFINE_PER_CPU(struct boost_groups, cpu_boost_groups);

static void
schedtune_cpu_update(int cpu)
{
	struct boost_groups *bg = &per_cpu(cpu_boost_groups, cpu);
	int boost_max;
	int idx;

	/* The root boost group is always active */
	boost_max = bg->group[0].boost;
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx) {
		/*
		 * A boost group affects a CPU only if it has
		 * RUNNABLE tasks on that CPU
		 */
		if (bg->group[idx].tasks == 0)
			continue;

		boost_max = max(boost_max, bg->group[idx].boost);
	}
	/* Ensures boost_max is non-negative when all cgroup boost values
	 * are neagtive. Avoids under-accounting of cpu capacity which may cause
	 * task stacking and frequency spikes.*/
	boost_max = max(boost_max, 0);
	bg->boost_max = boost_max;
}

static int
schedtune_boostgroup_update(int idx, int boost)
{
	struct boost_groups *bg;
	int cur_boost_max;
	int old_boost;
	int cpu;

	/* Update per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);

		/*
		 * Keep track of current boost values to compute the per CPU
		 * maximum only when it has been affected by the new value of
		 * the updated boost group
		 */
		cur_boost_max = bg->boost_max;
		old_boost = bg->group[idx].boost;

		/* Update the boost value of this boost group */
		bg->group[idx].boost = boost;

		/* Check if this update increase current max */
		if (boost > cur_boost_max && bg->group[idx].tasks) {
			bg->boost_max = boost;
			trace_sched_tune_boostgroup_update(cpu, 1, bg->boost_max);
			continue;
		}

		/* Check if this update has decreased current max */
		if (cur_boost_max == old_boost && old_boost > boost) {
			schedtune_cpu_update(cpu);
			trace_sched_tune_boostgroup_update(cpu, -1, bg->boost_max);
			continue;
		}

		trace_sched_tune_boostgroup_update(cpu, 0, bg->boost_max);
	}

	return 0;
}

#define ENQUEUE_TASK  1
#define DEQUEUE_TASK -1

static inline void
schedtune_tasks_update(struct task_struct *p, int cpu, int idx, int task_count)
{
	struct boost_groups *bg = &per_cpu(cpu_boost_groups, cpu);
	int tasks = bg->group[idx].tasks + task_count;

	/* Update boosted tasks count while avoiding to make it negative */
	bg->group[idx].tasks = max(0, tasks);

	trace_sched_tune_tasks_update(p, cpu, tasks, idx,
			bg->group[idx].boost, bg->boost_max);

	/* Boost group activation or deactivation on that RQ */
	if (tasks == 1 || tasks == 0)
		schedtune_cpu_update(cpu);
}

/*
 * NOTE: This function must be called while holding the lock on the CPU RQ
 */
void schedtune_enqueue_task(struct task_struct *p, int cpu)
{
	struct boost_groups *bg = &per_cpu(cpu_boost_groups, cpu);
	unsigned long irq_flags;
	struct schedtune *st;
	int idx;

	if (unlikely(!schedtune_initialized))
		return;

	/*
	 * Boost group accouting is protected by a per-cpu lock and requires
	 * interrupt to be disabled to avoid race conditions for example on
	 * do_exit()::cgroup_exit() and task migration.
	 */
	raw_spin_lock_irqsave(&bg->lock, irq_flags);
	rcu_read_lock();

	st = task_schedtune(p);
	idx = st->idx;

	schedtune_tasks_update(p, cpu, idx, ENQUEUE_TASK);

	rcu_read_unlock();
	raw_spin_unlock_irqrestore(&bg->lock, irq_flags);
}

int schedtune_can_attach(struct cgroup_taskset *tset)
{
	struct task_struct *task;
	struct cgroup_subsys_state *css;
	struct boost_groups *bg;
	struct rq_flags rq_flags;
	unsigned int cpu;
	struct rq *rq;
	int src_bg; /* Source boost group index */
	int dst_bg; /* Destination boost group index */
	int tasks;

	if (unlikely(!schedtune_initialized))
		return 0;


	cgroup_taskset_for_each(task, css, tset) {

		/*
		 * Lock the CPU's RQ the task is enqueued to avoid race
		 * conditions with migration code while the task is being
		 * accounted
		 */
		rq = task_rq_lock(task, &rq_flags);

		if (!task->on_rq) {
			task_rq_unlock(rq, task, &rq_flags);
			continue;
		}

		/*
		 * Boost group accouting is protected by a per-cpu lock and requires
		 * interrupt to be disabled to avoid race conditions on...
		 */
		cpu = cpu_of(rq);
		bg = &per_cpu(cpu_boost_groups, cpu);
		raw_spin_lock(&bg->lock);

		dst_bg = css_st(css)->idx;
		src_bg = task_schedtune(task)->idx;

		/*
		 * Current task is not changing boostgroup, which can
		 * happen when the new hierarchy is in use.
		 */
		if (unlikely(dst_bg == src_bg)) {
			raw_spin_unlock(&bg->lock);
			task_rq_unlock(rq, task, &rq_flags);
			continue;
		}

		/*
		 * This is the case of a RUNNABLE task which is switching its
		 * current boost group.
		 */

		/* Move task from src to dst boost group */
		tasks = bg->group[src_bg].tasks - 1;
		bg->group[src_bg].tasks = max(0, tasks);
		bg->group[dst_bg].tasks += 1;

		raw_spin_unlock(&bg->lock);
		task_rq_unlock(rq, task, &rq_flags);

		/* Update CPU boost group */
		if (bg->group[src_bg].tasks == 0 || bg->group[dst_bg].tasks == 1)
			schedtune_cpu_update(task_cpu(task));

	}

	return 0;
}

void schedtune_cancel_attach(struct cgroup_taskset *tset)
{
	/* This can happen only if SchedTune controller is mounted with
	 * other hierarchies ane one of them fails. Since usually SchedTune is
	 * mouted on its own hierarcy, for the time being we do not implement
	 * a proper rollback mechanism */
	WARN(1, "SchedTune cancel attach not implemented");
}

/*
 * NOTE: This function must be called while holding the lock on the CPU RQ
 */
void schedtune_dequeue_task(struct task_struct *p, int cpu)
{
	struct boost_groups *bg = &per_cpu(cpu_boost_groups, cpu);
	unsigned long irq_flags;
	struct schedtune *st;
	int idx;

	if (unlikely(!schedtune_initialized))
		return;

	/*
	 * Boost group accouting is protected by a per-cpu lock and requires
	 * interrupt to be disabled to avoid race conditions on...
	 */
	raw_spin_lock_irqsave(&bg->lock, irq_flags);
	rcu_read_lock();

	st = task_schedtune(p);
	idx = st->idx;

	schedtune_tasks_update(p, cpu, idx, DEQUEUE_TASK);

	rcu_read_unlock();
	raw_spin_unlock_irqrestore(&bg->lock, irq_flags);
}

int schedtune_cpu_boost(int cpu)
{
	struct boost_groups *bg;

	bg = &per_cpu(cpu_boost_groups, cpu);
	return bg->boost_max;
}

int schedtune_task_boost(struct task_struct *p)
{
	struct schedtune *st;
	int task_boost;

	if (unlikely(!schedtune_initialized))
		return 0;

	/* Get task boost value */
	rcu_read_lock();
	st = task_schedtune(p);
	task_boost = st->boost;
	rcu_read_unlock();

	return task_boost;
}

int schedtune_prefer_idle(struct task_struct *p)
{
	struct schedtune *st;
	int prefer_idle;

	if (unlikely(!schedtune_initialized))
		return 0;

	/* Get prefer_idle value */
	rcu_read_lock();
	st = task_schedtune(p);
	prefer_idle = st->prefer_idle;
	rcu_read_unlock();

	return prefer_idle;
}

#ifdef CONFIG_SCHED_EMS
static atomic_t kernel_prefer_perf_req[BOOSTGROUPS_COUNT];
int kernel_prefer_perf(int grp_idx)
{
	if (grp_idx >= BOOSTGROUPS_COUNT)
		return -EINVAL;

	return atomic_read(&kernel_prefer_perf_req[grp_idx]);
}

void request_kernel_prefer_perf(int grp_idx, int enable)
{
	if (grp_idx >= BOOSTGROUPS_COUNT)
		return;

	if (enable)
		atomic_inc(&kernel_prefer_perf_req[grp_idx]);
	else
		BUG_ON(atomic_dec_return(&kernel_prefer_perf_req[grp_idx]) < 0);
}
#else
static inline int kernel_prefer_perf(int grp_idx) { return 0; }
#endif

int schedtune_prefer_perf(struct task_struct *p)
{
	struct schedtune *st;
	int prefer_perf;

	if (unlikely(!schedtune_initialized))
		return 0;

	/* Get prefer_perf value */
	rcu_read_lock();
	st = task_schedtune(p);
	prefer_perf = max(st->prefer_perf, kernel_prefer_perf(st->idx));
	rcu_read_unlock();

	return prefer_perf;
}

int schedtune_need_group_balance(struct task_struct *p)
{
	bool balance;

	if (unlikely(!schedtune_initialized))
		return 0;

	rcu_read_lock();
	balance = task_schedtune(p)->gb.need_balance;
	rcu_read_unlock();

	return balance;
}

static inline void
check_need_group_balance(int group_idx, struct group_balancer *gb)
{
	int heaviest_ratio;

	if (!gb->util) {
		gb->imbalance_count = 0;
		gb->need_balance = false;

		goto out;
	}

	heaviest_ratio = gb->heaviest_util * 100 / gb->util;

	if (gb->need_balance) {
		if (gb->util < gb->threshold || heaviest_ratio < gb->balance_ratio) {
			gb->imbalance_count = 0;
			gb->need_balance = false;
		}

		goto out;
	}

	if (gb->util >= gb->threshold && heaviest_ratio > gb->imbalance_ratio) {
		gb->imbalance_count++;

		if (gb->imbalance_count >= gb->imbalance_duration)
			gb->need_balance = true;
	} else {
		gb->imbalance_count = 0;
	}

out:
	trace_sched_tune_check_group_balance(group_idx,
				gb->imbalance_count, gb->need_balance);
}

static void __schedtune_group_util_update(struct schedtune *st)
{
	struct group_balancer *gb = &st->gb;
	unsigned long now = cpu_rq(0)->clock_task;
	struct css_task_iter it;
	struct task_struct *p;
	struct task_struct *heaviest_p = NULL;
	unsigned long util_sum = 0;
	unsigned long heaviest_util = 0;
	unsigned int total = 0, accumulated = 0;

	if (!raw_spin_trylock(&gb->lock))
		return;

	if (!gb->update_interval)
		goto out;

	if (time_before(now, gb->next_update_time))
		goto out;

	css_task_iter_start(&st->css, 0, &it);
	while ((p = css_task_iter_next(&it))) {
		unsigned long clock_task, delta, util;

		total++;

		clock_task = task_rq(p)->clock_task;
		delta = clock_task - p->se.avg.last_update_time;
		if (p->se.avg.last_update_time && delta > gb->window)
			continue;

		util = p->se.avg.util_avg;
		if (util > heaviest_util) {
			heaviest_util = util;
			heaviest_p = p;
		}

		util_sum += p->se.avg.util_avg;
		accumulated++;
	}
	css_task_iter_end(&it);

	gb->util = util_sum;
	gb->heaviest_util = heaviest_util;
	gb->next_update_time = now + gb->update_interval;

	/* if there is no task in group, heaviest_p is always NULL */
	if (heaviest_p)
		trace_sched_tune_grouputil_update(st->idx, total, accumulated,
				gb->util, heaviest_p, gb->heaviest_util);

	check_need_group_balance(st->idx, gb);
out:
	raw_spin_unlock(&gb->lock);
}

void schedtune_group_util_update(void)
{
	int idx;

	if (unlikely(!schedtune_initialized))
		return;

	rcu_read_lock();

	for (idx = 1; idx < BOOSTGROUPS_COUNT; idx++) {
		struct schedtune *st = allocated_group[idx];

		if (!st)
			continue;
		__schedtune_group_util_update(st);
	}

	rcu_read_unlock();
}

static u64
gb_util_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.util;
}

static u64
gb_heaviest_ratio_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	if (!st->gb.util)
		return 0;

	return st->gb.heaviest_util * 100 / st->gb.util;
}

static u64
gb_threshold_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.threshold;
}

static int
gb_threshold_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 threshold)
{
	struct schedtune *st = css_st(css);
	struct group_balancer *gb = &st->gb;

	raw_spin_lock(&gb->lock);
	gb->threshold = threshold;
	check_need_group_balance(st->idx, gb);
	raw_spin_unlock(&gb->lock);

	return 0;
}

static u64
gb_imbalance_ratio_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.imbalance_ratio;
}

static int
gb_imbalance_ratio_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 ratio)
{
	struct schedtune *st = css_st(css);
	struct group_balancer *gb = &st->gb;

	ratio = min_t(u64, ratio, 100);

	raw_spin_lock(&gb->lock);
	gb->imbalance_ratio = ratio;
	check_need_group_balance(st->idx, gb);
	raw_spin_unlock(&gb->lock);

	return 0;
}

static u64
gb_balance_ratio_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.balance_ratio;
}

static int
gb_balance_ratio_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 ratio)
{
	struct schedtune *st = css_st(css);
	struct group_balancer *gb = &st->gb;

	ratio = min_t(u64, ratio, 100);

	raw_spin_lock(&gb->lock);
	gb->balance_ratio = ratio;
	check_need_group_balance(st->idx, gb);
	raw_spin_unlock(&gb->lock);

	return 0;
}

static u64
gb_interval_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.update_interval / NSEC_PER_USEC;
}

static int
gb_interval_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 interval_us)
{
	struct schedtune *st = css_st(css);
	struct group_balancer *gb = &st->gb;

	raw_spin_lock(&gb->lock);
	gb->update_interval = interval_us * NSEC_PER_USEC;
	if (!interval_us) {
		gb->util = 0;
		gb->need_balance = false;
	}
	raw_spin_unlock(&gb->lock);

	return 0;
}

static u64
gb_duration_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.imbalance_duration;
}

static int
gb_duration_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 duration)
{
	struct schedtune *st = css_st(css);
	struct group_balancer *gb = &st->gb;

	raw_spin_lock(&gb->lock);
	gb->imbalance_duration = duration;
	check_need_group_balance(st->idx, gb);
	raw_spin_unlock(&gb->lock);

	return 0;
}

static u64
gb_window_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->gb.window / NSEC_PER_MSEC;
}

static int
gb_window_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 window)
{
	struct schedtune *st = css_st(css);
	struct group_balancer *gb = &st->gb;

	raw_spin_lock(&gb->lock);
	gb->window = window * NSEC_PER_MSEC;
	raw_spin_unlock(&gb->lock);

	return 0;
}

static u64
prefer_idle_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->prefer_idle;
}

static int
prefer_idle_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 prefer_idle)
{
	struct schedtune *st = css_st(css);
	st->prefer_idle = prefer_idle;

	return 0;
}

static u64
prefer_perf_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->prefer_perf;
}

static int
prefer_perf_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    u64 prefer_perf)
{
	struct schedtune *st = css_st(css);
	st->prefer_perf = prefer_perf;

	return 0;
}

static s64
boost_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct schedtune *st = css_st(css);

	return st->boost;
}

static int
boost_write(struct cgroup_subsys_state *css, struct cftype *cft,
	    s64 boost)
{
	struct schedtune *st = css_st(css);

	if (boost < 0 || boost > 100)
		return -EINVAL;

	st->boost = boost;

	/* Update CPU boost */
	schedtune_boostgroup_update(st->idx, st->boost);

	return 0;
}

static struct cftype files[] = {
	{
		.name = "boost",
		.read_s64 = boost_read,
		.write_s64 = boost_write,
	},
	{
		.name = "prefer_idle",
		.read_u64 = prefer_idle_read,
		.write_u64 = prefer_idle_write,
	},
	{
		.name = "prefer_perf",
		.read_u64 = prefer_perf_read,
		.write_u64 = prefer_perf_write,
	},
	{
		.name = "gb_util",
		.read_u64 = gb_util_read,
	},
	{
		.name = "gb_heaviest_ratio",
		.read_u64 = gb_heaviest_ratio_read,
	},
	{
		.name = "gb_threshold",
		.read_u64 = gb_threshold_read,
		.write_u64 = gb_threshold_write,
	},
	{
		.name = "gb_imbalance_ratio",
		.read_u64 = gb_imbalance_ratio_read,
		.write_u64 = gb_imbalance_ratio_write,
	},
	{
		.name = "gb_balance_ratio",
		.read_u64 = gb_balance_ratio_read,
		.write_u64 = gb_balance_ratio_write,
	},
	{
		.name = "gb_interval_us",
		.read_u64 = gb_interval_read,
		.write_u64 = gb_interval_write,
	},
	{
		.name = "gb_duration",
		.read_u64 = gb_duration_read,
		.write_u64 = gb_duration_write,
	},
	{
		.name = "gb_window_ms",
		.read_u64 = gb_window_read,
		.write_u64 = gb_window_write,
	},
	{ }	/* terminate */
};

static int
schedtune_boostgroup_init(struct schedtune *st)
{
	struct boost_groups *bg;
	int cpu;

	/* Keep track of allocated boost groups */
	allocated_group[st->idx] = st;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		bg->group[st->idx].boost = 0;
		bg->group[st->idx].tasks = 0;
	}

	return 0;
}

static void
schedtune_group_balancer_init(struct schedtune *st)
{
	raw_spin_lock_init(&st->gb.lock);

	st->gb.threshold = ULONG_MAX;
	st->gb.imbalance_ratio = 0;				/* 0% */
	st->gb.update_interval = 0;				/* disable update */
	st->gb.next_update_time = cpu_rq(0)->clock_task;

	st->gb.imbalance_duration = 0;
	st->gb.imbalance_count = 0;

	st->gb.window = 100 * NSEC_PER_MSEC;		/* 100ms */
}

static struct cgroup_subsys_state *
schedtune_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct schedtune *st;
	int idx;

	if (!parent_css)
		return &root_schedtune.css;

	/* Allow only single level hierachies */
	if (parent_css != &root_schedtune.css) {
		pr_err("Nested SchedTune boosting groups not allowed\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Allow only a limited number of boosting groups */
	for (idx = 1; idx < BOOSTGROUPS_COUNT; ++idx)
		if (!allocated_group[idx])
			break;
	if (idx == BOOSTGROUPS_COUNT) {
		pr_err("Trying to create more than %d SchedTune boosting groups\n",
		       BOOSTGROUPS_COUNT);
		return ERR_PTR(-ENOSPC);
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto out;

	schedtune_group_balancer_init(st);

	/* Initialize per CPUs boost group support */
	st->idx = idx;
	if (schedtune_boostgroup_init(st))
		goto release;

	return &st->css;

release:
	kfree(st);
out:
	return ERR_PTR(-ENOMEM);
}

static void
schedtune_boostgroup_release(struct schedtune *st)
{
	/* Reset this boost group */
	schedtune_boostgroup_update(st->idx, 0);

	/* Keep track of allocated boost groups */
	allocated_group[st->idx] = NULL;
}

static void
schedtune_css_free(struct cgroup_subsys_state *css)
{
	struct schedtune *st = css_st(css);

	schedtune_boostgroup_release(st);
	kfree(st);
}

struct cgroup_subsys schedtune_cgrp_subsys = {
	.css_alloc	= schedtune_css_alloc,
	.css_free	= schedtune_css_free,
	.can_attach     = schedtune_can_attach,
	.cancel_attach  = schedtune_cancel_attach,
	.legacy_cftypes	= files,
	.early_init	= 1,
};

static inline void
schedtune_init_cgroups(void)
{
	struct boost_groups *bg;
	int cpu;

	/* Initialize the per CPU boost groups */
	for_each_possible_cpu(cpu) {
		bg = &per_cpu(cpu_boost_groups, cpu);
		memset(bg, 0, sizeof(struct boost_groups));
		raw_spin_lock_init(&bg->lock);
	}

	pr_info("schedtune: configured to support %d boost groups\n",
		BOOSTGROUPS_COUNT);

	schedtune_initialized = true;
}

/*
 * Initialize the cgroup structures
 */
static int
schedtune_init(void)
{
	schedtune_spc_rdiv = reciprocal_value(100);
	schedtune_init_cgroups();

	return 0;
}
postcore_initcall(schedtune_init);
