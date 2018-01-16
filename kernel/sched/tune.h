
#ifdef CONFIG_SCHED_TUNE

#include <linux/reciprocal_div.h>

/*
 * System energy normalization constants
 */
struct target_nrg {
	unsigned long min_power;
	unsigned long max_power;
	struct reciprocal_value rdiv;
};

int schedtune_cpu_boost(int cpu);
int schedtune_task_boost(struct task_struct *tsk);

void schedtune_group_util_update(void);
int schedtune_need_group_balance(struct task_struct *p);

int schedtune_perf_threshold(void);

int schedtune_prefer_idle(struct task_struct *tsk);
int schedtune_prefer_perf(struct task_struct *tsk);

void schedtune_enqueue_task(struct task_struct *p, int cpu);
void schedtune_dequeue_task(struct task_struct *p, int cpu);

#else /* CONFIG_SCHED_TUNE */

#define schedtune_cpu_boost(cpu)  0
#define schedtune_task_boost(tsk) 0

#define schedtune_group_util_update() do { } while (0)
#define schedtune_need_group_balance(task) 0

#define schedtune_perf_threshold() 0

#define schedtune_prefer_idle(tsk) 0
#define schedtune_prefer_perf(tsk) 0

#define schedtune_enqueue_task(task, cpu) do { } while (0)
#define schedtune_dequeue_task(task, cpu) do { } while (0)

#endif /* CONFIG_SCHED_TUNE */
