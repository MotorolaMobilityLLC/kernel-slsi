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
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/kallsyms.h>
#include <linux/input.h>
#include <linux/smc.h>
#include <linux/bitops.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/nmi.h>
#include <linux/init_task.h>
#include <linux/ftrace.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/core_regs.h>
#include <asm/cacheflush.h>

#include "debug-snapshot-local.h"

DEFINE_PER_CPU(struct pt_regs *, dss_core_reg);
DEFINE_PER_CPU(struct dbg_snapshot_mmu_reg *, dss_mmu_reg);

struct dbg_snapshot_allcorelockup_param {
	unsigned long last_pc_addr;
	unsigned long spin_pc_addr;
} dss_allcorelockup_param;

void dbg_snapshot_hook_hardlockup_entry(void *v_regs)
{
	int cpu = raw_smp_processor_id();
	unsigned int val;

	if (!dss_base.enabled)
		return;

	if (!dss_desc.hardlockup_core_mask) {
		if (dss_desc.multistage_wdt_irq &&
			!dss_desc.allcorelockup_detected) {
			/* 1st FIQ trigger */
			val = readl(dbg_snapshot_get_base_vaddr() +
				DSS_OFFSET_CORE_LAST_PC + (DSS_NR_CPUS * sizeof(unsigned long)));
			if (val == DSS_SIGN_LOCKUP || val == (DSS_SIGN_LOCKUP + 1)) {
				dss_desc.allcorelockup_detected = true;
				dss_desc.hardlockup_core_mask = GENMASK(DSS_NR_CPUS - 1, 0);
			} else {
				return;
			}
		}
	}

	/* re-check the cpu number which is lockup */
	if (dss_desc.hardlockup_core_mask & BIT(cpu)) {
		int ret;
		unsigned long last_pc;
		struct pt_regs *regs;
		unsigned long timeout = USEC_PER_SEC * 2;

		do {
			/*
			 * If one cpu is occurred to lockup,
			 * others are going to output its own information
			 * without side-effect.
			 */
			ret = do_raw_spin_trylock(&dss_desc.nmi_lock);
			if (!ret)
				udelay(1);
		} while (!ret && timeout--);

		last_pc = dbg_snapshot_get_last_pc(cpu);

		regs = (struct pt_regs *)v_regs;

		/* Replace real pc value even if it is invalid */
		regs->pc = last_pc;

		/* Then, we expect bug() function works well */
		pr_emerg("\n--------------------------------------------------------------------------\n"
			"      Debugging Information for Hardlockup core - CPU(%d), Mask:(0x%x)"
			"\n--------------------------------------------------------------------------\n\n",
			cpu, dss_desc.hardlockup_core_mask);
	}
}

void dbg_snapshot_hook_hardlockup_exit(void)
{
	int cpu = raw_smp_processor_id();

	if (!dss_base.enabled ||
		!dss_desc.hardlockup_core_mask) {
		return;
	}

	/* re-check the cpu number which is lockup */
	if (dss_desc.hardlockup_core_mask & BIT(cpu)) {
		/* clear bit to complete replace */
		dss_desc.hardlockup_core_mask &= ~(BIT(cpu));
		/*
		 * If this unlock function does not make a side-effect
		 * even it's not lock
		 */
		do_raw_spin_unlock(&dss_desc.nmi_lock);
	}
}

void dbg_snapshot_recall_hardlockup_core(void)
{
	int i;
#ifdef SMC_CMD_KERNEL_PANIC_NOTICE
	int ret;
#endif
	unsigned long cpu_mask = 0, tmp_bit = 0;
	unsigned long last_pc_addr = 0, timeout;

	if (dss_desc.allcorelockup_detected) {
		pr_emerg("debug-snapshot: skip recall hardlockup for dump of each core\n");
		goto out;
	}

	for (i = 0; i < DSS_NR_CPUS; i++) {
		if (i == raw_smp_processor_id())
			continue;
		tmp_bit = cpu_online_mask->bits[DSS_NR_CPUS/SZ_64] & (1 << i);
		if (tmp_bit)
			cpu_mask |= tmp_bit;
	}

	if (!cpu_mask)
		goto out;

	last_pc_addr = dbg_snapshot_get_last_pc_paddr();

	pr_emerg("debug-snapshot: core hardlockup mask information: 0x%lx\n", cpu_mask);
	dss_desc.hardlockup_core_mask = cpu_mask;

#ifdef SMC_CMD_KERNEL_PANIC_NOTICE
	/* Setup for generating NMI interrupt to unstopped CPUs */
	ret = dss_soc_ops->soc_smc_call(SMC_CMD_KERNEL_PANIC_NOTICE,
				 cpu_mask,
				 (unsigned long)dbg_snapshot_bug_func,
				 last_pc_addr);
	if (ret) {
		pr_emerg("debug-snapshot: failed to generate NMI, "
			 "not support to dump information of core\n");
		dss_desc.hardlockup_core_mask = 0;
		goto out;
	}
#endif
	/* Wait up to 3 seconds for NMI interrupt */
	timeout = USEC_PER_SEC * 3;
	while (dss_desc.hardlockup_core_mask != 0 && timeout--)
		udelay(1);
out:
	return;
}

void dbg_snapshot_save_system(void *unused)
{
	struct dbg_snapshot_mmu_reg *mmu_reg;

	if (!dbg_snapshot_get_enable("header"))
		return;

	mmu_reg = per_cpu(dss_mmu_reg, raw_smp_processor_id());
#ifdef CONFIG_ARM64
	asm("mrs x1, SCTLR_EL1\n\t"		/* SCTLR_EL1 */
		"str x1, [%0]\n\t"
		"mrs x1, TTBR0_EL1\n\t"		/* TTBR0_EL1 */
		"str x1, [%0,#8]\n\t"
		"mrs x1, TTBR1_EL1\n\t"		/* TTBR1_EL1 */
		"str x1, [%0,#16]\n\t"
		"mrs x1, TCR_EL1\n\t"		/* TCR_EL1 */
		"str x1, [%0,#24]\n\t"
		"mrs x1, ESR_EL1\n\t"		/* ESR_EL1 */
		"str x1, [%0,#32]\n\t"
		"mrs x1, FAR_EL1\n\t"		/* FAR_EL1 */
		"str x1, [%0,#40]\n\t"
		/* Don't populate AFSR0_EL1 and AFSR1_EL1 */
		"mrs x1, CONTEXTIDR_EL1\n\t"	/* CONTEXTIDR_EL1 */
		"str x1, [%0,#48]\n\t"
		"mrs x1, TPIDR_EL0\n\t"		/* TPIDR_EL0 */
		"str x1, [%0,#56]\n\t"
		"mrs x1, TPIDRRO_EL0\n\t"		/* TPIDRRO_EL0 */
		"str x1, [%0,#64]\n\t"
		"mrs x1, TPIDR_EL1\n\t"		/* TPIDR_EL1 */
		"str x1, [%0,#72]\n\t"
		"mrs x1, MAIR_EL1\n\t"		/* MAIR_EL1 */
		"str x1, [%0,#80]\n\t"
		"mrs x1, ELR_EL1\n\t"		/* ELR_EL1 */
		"str x1, [%0, #88]\n\t"
		"mrs x1, SP_EL0\n\t"		/* SP_EL0 */
		"str x1, [%0, #96]\n\t" :	/* output */
		: "r"(mmu_reg)			/* input */
		: "%x1", "memory"			/* clobbered register */
	);
#else
	asm("mrc    p15, 0, r1, c1, c0, 0\n\t"	/* SCTLR */
	    "str r1, [%0]\n\t"
	    "mrc    p15, 0, r1, c2, c0, 0\n\t"	/* TTBR0 */
	    "str r1, [%0,#4]\n\t"
	    "mrc    p15, 0, r1, c2, c0,1\n\t"	/* TTBR1 */
	    "str r1, [%0,#8]\n\t"
	    "mrc    p15, 0, r1, c2, c0,2\n\t"	/* TTBCR */
	    "str r1, [%0,#12]\n\t"
	    "mrc    p15, 0, r1, c3, c0,0\n\t"	/* DACR */
	    "str r1, [%0,#16]\n\t"
	    "mrc    p15, 0, r1, c5, c0,0\n\t"	/* DFSR */
	    "str r1, [%0,#20]\n\t"
	    "mrc    p15, 0, r1, c6, c0,0\n\t"	/* DFAR */
	    "str r1, [%0,#24]\n\t"
	    "mrc    p15, 0, r1, c5, c0,1\n\t"	/* IFSR */
	    "str r1, [%0,#28]\n\t"
	    "mrc    p15, 0, r1, c6, c0,2\n\t"	/* IFAR */
	    "str r1, [%0,#32]\n\t"
	    /* Don't populate DAFSR and RAFSR */
	    "mrc    p15, 0, r1, c10, c2,0\n\t"	/* PMRRR */
	    "str r1, [%0,#44]\n\t"
	    "mrc    p15, 0, r1, c10, c2,1\n\t"	/* NMRRR */
	    "str r1, [%0,#48]\n\t"
	    "mrc    p15, 0, r1, c13, c0,0\n\t"	/* FCSEPID */
	    "str r1, [%0,#52]\n\t"
	    "mrc    p15, 0, r1, c13, c0,1\n\t"	/* CONTEXT */
	    "str r1, [%0,#56]\n\t"
	    "mrc    p15, 0, r1, c13, c0,2\n\t"	/* URWTPID */
	    "str r1, [%0,#60]\n\t"
	    "mrc    p15, 0, r1, c13, c0,3\n\t"	/* UROTPID */
	    "str r1, [%0,#64]\n\t"
	    "mrc    p15, 0, r1, c13, c0,4\n\t"	/* POTPIDR */
	    "str r1, [%0,#68]\n\t" :		/* output */
	    : "r"(mmu_reg)			/* input */
	    : "%r1", "memory"			/* clobbered register */
	);
#endif
}

int dbg_snapshot_dump(void)
{
	/*
	 *  Output CPU Memory Error syndrome Register
	 *  CPUMERRSR, L2MERRSR
	 */
#ifdef CONFIG_ARM64
	unsigned long reg1, reg2, reg3;

	if (read_cpuid_implementor() == ARM_CPU_IMP_SEC) {
		switch (read_cpuid_part_number()) {
		case ARM_CPU_PART_MONGOOSE:
		case ARM_CPU_PART_MEERKAT:
			asm ("mrs %0, S3_1_c15_c2_0\n\t"
					"mrs %1, S3_1_c15_c2_4\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("FEMERR0SR: %016lx, FEMERR1SR: %016lx\n", reg1, reg2);
			asm ("mrs %0, S3_1_c15_c2_1\n\t"
					"mrs %1, S3_1_c15_c2_5\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("LSMERR0SR: %016lx, LSMERR1SR: %016lx\n", reg1, reg2);
			asm ("mrs %0, S3_1_c15_c2_2\n\t"
					"mrs %1, S3_1_c15_c2_6\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("TBWMERR0SR: %016lx, TBWMERR1SR: %016lx\n", reg1, reg2);
			asm ("mrs %0, S3_1_c15_c2_3\n\t"
					"mrs %1, S3_1_c15_c2_7\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("L2MERR0SR: %016lx, L2MERR1SR: %016lx\n", reg1, reg2);

			/* L3 MERR */
			asm ("msr S3_1_c15_c7_1, %0\n\t"
					"isb\n"
					:: "r" (0));
			asm ("mrs %0, S3_1_c15_c3_0\n\t"
					"mrs %1, S3_1_c15_c3_4\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("BANK0 L3MERR0SR: %016lx, L3MERR1SR: %016lx\n", reg1, reg2);
			asm ("msr S3_1_c15_c7_1, %0\n\t"
					"isb\n"
					:: "r" (1));
			asm ("mrs %0, S3_1_c15_c3_0\n\t"
					"mrs %1, S3_1_c15_c3_4\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("BANK1 L3MERR0SR: %016lx, L3MERR1SR: %016lx\n", reg1, reg2);
			asm ("msr S3_1_c15_c7_1, %0\n\t"
					"isb\n"
					:: "r" (2));
			asm ("mrs %0, S3_1_c15_c3_0\n\t"
					"mrs %1, S3_1_c15_c3_4\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("BANK2 L3MERR0SR: %016lx, L3MERR1SR: %016lx\n", reg1, reg2);
			asm ("msr S3_1_c15_c7_1, %0\n\t"
					"isb\n"
					:: "r" (3));
			asm ("mrs %0, S3_1_c15_c3_0\n\t"
					"mrs %1, S3_1_c15_c3_4\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("BANK3 L3MERR0SR: %016lx, L3MERR1SR: %016lx\n", reg1, reg2);

			break;
		default:
			break;
		}
	} else {
		switch (read_cpuid_part_number()) {
		case ARM_CPU_PART_CORTEX_A73:
			asm ("mrs %0, S3_1_c15_c2_3\n" : "=r" (reg1));
			pr_emerg("L2MERRSR: %016lx\n", reg1);
			break;
		case ARM_CPU_PART_CORTEX_A57:
		case ARM_CPU_PART_CORTEX_A53:
			asm ("mrs %0, S3_1_c15_c2_2\n\t"
					"mrs %1, S3_1_c15_c2_3\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("CPUMERRSR: %016lx, L2MERRSR: %016lx\n", reg1, reg2);
			break;
		case ARM_CPU_PART_ANANKE:
			asm ("HINT #16");
			asm ("mrs %0, S3_0_c12_c1_1\n" : "=r" (reg1)); /* read DISR_EL1 */
			pr_emerg("DISR_EL1: %016lx\n", reg1);

			asm ("msr S3_0_c5_c3_1, %0\n"  :: "r" (0)); /* set 1st ERRSELR_EL1 */

			asm ("mrs %0, S3_0_c5_c4_2\n"
					"mrs %1, S3_0_c5_c4_3\n"
					"mrs %2, S3_0_c5_c5_0\n"
					: "=r" (reg1), "=r" (reg2), "=r" (reg3));
			pr_emerg("1st : ERXSTATUS_EL1: %016lx, ERXADDR_EL1: %016lx, "
					"ERXMISC0_EL1: %016lx\n", reg1, reg2, reg3);

			asm ("msr S3_0_c5_c3_1, %0\n"  :: "r" (1)); /* set 2nd ERRSELR_EL1 */

			asm ("mrs %0, S3_0_c5_c4_2\n"
					"mrs %1, S3_0_c5_c4_3\n"
					"mrs %2, S3_0_c5_c5_0\n"
					: "=r" (reg1), "=r" (reg2), "=r" (reg3));
			pr_emerg("2nd : ERXSTATUS_EL1: %016lx, ERXADDR_EL1: %016lx, "
					"ERXMISC0_EL1: %016lx\n", reg1, reg2, reg3);

			break;
		default:
			break;
		}
	}
#else
	unsigned long reg0;
	asm ("mrc p15, 0, %0, c0, c0, 0\n": "=r" (reg0));
	if (((reg0 >> 4) & 0xFFF) == 0xC0F) {
		/*  Only Cortex-A15 */
		unsigned long reg1, reg2, reg3;
		asm ("mrrc p15, 0, %0, %1, c15\n\t"
			"mrrc p15, 1, %2, %3, c15\n"
			: "=r" (reg0), "=r" (reg1),
			"=r" (reg2), "=r" (reg3));
		pr_emerg("CPUMERRSR: %08lx_%08lx, L2MERRSR: %08lx_%08lx\n",
				reg1, reg0, reg3, reg2);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(dbg_snapshot_dump);

int dbg_snapshot_save_core(void *v_regs)
{
	struct pt_regs *regs = (struct pt_regs *)v_regs;
	struct pt_regs *core_reg =
			per_cpu(dss_core_reg, smp_processor_id());

	if(!dbg_snapshot_get_enable("header"))
		return 0;

	if (!regs) {
		asm("str x0, [%0, #0]\n\t"
		    "mov x0, %0\n\t"
		    "str x1, [x0, #8]\n\t"
		    "str x2, [x0, #16]\n\t"
		    "str x3, [x0, #24]\n\t"
		    "str x4, [x0, #32]\n\t"
		    "str x5, [x0, #40]\n\t"
		    "str x6, [x0, #48]\n\t"
		    "str x7, [x0, #56]\n\t"
		    "str x8, [x0, #64]\n\t"
		    "str x9, [x0, #72]\n\t"
		    "str x10, [x0, #80]\n\t"
		    "str x11, [x0, #88]\n\t"
		    "str x12, [x0, #96]\n\t"
		    "str x13, [x0, #104]\n\t"
		    "str x14, [x0, #112]\n\t"
		    "str x15, [x0, #120]\n\t"
		    "str x16, [x0, #128]\n\t"
		    "str x17, [x0, #136]\n\t"
		    "str x18, [x0, #144]\n\t"
		    "str x19, [x0, #152]\n\t"
		    "str x20, [x0, #160]\n\t"
		    "str x21, [x0, #168]\n\t"
		    "str x22, [x0, #176]\n\t"
		    "str x23, [x0, #184]\n\t"
		    "str x24, [x0, #192]\n\t"
		    "str x25, [x0, #200]\n\t"
		    "str x26, [x0, #208]\n\t"
		    "str x27, [x0, #216]\n\t"
		    "str x28, [x0, #224]\n\t"
		    "str x29, [x0, #232]\n\t"
		    "str x30, [x0, #240]\n\t" :
		    : "r"(core_reg));
		core_reg->sp = core_reg->regs[29];
		core_reg->pc =
			(unsigned long)(core_reg->regs[30] - sizeof(unsigned int));
	} else {
		memcpy(core_reg, regs, sizeof(struct user_pt_regs));
	}

	pr_emerg("debug-snapshot: core register saved(CPU:%d)\n",
						smp_processor_id());
	return 0;
}
EXPORT_SYMBOL(dbg_snapshot_save_core);

int dbg_snapshot_save_context(void *v_regs)
{
	int cpu;
	unsigned long flags;
	struct pt_regs *regs = (struct pt_regs *)v_regs;

	if (unlikely(!dss_base.enabled))
		return 0;

	dss_soc_ops->soc_save_context_entry(NULL);

	cpu = smp_processor_id();
	raw_spin_lock_irqsave(&dss_desc.ctrl_lock, flags);

	/* If it was already saved the context information, it should be skipped */
	if (dbg_snapshot_get_core_panic_stat(cpu) !=  DSS_SIGN_PANIC) {
		dbg_snapshot_save_system(NULL);
		dbg_snapshot_save_core(regs);
		dbg_snapshot_dump();
		dbg_snapshot_set_core_panic_stat(DSS_SIGN_PANIC, cpu);
		pr_emerg("debug-snapshot: context saved(CPU:%d)\n", cpu);
	} else
		pr_emerg("debug-snapshot: skip context saved(CPU:%d)\n", cpu);

	raw_spin_unlock_irqrestore(&dss_desc.ctrl_lock, flags);

	dss_soc_ops->soc_save_context_exit(NULL);
	return 0;
}
EXPORT_SYMBOL(dbg_snapshot_save_context);

static void dbg_snapshot_dump_one_task_info(struct task_struct *tsk, bool is_main)
{
	char state_array[] = {'R', 'S', 'D', 'T', 't', 'Z', 'X', 'x', 'K', 'W'};
	unsigned char idx = 0;
	unsigned long state = (tsk->state & TASK_REPORT) | tsk->exit_state;
	unsigned long wchan;
	unsigned long pc = 0;
	char symname[KSYM_NAME_LEN];

	if ((tsk == NULL) || (tsk->stack == NULL))
		return;

	pc = KSTK_EIP(tsk);
	wchan = get_wchan(tsk);
	if (lookup_symbol_name(wchan, symname) < 0)
		snprintf(symname, KSYM_NAME_LEN, "%lu", wchan);

	while (state) {
		idx++;
		state >>= 1;
	}

	/*
	 * kick watchdog to prevent unexpected reset during panic sequence
	 * and it prevents the hang during panic sequence by watchedog
	 */
	touch_softlockup_watchdog();
	dss_soc_ops->soc_kick_watchdog(NULL);

	pr_info("%8d %8d %8d %16lld %c(%d) %3d  %16zx %16zx  %16zx %c %16s [%s]\n",
			tsk->pid, (int)(tsk->utime), (int)(tsk->stime),
			tsk->se.exec_start, state_array[idx], (int)(tsk->state),
			task_cpu(tsk), wchan, pc, (unsigned long)tsk,
			is_main ? '*' : ' ', tsk->comm, symname);

	if (tsk->state == TASK_RUNNING
			|| tsk->state == TASK_UNINTERRUPTIBLE
			|| tsk->mm == NULL) {
		show_stack(tsk, NULL);
		pr_info("\n");
	}
}

static inline struct task_struct *get_next_thread(struct task_struct *tsk)
{
	return container_of(tsk->thread_group.next,
				struct task_struct,
				thread_group);
}

void dbg_snapshot_dump_task_info(void)
{
	struct task_struct *frst_tsk;
	struct task_struct *curr_tsk;
	struct task_struct *frst_thr;
	struct task_struct *curr_thr;

	pr_info("\n");
	pr_info(" current proc : %d %s\n", current->pid, current->comm);
	pr_info(" ----------------------------------------------------------------------------------------------------------------------------\n");
	pr_info("     pid      uTime    sTime      exec(ns)  stat  cpu       wchan           user_pc        task_struct       comm   sym_wchan\n");
	pr_info(" ----------------------------------------------------------------------------------------------------------------------------\n");

	/* processes */
	frst_tsk = &init_task;
	curr_tsk = frst_tsk;
	while (curr_tsk != NULL) {
		dbg_snapshot_dump_one_task_info(curr_tsk,  true);
		/* threads */
		if (curr_tsk->thread_group.next != NULL) {
			frst_thr = get_next_thread(curr_tsk);
			curr_thr = frst_thr;
			if (frst_thr != curr_tsk) {
				while (curr_thr != NULL) {
					dbg_snapshot_dump_one_task_info(curr_thr, false);
					curr_thr = get_next_thread(curr_thr);
					if (curr_thr == curr_tsk)
						break;
				}
			}
		}
		curr_tsk = container_of(curr_tsk->tasks.next,
					struct task_struct, tasks);
		if (curr_tsk == frst_tsk)
			break;
	}
	pr_info(" ----------------------------------------------------------------------------------------------------------------------------\n");
}

#ifdef CONFIG_DEBUG_SNAPSHOT_CRASH_KEY
void dbg_snapshot_check_crash_key(unsigned int code, int value)
{
	static bool volup_p;
	static bool voldown_p;
	static int loopcount;

	static const unsigned int VOLUME_UP = KEY_VOLUMEUP;
	static const unsigned int VOLUME_DOWN = KEY_VOLUMEDOWN;

	if (code == KEY_POWER)
		pr_info("debug-snapshot: POWER-KEY %s\n", value ? "pressed" : "released");

	/* Enter Forced Upload
	 *  Hold volume down key first
	 *  and then press power key twice
	 *  and volume up key should not be pressed
	 */
	if (value) {
		if (code == VOLUME_UP)
			volup_p = true;
		if (code == VOLUME_DOWN)
			voldown_p = true;
		if (!volup_p && voldown_p) {
			if (code == KEY_POWER) {
				pr_info
				    ("debug-snapshot: count for entering forced upload [%d]\n",
				     ++loopcount);
				if (loopcount == 2) {
					panic("Crash Key");
				}
			}
		}
	} else {
		if (code == VOLUME_UP)
			volup_p = false;
		if (code == VOLUME_DOWN) {
			loopcount = 0;
			voldown_p = false;
		}
	}
}
#endif

void __init dbg_snapshot_allcorelockup_detector_init(void)
{
	int ret;

	if (!dss_desc.multistage_wdt_irq)
		return;

	dss_allcorelockup_param.last_pc_addr = dbg_snapshot_get_last_pc_paddr();
	dss_allcorelockup_param.spin_pc_addr = __pa_symbol(dbg_snapshot_spin_func);

	__flush_dcache_area((void *)&dss_allcorelockup_param,
			sizeof(struct dbg_snapshot_allcorelockup_param));

#ifdef SMC_CMD_LOCKUP_NOTICE
	/* Setup for generating NMI interrupt to unstopped CPUs */
	ret = dss_soc_ops->soc_smc_call(SMC_CMD_LOCKUP_NOTICE,
				 (unsigned long)dbg_snapshot_bug_func,
				 dss_desc.multistage_wdt_irq,
				 (unsigned long)(virt_to_phys)(&dss_allcorelockup_param));
#endif

	pr_emerg("debug-snapshot: %s to register all-core lockup detector - ret: %d\n",
			ret == 0 ? "success" : "failed", ret);
}

void __init dbg_snapshot_utils_init(void)
{
	size_t vaddr;
	int i;

	vaddr = dss_items[dss_desc.header_num].entry.vaddr;

	for (i = 0; i < DSS_NR_CPUS; i++) {
		per_cpu(dss_mmu_reg, i) = (struct dbg_snapshot_mmu_reg *)
					  (vaddr + DSS_HEADER_SZ +
					   i * DSS_MMU_REG_OFFSET);
		per_cpu(dss_core_reg, i) = (struct pt_regs *)
					   (vaddr + DSS_HEADER_SZ + DSS_MMU_REG_SZ +
					    i * DSS_CORE_REG_OFFSET);
	}

	/* hardlockup_detector function should be called before secondary booting */
	dbg_snapshot_allcorelockup_detector_init();
}

static int __init dbg_snapshot_utils_save_systems_all(void)
{
	smp_call_function(dbg_snapshot_save_system, NULL, 1);
	dbg_snapshot_save_system(NULL);

	return 0;
}
postcore_initcall(dbg_snapshot_utils_save_systems_all);
