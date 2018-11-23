/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 *
 * Exynos - Support SoC For Debug SnapShot
 * Author: Hosung Kim <hosung0.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/smc.h>
#include <linux/reboot.h>
#include <linux/debug-snapshot-helper.h>
#include <linux/debug-snapshot.h>
#include <soc/samsung/exynos-debug.h>
#include <soc/samsung/exynos-pmu.h>
#include <soc/samsung/exynos-sdm.h>

#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/core_regs.h>

#ifdef CONFIG_EXYNOS_BCM_DBG
#include <soc/samsung/exynos-bcm_dbg.h>
#endif

#if defined(CONFIG_SEC_SIPC_MODEM_IF)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

#if defined(CONFIG_ACPM_DVFS)
#include <soc/samsung/acpm_ipc_ctrl.h>
#endif

extern void (*arm_pm_restart)(enum reboot_mode str, const char *cmd);

static void exynos_early_panic(void *val)
{
	exynos_bcm_dbg_stop(PANIC_HANDLE);
}

static void exynos_prepare_panic_entry(void *val)
{
	/* TODO: Something */
}

static void exynos_prepare_panic_exit(void *val)
{
#if defined(CONFIG_SEC_SIPC_MODEM_IF)
	modem_send_panic_noti_ext();
#endif
#if defined(CONFIG_ACPM_DVFS)
	acpm_stop_log();
#endif
}

static void exynos_post_panic_entry(void *val)
{
	flush_cache_all();

#ifdef CONFIG_EXYNOS_SDM
	if (dbg_snapshot_is_scratch())
		exynos_sdm_dump_secure_region();
#endif
}

static void exynos_post_panic_exit(void *val)
{
#ifdef CONFIG_DEBUG_SNAPSHOT_PANIC_REBOOT
	arm_pm_restart(REBOOT_COLD, "panic");
#endif
}

static void exynos_post_reboot_entry(void *val)
{
	/* TODO: Something */

}

static void exynos_post_reboot_exit(void *val)
{
	flush_cache_all();
}

static void exynos_save_core(void *val)
{
	struct pt_regs *core_reg = (struct pt_regs *)val;

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
}

static void exynos_save_system(void *val)
{
	struct dbg_snapshot_mmu_reg *mmu_reg =
			(struct dbg_snapshot_mmu_reg *)val;

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

static void exynos_dump_info(void *val)
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
		case ARM_CPU_PART_CORTEX_A57:
		case ARM_CPU_PART_CORTEX_A53:
			asm ("mrs %0, S3_1_c15_c2_2\n\t"
					"mrs %1, S3_1_c15_c2_3\n"
					: "=r" (reg1), "=r" (reg2));
			pr_emerg("CPUMERRSR: %016lx, L2MERRSR: %016lx\n", reg1, reg2);
			break;
		case ARM_CPU_PART_ANANKE:
		case ARM_CPU_PART_CORTEX_A75:
			asm ("HINT #16");
			asm ("mrs %0, S3_0_c12_c1_1\n" : "=r" (reg1)); /* read DISR_EL1 */
			pr_emerg("DISR_EL1: %016lx\n", reg1);

			asm ("msr S3_0_c5_c3_1, %0\n"
					"isb\n"
					:: "r" (0)); /* set 1st ERRSELR_EL1 */

			asm ("mrs %0, S3_0_c5_c4_2\n"
					"mrs %1, S3_0_c5_c4_3\n"
					"mrs %2, S3_0_c5_c5_0\n"
					: "=r" (reg1), "=r" (reg2), "=r" (reg3));
			pr_emerg("1st : ERXSTATUS_EL1: %016lx, ERXADDR_EL1: %016lx, "
					"ERXMISC0_EL1: %016lx\n", reg1, reg2, reg3);

			asm ("msr S3_0_c5_c3_1, %0\n"
					"isb\n"
					:: "r" (1)); /* set 2nd ERRSELR_EL1 */

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
}

static void exynos_save_context_entry(void *val)
{
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	exynos_trace_stop();
#endif
}

static void exynos_save_context_exit(void *val)
{
	flush_cache_all();
}

static void exynos_start_watchdog(void *val)
{
#ifdef CONFIG_S3C2410_WATCHDOG
	s3c2410wdt_keepalive_emergency(true, 0);
#endif
}

static void exynos_expire_watchdog(void *val)
{
#ifdef CONFIG_S3C2410_WATCHDOG
	s3c2410wdt_set_emergency_reset(100, 0);
#endif
}

static void exynos_stop_watchdog(void *val)
{

}

static void exynos_kick_watchdog(void *val)
{
#ifdef CONFIG_S3C2410_WATCHDOG
	s3c2410wdt_keepalive_emergency(false, 0);
#endif
}

static int exynos_is_power_cpu(void *cpu)
{
#ifdef CONFIG_EXYNOS_PMU
	return exynos_cpu.power_state((unsigned int)(unsigned long)cpu);
#else
	return 0;
#endif
}

struct dbg_snapshot_helper_ops exynos_debug_ops = {
	.soc_early_panic 	= exynos_early_panic,
	.soc_prepare_panic_entry = exynos_prepare_panic_entry,
	.soc_prepare_panic_exit	= exynos_prepare_panic_exit,
	.soc_post_panic_entry	= exynos_post_panic_entry,
	.soc_post_panic_exit	= exynos_post_panic_exit,
	.soc_post_reboot_entry	= exynos_post_reboot_entry,
	.soc_post_reboot_exit	= exynos_post_reboot_exit,
	.soc_save_context_entry	= exynos_save_context_entry,
	.soc_save_context_exit	= exynos_save_context_exit,
	.soc_save_core		= exynos_save_core,
	.soc_save_system	= exynos_save_system,
	.soc_dump_info		= exynos_dump_info,
	.soc_start_watchdog	= exynos_start_watchdog,
	.soc_expire_watchdog	= exynos_expire_watchdog,
	.soc_stop_watchdog	= exynos_stop_watchdog,
	.soc_kick_watchdog	= exynos_kick_watchdog,
	.soc_is_power_cpu	= exynos_is_power_cpu,
	.soc_smc_call		= exynos_smc,
};

void __init dbg_snapshot_soc_helper_init(void)
{
	dbg_snapshot_register_soc_ops(&exynos_debug_ops);
}
