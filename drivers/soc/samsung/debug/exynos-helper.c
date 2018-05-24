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
#include <linux/debug-snapshot-helper.h>
#include <linux/debug-snapshot.h>
#include <soc/samsung/exynos-debug.h>
#include <soc/samsung/exynos-pmu.h>
#include <soc/samsung/exynos-sdm.h>

#include <asm/cacheflush.h>

#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
#include <asm/core_regs.h>
#endif

#if defined(CONFIG_SEC_SIPC_MODEM_IF)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

#if defined(CONFIG_ACPM_DVFS)
#include <soc/samsung/acpm_ipc_ctrl.h>
#endif

static void exynos_early_panic(void *val)
{
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
	/* TODO: Something */

}

static void exynos_post_panic_exit(void *val)
{
	flush_cache_all();

#ifdef CONFIG_EXYNOS_SDM
	if (dbg_snapshot_is_scratch())
		exynos_sdm_dump_secure_region();
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
#ifdef CONFIG_EXYNOS_SNAPSHOT_WATCHDOG_RESET
	s3c2410wdt_keepalive_emergency(true, 0);
#endif
}

static void exynos_expire_watchdog(void *val)
{
#ifdef CONFIG_EXYNOS_SNAPSHOT_WATCHDOG_RESET
	s3c2410wdt_set_emergency_reset(100, 0);
#endif
}

static void exynos_stop_watchdog(void *val)
{

}

static void exynos_kick_watchdog(void *val)
{
#ifdef CONFIG_EXYNOS_SNAPSHOT_WATCHDOG_RESET
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
	.soc_save_context_entry	= exynos_save_context_entry,
	.soc_save_context_exit	= exynos_save_context_exit,
	.soc_start_watchdog	= exynos_start_watchdog,
	.soc_expire_watchdog	= exynos_expire_watchdog,
	.soc_stop_watchdog	= exynos_stop_watchdog,
	.soc_kick_watchdog	= exynos_kick_watchdog,
	.soc_smc_call		= exynos_smc,
	.soc_is_power_cpu	= exynos_is_power_cpu,
};

void __init dbg_snapshot_soc_helper_init(void)
{
	dbg_snapshot_register_soc_ops(&exynos_debug_ops);
}
