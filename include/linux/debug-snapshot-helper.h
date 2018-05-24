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

#ifndef DEBUG_SNAPSHOT_HELPER_H
#define DEBUG_SNAPSHOT_HELPER_H

struct dbg_snapshot_helper_ops {
	void (*soc_early_panic)(void *);

	void (*soc_prepare_panic_entry)(void *);
	void (*soc_prepare_panic_exit)(void *);

	void (*soc_post_panic_entry)(void *);
	void (*soc_post_panic_exit)(void *);

	void (*soc_post_reboot_entry)(void *);
	void (*soc_post_reboot_exit)(void *);

	void (*soc_save_context_entry)(void *);
	void (*soc_save_context_exit)(void *);

	void (*soc_start_watchdog)(void *);
	void (*soc_expire_watchdog)(void *);
	void (*soc_stop_watchdog)(void *);
	void (*soc_kick_watchdog)(void *);

	int (*soc_is_power_cpu)(void *);
	int (*soc_smc_call)(unsigned long, unsigned long, unsigned long, unsigned long);
};

extern void dbg_snapshot_register_soc_ops(struct dbg_snapshot_helper_ops *ops);
extern bool dbg_snapshot_is_scratch(void);
#endif
