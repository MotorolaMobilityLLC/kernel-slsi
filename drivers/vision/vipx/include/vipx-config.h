/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef VIPX_CONFIG_H_
#define VIPX_CONFIG_H_

/*
 * =================================================================================================
 * CONFIG - GLOBAL OPTIONS
 * =================================================================================================
 */

#define VIPX_MAX_BUFFER			16
#define VIPX_MAX_PLANE			3

#define VIPX_MAX_GRAPH			32
#define VIPX_MAX_TASK			VIPX_MAX_BUFFER

/* this macro determines schedule period, the unit is mile second */
#define VIPX_TIME_TICK			5
/*
 * =================================================================================================
 * CONFIG -PLATFORM CONFIG
 * =================================================================================================
 */
#define VIPX_AHB_BASE_ADDR 0x20200000
#define VIPX_STOP_WAIT_COUNT 200


/*
 * =================================================================================================
 * CONFIG - FEATURE ENABLE
 * =================================================================================================
 */

/* #define VIPX_DYNAMIC_RESOURCE */

/*
 * =================================================================================================
 * CONFIG - DEBUG OPTIONS
 * =================================================================================================
 */

/* #define DBG_STREAMING */
#define DBG_HISTORY
/* #define DBG_INTERFACE_ISR */
/* #define DBG_TIMEMEASURE */
#define DBG_MAP_KVADDR
/* #define DBG_RESOURCE */
#define DBG_MARKING
/* #define DBG_HW_SFR */
/* #define DBG_PRINT_TASK */
/* #define DBG_VERBOSE_IO */

//#define DUMP_DEBUG_LOG_REGION
#define DEBUG_LOG_MEMORY
//#define PRINT_DBG

#define DISABLE_VIPX_LOG 0

#if DISABLE_VIPX_LOG
#define probe_info(fmt, ...) 
#define probe_warn(fmt, args...) 
#define probe_err(fmt, args...) 
#define vipx_err_target(fmt, ...) 
#define vipx_warn_target(fmt, ...) 
#define vipx_info_target(fmt, ...) 
#define vipx_dbg_target(fmt, ...) 

#else

#define probe_info(fmt, ...)		pr_info("[V]" fmt, ##__VA_ARGS__)
#define probe_warn(fmt, args...)	pr_warning("[V][WRN]" fmt, ##args)
#define probe_err(fmt, args...) 	pr_err("[V][ERR]%s:%d:" fmt, __func__, __LINE__, ##args)

#ifdef DEBUG_LOG_MEMORY
#define vipx_err_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define vipx_warn_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define vipx_info_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#ifdef PRINT_DBG
#define vipx_dbg_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#else
#define vipx_dbg_target(fmt, ...)
#endif
#else
#define vipx_err_target(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define vipx_warn_target(fmt, ...)	pr_warning(fmt, ##__VA_ARGS__)
#define vipx_info_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define vipx_dbg_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#endif

#endif // DISABLE_ALL_LOG


#define vipx_err(fmt, args...) \
	vipx_err_target("[V][ERR]%s:%d:" fmt, __func__, __LINE__, ##args)

#define vipx_ierr(fmt, vctx, args...) \
	vipx_err_target("[V][I%d][ERR]%s:%d:" fmt, vctx->idx, __func__, __LINE__, ##args)

#define vipx_irerr(fmt, vctx, task, args...) \
	vipx_err_target("[V][I%d][F%d][ERR]%s:%d:" fmt, vctx->idx, task->id, __func__, __LINE__, ##args)

#define vipx_warn(fmt, args...) \
	vipx_warn_target("[V][WRN]%s:%d:" fmt, __func__, __LINE__, ##args)

#define vipx_iwarn(fmt, vctx, args...) \
	vipx_warn_target("[V][I%d][WRN]%s:%d:" fmt, vctx->idx, __func__, __LINE__, ##args)

#define vipx_irwarn(fmt, vctx, task, args...) \
	vipx_warn_target("[V][I%d][F%d][WRN]%s:%d:" fmt, vctx->idx, task->id, __func__, __LINE__, ##args)

#define vipx_info(fmt, args...) \
	vipx_info_target("[V]%s:%d:" fmt, __func__, __LINE__, ##args)

#define vipx_iinfo(fmt, vctx, args...) \
	vipx_info_target("[V][I%d]" fmt, vctx->idx, ##args)

#define vipx_irinfo(fmt, vctx, task, args...) \
	vipx_info_target("[V][I%d][F%d]" fmt, vctx->idx, task->id, ##args)

#define vipx_dbg(fmt, args...) \
	vipx_dbg_target("[V]" fmt, ##args)

#define vipx_idbg(fmt, vctx, args...) \
	vipx_dbg_target("[V][I%d]" fmt, vctx->idx, ##args)

#define vipx_irdbg(fmt, vctx, task, args...) \
	vipx_dbg_target("[V][I%d][F%d]" fmt, vctx->idx, task->id, ##args)

#endif
