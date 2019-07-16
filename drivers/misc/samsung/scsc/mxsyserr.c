/****************************************************************************
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/kmod.h>
#include <linux/notifier.h>

#include "scsc_mx_impl.h"
#include "miframman.h"
#include "mifmboxman.h"
#include "mxman.h"
#include "srvman.h"
#include "mxmgmt_transport.h"
#include "mxlog.h"
#include "mxlogger.h"
#include "fw_panic_record.h"
#include "panicmon.h"
#include "mxproc.h"
#include "mxsyserr.h"

#include <scsc/scsc_release.h>
#include <scsc/scsc_mx.h>
#include <scsc/scsc_logring.h>

void mx_syserr_handler(struct mxman *mx, const void *message)
{
	const struct mx_syserr_msg *msg = (const struct mx_syserr_msg *)message;

	SCSC_TAG_INFO(MXMAN, "MM_SYSERR_IND len: %u, ts: 0x%08X, tf: 0x%08X, str: 0x%x, code: 0x%08x, p0: 0x%x, p1: 0x%x\n",
		msg->syserr.length,
		msg->syserr.slow_clock,
		msg->syserr.fast_clock,
		msg->syserr.string_index,
		msg->syserr.syserr_code,
		msg->syserr.param[0],
		msg->syserr.param[1]);
}
