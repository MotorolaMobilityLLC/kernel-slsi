/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Boojin Kim <boojin.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/random.h>
#include <linux/rtc.h>
#include <linux/clk.h>
#include <linux/timekeeping.h>
#ifdef CONFIG_EXYNOS_ITMON
#include <soc/samsung/exynos-itmon.h>
#endif

#ifdef CONFIG_CHRE_SENSORHUB_HAL
#include "main.h"
#endif
#include "bl.h"
#include "comms.h"
#include "chub.h"
#include "chub_ipc.h"
#include "chub_dbg.h"
#include "../../soc/samsung/cal-if/pmucal_shub.h"
#define WAIT_TRY_CNT (3)
#define WAIT_TIMEOUT_MS (1000)
enum { CHUB_ON, CHUB_OFF };
enum { C2A_ON, C2A_OFF };

/* host interface functions */
int contexthub_is_run(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return 1;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	return nanohub_irq1_fired(ipc->data);
#else
	return 1;
#endif
}

/* request contexthub to host driver */
int contexthub_request(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return 0;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	return request_wakeup_timeout(ipc->data, WAIT_TIMEOUT_MS);
#else
	return 0;
#endif
}

/* rlease contexthub to host driver */
void contexthub_release(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	release_wakeup(ipc->data);
#endif
}

static inline void contexthub_notify_host(struct contexthub_ipc_info *ipc)
{
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	nanohub_handle_irq1(ipc->data);
#else
	/* TODO */
#endif
}

#ifdef CONFIG_CHRE_SENSORHUB_HAL
/* by nanohub kernel RxBufStruct. packet header is 10 + 2 bytes to align */
struct rxbuf {
	u8 pad;
	u8 pre_preamble;
	u8 buf[PACKET_SIZE_MAX];
	u8 post_preamble;
};

static int nanohub_mailbox_open(void *data)
{
	return 0;
}

static void nanohub_mailbox_close(void *data)
{
	(void)data;
}

static int nanohub_mailbox_write(void *data, uint8_t *tx, int length,
				 int timeout)
{
	struct nanohub_data *ipc = data;

	return contexthub_ipc_write(ipc->pdata->mailbox_client, tx, length, timeout);
}

static int nanohub_mailbox_read(void *data, uint8_t *rx, int max_length,
				int timeout)
{
	struct nanohub_data *ipc = data;

	return contexthub_ipc_read(ipc->pdata->mailbox_client, rx, max_length, timeout);
}

void nanohub_mailbox_comms_init(struct nanohub_comms *comms)
{
	comms->seq = 1;
	comms->timeout_write = 544;
	comms->timeout_ack = 272;
	comms->timeout_reply = 512;
	comms->open = nanohub_mailbox_open;
	comms->close = nanohub_mailbox_close;
	comms->write = nanohub_mailbox_write;
	comms->read = nanohub_mailbox_read;
}
#endif

static int contexthub_read_process(uint8_t *rx, u8 *raw_rx, u32 size)
{
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	struct rxbuf *rxstruct;
	struct nanohub_packet *packet;

	rxstruct = (struct rxbuf *)raw_rx;
	packet = (struct nanohub_packet *)&rxstruct->pre_preamble;
	memcpy_fromio(rx, (void *)packet, size);

	return NANOHUB_PACKET_SIZE(packet->len);
#else
	memcpy_fromio(rx, (void *)raw_rx, size);
	return size;
#endif
}

static int contexthub_ipc_drv_init(struct contexthub_ipc_info *chub)
{
	struct device *chub_dev = chub->dev;
	int ret = 0;

	chub->ipc_map = ipc_get_chub_map();
	if (!chub->ipc_map)
		return -EINVAL;

	/* init debug-log */
	/* HACK for clang */
	chub->ipc_map->logbuf.eq = 0;
	chub->ipc_map->logbuf.dq = 0;
	chub->fw_log = log_register_buffer(chub_dev, 0,
					   (void *)&chub->ipc_map->logbuf.eq,
					   "fw", 1);
	if (!chub->fw_log)
		return -EINVAL;

#ifdef LOWLEVEL_DEBUG
	chub->dd_log_buffer = vmalloc(SZ_256K + sizeof(struct LOG_BUFFER *));
	chub->dd_log_buffer->index_reader = 0;
	chub->dd_log_buffer->index_writer = 0;
	chub->dd_log_buffer->size = SZ_256K;
	chub->dd_log =
	    log_register_buffer(chub_dev, 1, chub->dd_log_buffer, "dd", 0);
#endif
	ret = chub_dbg_init(chub_dev);
	if (ret)
		dev_err(chub_dev, "%s: fails. ret:%d\n", __func__);

	return ret;
}

#ifdef PACKET_LOW_DEBUG
static void debug_dumpbuf(unsigned char *buf, int len)
{
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET, 16, 1, buf, len,
		       false);
}
#endif

static inline int get_recv_channel(struct recv_ctrl *recv)
{
	int i;
	unsigned long min_order = 0;
	int min_order_evt = INVAL_CHANNEL;

	for (i = 0; i < IPC_BUF_NUM; i++) {
		if (recv->container[i]) {
			if (!min_order) {
				min_order = recv->container[i];
				min_order_evt = i;
			} else if (recv->container[i] < min_order) {
				min_order = recv->container[i];
				min_order_evt = i;
			}
		}
	}

	if (min_order_evt != INVAL_CHANNEL)
		recv->container[min_order_evt] = 0;

	return min_order_evt;
}

static inline bool read_is_locked(struct contexthub_ipc_info *ipc)
{
	return atomic_read(&ipc->read_lock.cnt) != 0;
}

static inline void read_get_locked(struct contexthub_ipc_info *ipc)
{
	atomic_inc(&ipc->read_lock.cnt);
}

static inline void read_put_unlocked(struct contexthub_ipc_info *ipc)
{
	atomic_dec(&ipc->read_lock.cnt);
}

static void enable_debug_workqueue(struct contexthub_ipc_info *ipc, enum chub_err_type err)
{
	if (err != CHUB_ERR_NANOHUB) {
		ipc->err_cnt[err]++;
		ipc->active_err |= (1 << err);

		dev_info(ipc->dev, "%s: err:%d(cnt:%d), active:0x%x\n",
			__func__, err, ipc->err_cnt[err], ipc->active_err);
	}

	if (err == CHUB_ERR_ITMON) {
		chub_dbg_dump_gpr(ipc);
		chub_dbg_dump_ram(ipc, err);
	} else {
		schedule_work(&ipc->debug_work);
	}
}

int contexthub_ipc_read(struct contexthub_ipc_info *ipc, uint8_t *rx, int max_length,
				int timeout)
{
	unsigned long flag;
#ifdef USE_IPC_BUF
	int size = 0;
	int ret;

	if (!ipc->read_lock.flag) {
		spin_lock_irqsave(&ipc->read_lock.event.lock, flag);
		read_get_locked(ipc);
		ret =
			wait_event_interruptible_timeout_locked(ipc->read_lock.event,
								ipc->read_lock.flag,
								msecs_to_jiffies(timeout));
		read_put_unlocked(ipc);
		spin_unlock_irqrestore(&ipc->read_lock.event.lock, flag);
		if (ret < 0)
			dev_warn(ipc->dev,
				 "fails to get read ret:%d timeout:%d, flag:0x%x",
				 ret, timeout, ipc->read_lock.flag);

		if (!ipc->read_lock.flag)
			goto fail_get_channel;
	}

	ipc->read_lock.flag--;
	size = ipc_read_data(IPC_DATA_C2A, ipc->rxbuf);
	if (size)
		return contexthub_read_process(rx, ipc->rxbuf, size);
#else
	struct ipc_content *content;
	int ch = INVAL_CHANNEL;

	if (ipc->read_lock.flag) {
search_channel:
		ch = get_recv_channel(&ipc->recv_order);

		if (ch == INVAL_CHANNEL)
			goto fail_get_channel;
		else
			ipc->read_lock.flag &= ~(1 << ch);
	} else {
		spin_lock_irqsave(&ipc->read_lock.event.lock, flag);
		read_get_locked(ipc);
		ret =
		    wait_event_interruptible_timeout_locked(ipc->read_lock.event,
							    ipc->read_lock.flag,
							    msecs_to_jiffies(timeout));
		read_put_unlocked(ipc);
		spin_unlock_irqrestore(&ipc->read_lock.event.lock, flag);
		if (ret < 0)
			dev_warn(ipc->dev,
				 "fails to get read ret:%d timeout:%d, flag:0x%x",
				 ret, timeout, ipc->read_lock.flag);

		if (ipc->read_lock.flag)
			goto search_channel;
		else
			goto fail_get_channel;
	}

	content = ipc_get_addr(IPC_REG_IPC_C2A, ch);
	ipc->recv_order.container[ch] = 0;
	ipc_update_channel_status(content, CS_CHUB_OWN);

	return contexthub_read_process(rx, content->buf, content->size);
#endif

fail_get_channel:
	enable_debug_workqueue(ipc, CHUB_ERR_READ_FAIL);
	return -EINVAL;
}

int contexthub_ipc_write(struct contexthub_ipc_info *ipc,
				uint8_t *tx, int length, int timeout)
{
#ifdef USE_IPC_BUF
	int ret;

	ret = ipc_write_data(IPC_DATA_A2C, tx, (u16)length);
	if (ret) {
		pr_err("%s: fails to write data: ret:%d, len:%d errcnt:%d\n",
			__func__, ret, length, ipc->err_cnt[CHUB_ERR_WRITE_FAIL]);
		enable_debug_workqueue(ipc, CHUB_ERR_WRITE_FAIL);
		length = 0;
	}
	return length;
#else
	struct ipc_content *content =
	    ipc_get_channel(IPC_REG_IPC_A2C, CS_IDLE, CS_AP_WRITE);

	if (!content) {
		pr_err("%s: fails to get channel.\n", __func__);
		ipc_print_channel();

		return -EINVAL;
	}
	content->size = length;
	memcpy_toio(content->buf, tx, length);

	DEBUG_PRINT(KERN_DEBUG, "->W%d\n", content->num);
	if (ipc_add_evt(IPC_EVT_A2C, content->num)) {
		contexthub_ipc_write_event(ipc, MAILBOX_EVT_CHUB_ALIVE);
		length = 0;
	}
#endif
	return length;
}

static void check_rtc_time(void)
{
	struct rtc_device *chub_rtc = rtc_class_open(CONFIG_RTC_SYSTOHC_DEVICE);
	struct rtc_device *ap_rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	struct rtc_time chub_tm, ap_tm;
	time64_t chub_t, ap_t;

	rtc_read_time(ap_rtc, &chub_tm);
	rtc_read_time(chub_rtc, &ap_tm);

	chub_t = rtc_tm_sub(&chub_tm, &ap_tm);

	if (chub_t) {
		pr_info("nanohub %s: diff_time: %llu\n", __func__, chub_t);
		rtc_set_time(chub_rtc, &ap_tm);
	};

	chub_t = rtc_tm_to_time64(&chub_tm);
	ap_t = rtc_tm_to_time64(&ap_tm);
}

/* simple alive check function */
static bool contexthub_lowlevel_alive(struct contexthub_ipc_info *ipc)
{
	int val;

	ipc->chub_alive_lock.flag = 0;
	ipc_write_val(AP, sched_clock());
	ipc_hw_gen_interrupt(AP, IRQ_EVT_CHUB_ALIVE);
	val = wait_event_timeout(ipc->chub_alive_lock.event,
				 ipc->chub_alive_lock.flag,
				 msecs_to_jiffies(WAIT_TIMEOUT_MS));

	return ipc->chub_alive_lock.flag;
}

static int contexthub_wait_alive(struct contexthub_ipc_info *ipc)
{
	int trycnt = 0;

	do {
		msleep(WAIT_TIMEOUT_MS);
		contexthub_ipc_write_event(ipc, MAILBOX_EVT_CHUB_ALIVE);
		if (++trycnt > WAIT_TRY_CNT)
			break;
	} while ((atomic_read(&ipc->chub_status) != CHUB_ST_RUN));

	if (atomic_read(&ipc->chub_status) == CHUB_ST_RUN) {
		return 0;
	} else {
		dev_warn(ipc->dev, "%s fails. contexthub status is %d\n",
			 __func__, atomic_read(&ipc->chub_status));
		return -ETIMEDOUT;
	}
}

static int contexthub_hw_reset(struct contexthub_ipc_info *ipc,
				 enum mailbox_event event)
{
	u32 val;
	int trycnt = 0;
	int ret = 0;
	int i;

	/* clear ipc value */
	ipc_init();

	atomic_set(&ipc->wakeup_chub, CHUB_OFF);
	atomic_set(&ipc->irq1_apInt, C2A_OFF);
	atomic_set(&ipc->read_lock.cnt, 0x0);

	/* chub err init */
	for (i = 0; i < CHUB_ERR_MAX; i++)
		ipc->err_cnt[i] = 0;

	ipc->read_lock.flag = 0;
#ifndef USE_IPC_BUF
	ipc->recv_order.order = 0;
	for (val = 0; val < IRQ_EVT_CH_MAX; val++)
		ipc->recv_order.container[val] = 0;
#endif
	ipc_hw_write_shared_reg(AP, ipc->os_load, SR_BOOT_MODE);
	ipc_set_chub_clk((u32)ipc->clkrate);
	ipc_set_chub_bootmode(BOOTMODE_COLD);

	switch (event) {
	case MAILBOX_EVT_POWER_ON:
#ifdef NEED_TO_RTC_SYNC
		check_rtc_time();
#endif
		if (atomic_read(&ipc->chub_status) == CHUB_ST_NO_POWER) {
			atomic_set(&ipc->chub_status, CHUB_ST_POWER_ON);

			/* enable Dump GRP */
			IPC_HW_WRITE_DUMPGPR_CTRL(ipc->chub_dumpgrp, 0x1);

#if defined(CONFIG_SOC_EXYNOS9610)
			/* cmu cm4 clock - gating */
			val = __raw_readl(ipc->cmu_chub_qch +
					REG_QCH_CON_CM4_SHUB_QCH);
			val &= ~(IGNORE_FORCE_PM_EN | CLOCK_REQ | ENABLE);
			__raw_writel((val | IGNORE_FORCE_PM_EN),
				     ipc->cmu_chub_qch +
				     REG_QCH_CON_CM4_SHUB_QCH);
#endif
			/* pmu reset-release on CHUB */
			val = __raw_readl(ipc->pmu_chub_reset +
					REG_CHUB_RESET_CHUB_OPTION);
			__raw_writel((val | CHUB_RESET_RELEASE_VALUE),
				     ipc->pmu_chub_reset +
				     REG_CHUB_RESET_CHUB_OPTION);

#if defined(CONFIG_SOC_EXYNOS9610)
			/* check chub cpu status */
			do {
				val = __raw_readl(ipc->pmu_chub_reset +
						REG_CHUB_RESET_CHUB_CONFIGURATION);
				msleep(WAIT_TIMEOUT_MS);
				if (++trycnt > WAIT_TRY_CNT) {
					dev_warn(ipc->dev,
						"chub cpu status is not set correctly\n");
					break;
				}
			} while ((val & 0x1) == 0x0);

			/* cmu cm4 clock - release */
			val = __raw_readl(ipc->cmu_chub_qch +
					REG_QCH_CON_CM4_SHUB_QCH);
			val &= ~(IGNORE_FORCE_PM_EN | CLOCK_REQ | ENABLE);
			__raw_writel((val | IGNORE_FORCE_PM_EN | CLOCK_REQ),
				     ipc->cmu_chub_qch +
				    REG_QCH_CON_CM4_SHUB_QCH);

			val = __raw_readl(ipc->cmu_chub_qch +
					REG_QCH_CON_CM4_SHUB_QCH);
			val &= ~(IGNORE_FORCE_PM_EN | CLOCK_REQ | ENABLE);
			__raw_writel((val | CLOCK_REQ),
				     ipc->cmu_chub_qch +
				    REG_QCH_CON_CM4_SHUB_QCH);
#endif
		} else {
			ret = -EINVAL;
			dev_warn(ipc->dev,
				 "fails to contexthub power on. Status is %d\n",
				 atomic_read(&ipc->chub_status));
		}
		break;
	case MAILBOX_EVT_RESET:
		ret = pmucal_shub_reset_release();
		break;
	default:
		break;
	}

	if (ret)
		return ret;
	else
		return contexthub_wait_alive(ipc);
}

static void contexthub_config_init(struct contexthub_ipc_info *chub)
{
	/* BAAW-P-APM-CHUB for CHUB to access APM_CMGP. 1 window is used */
	if (chub->chub_baaw) {
		IPC_HW_WRITE_BAAW_CHUB0(chub->chub_baaw,
					chub->baaw_info.baaw_p_apm_chub_start);
		IPC_HW_WRITE_BAAW_CHUB1(chub->chub_baaw,
					chub->baaw_info.baaw_p_apm_chub_end);
		IPC_HW_WRITE_BAAW_CHUB2(chub->chub_baaw,
					chub->baaw_info.baaw_p_apm_chub_remap);
		IPC_HW_WRITE_BAAW_CHUB3(chub->chub_baaw, BAAW_RW_ACCESS_ENABLE);
	}

	/* enable mailbox ipc */
	ipc_set_base(chub->sram);
	ipc_set_owner(AP, chub->mailbox, IPC_SRC);
}

int contexthub_ipc_write_event(struct contexthub_ipc_info *ipc,
				enum mailbox_event event)
{
	u32 val;
	int ret = 0;

	switch (event) {
	case MAILBOX_EVT_INIT_IPC:
		ret = contexthub_ipc_drv_init(ipc);
		break;
	case MAILBOX_EVT_ENABLE_IRQ:
		/* if enable, mask from CHUB IRQ, else, unmask from CHUB IRQ */
		ipc_hw_unmask_irq(AP, IRQ_EVT_C2A_INT);
		ipc_hw_unmask_irq(AP, IRQ_EVT_C2A_INTCLR);
		break;
	case MAILBOX_EVT_DISABLE_IRQ:
		ipc_hw_mask_irq(AP, IRQ_EVT_C2A_INT);
		ipc_hw_mask_irq(AP, IRQ_EVT_C2A_INTCLR);
		break;
	case MAILBOX_EVT_ERASE_SHARED:
		memset(ipc_get_base(IPC_REG_SHARED), 0, ipc_get_offset(IPC_REG_SHARED));
		break;
	case MAILBOX_EVT_DUMP_STATUS:
		break;
	case MAILBOX_EVT_WAKEUP_CLR:
		if (atomic_read(&ipc->wakeup_chub) == CHUB_ON) {
			atomic_set(&ipc->wakeup_chub, CHUB_OFF);
			ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_WAKEUP_CLR);
		}
		break;
	case MAILBOX_EVT_WAKEUP:
		if (atomic_read(&ipc->wakeup_chub) == CHUB_OFF) {
			atomic_set(&ipc->wakeup_chub, CHUB_ON);
			ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_WAKEUP);
		}
		break;
	case MAILBOX_EVT_POWER_ON:
		ret = contexthub_hw_reset(ipc, event);
		if (!ret)
			log_schedule_flush_all();
		break;
	case MAILBOX_EVT_RESET:
		if (atomic_read(&ipc->chub_status) == CHUB_ST_SHUTDOWN) {
			if (ipc->block_reset) {
				/* tzpc setting */
				ret = exynos_smc(SMC_CMD_CONN_IF,
					(EXYNOS_SHUB << 32) |
					EXYNOS_SET_CONN_TZPC, 0, 0);
				if (ret) {
					pr_err("%s: TZPC setting fail\n",
						__func__);
					return -EINVAL;
				}

				/* baaw config */
				contexthub_config_init(ipc);
			}
			ret = contexthub_hw_reset(ipc, event);
		} else {
			dev_err(ipc->dev,
				"contexthub status isn't shutdown. fails to reset\n");
			ret = -EINVAL;
		}
		break;
	case MAILBOX_EVT_SHUTDOWN:
		/* assert */
		if (ipc->block_reset) {
			/* pmu call assert */
			ret = pmucal_shub_reset_assert();
			if (ret) {
				pr_err("%s: reset assert fail\n", __func__);
				return ret;
			}

			/* pmu call reset-release_config */
			ret = pmucal_shub_reset_release_config();
			if (ret) {
				pr_err("%s: reset release cfg fail\n", __func__);
				return ret;
			}

		} else {
			/* core reset */
			ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_SHUTDOWN);
			msleep(100);	/* wait for shut down time */
			val = __raw_readl(ipc->pmu_chub_reset +
					  REG_CHUB_CPU_STATUS);
			if (val & (1 << REG_CHUB_CPU_STATUS_BIT_STANDBYWFI)) {
				val = __raw_readl(ipc->pmu_chub_reset +
						  REG_CHUB_RESET_CHUB_CONFIGURATION);
				__raw_writel(val & ~(1 << 0),
					     ipc->pmu_chub_reset +
					     REG_CHUB_RESET_CHUB_CONFIGURATION);
			} else {
				dev_err(ipc->dev,
					"fails to shutdown contexthub. cpu_status: 0x%x\n",
					val);
				return -EINVAL;
			}
		}
		atomic_set(&ipc->chub_status, CHUB_ST_SHUTDOWN);
		break;
	case MAILBOX_EVT_CHUB_ALIVE:
		val = contexthub_lowlevel_alive(ipc);
		if (val) {
			atomic_set(&ipc->chub_status, CHUB_ST_RUN);
			dev_info(ipc->dev, "chub is alive");
		} else {
			dev_err(ipc->dev,
				"chub isn't alive, should be reset. status:%d\n",
				atomic_read(&ipc->chub_status));
			if (atomic_read(&ipc->chub_status) == CHUB_ST_RUN ||
				atomic_read(&ipc->chub_status) == CHUB_ST_NO_RESPONSE) {
				atomic_set(&ipc->chub_status, CHUB_ST_NO_RESPONSE);
				enable_debug_workqueue(ipc, CHUB_ERR_CHUB_NO_RESPONSE);
			}
			ret = -EINVAL;
		}
		break;
	default:
		break;
	}

	if ((int)event < IPC_DEBUG_UTC_MAX) {
		ipc->utc_run = event;
		if ((int)event == IPC_DEBUG_UTC_TIME_SYNC) {
			check_rtc_time();
#ifdef CONFIG_CONTEXTHUB_DEBUG
			/* log_flush enable when utc_run is set */
			schedule_work(&ipc->utc_work);
#else
			ipc_write_debug_event(AP, (u32)event);
			ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_DEBUG);
#endif
		}
		ipc_write_debug_event(AP, (u32)event);
		ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_DEBUG);
	}
	return ret;
}

int contexthub_poweron(struct contexthub_ipc_info *ipc)
{
	int ret = 0;
	struct device *dev = ipc->dev;

	if (!atomic_read(&ipc->chub_status)) {
		ret = contexthub_download_image(ipc, 1);
		if (ret) {
			dev_warn(dev, "fails to download bootloader\n");
			return ret;
		}

		ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_INIT_IPC);
		if (ret) {
			dev_warn(dev, "fails to init ipc\n");
			return ret;
		}

		ret = contexthub_download_image(ipc, 0);
		if (ret) {
			dev_warn(dev, "fails to download kernel\n");
			return ret;
		}
		ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_POWER_ON);
		if (ret) {
			dev_warn(dev, "fails to poweron\n");
			return ret;
		}

		if (atomic_read(&ipc->chub_status) == CHUB_ST_RUN)
			dev_info(dev, "contexthub power-on");
		else
			dev_warn(dev, "contexthub fails to power-on");
	} else {
		ret = -EINVAL;
	}

	if (ret)
		dev_warn(dev, "fails to %s with %d. Status is %d\n",
			 __func__, ret, atomic_read(&ipc->chub_status));
	return ret;
}

int contexthub_reset(struct contexthub_ipc_info *ipc)
{
	int ret;

	dev_info(ipc->dev, "%s\n", __func__);
	if (atomic_read(&ipc->in_reset)) {
		dev_info(ipc->dev, "%s is in-progress\n", __func__);
		return -EINVAL;
	}

	atomic_inc(&ipc->in_reset);
	ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_SHUTDOWN);
	if (ret) {
		dev_err(ipc->dev, "%s: shutdonw fails, ret:%d\n", __func__, ret);
		return ret;
	}
	if (ipc->block_reset) {
		ret = contexthub_download_image(ipc, 1);
		if (!ret) {
			ret = contexthub_download_image(ipc, 0);
			if (ret) {
				dev_err(ipc->dev, "%s: download os fails\n", __func__);
				return ret;
			}
		} else {
			dev_err(ipc->dev, "%s: download bl fails\n", __func__);
			return ret;
		}
	}
	ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_RESET);
	if (ret)
		dev_err(ipc->dev, "%s: reset fails, ret:%d\n", __func__, ret);
	else
		dev_info(ipc->dev, "%s: chub reseted! (cnt:%d)\n",
			__func__, atomic_read(&ipc->in_reset));

	atomic_dec(&ipc->in_reset);
	return ret;
}

int contexthub_download_image(struct contexthub_ipc_info *ipc, int bl)
{
	const struct firmware *entry;
	int ret;
	enum ipc_region reg;
	char *name;

	if (bl) {
		ret = request_firmware(&entry, "bl.unchecked.bin", ipc->dev);
		reg = IPC_REG_BL;
	} else {
		ret = request_firmware(&entry, ipc->os_name, ipc->dev);
		reg = IPC_REG_OS;
	}

	if (ret) {
		dev_err(ipc->dev, "%s, bl(%d) request_firmware failed\n",
			bl, __func__);
		return ret;
	}
	memcpy(ipc_get_base(reg), entry->data, entry->size);
	dev_info(ipc->dev, "%s: bl:%d, bin(size:0x%x) on %lx\n",
		 __func__, bl, (int)entry->size,
		 (unsigned long)ipc_get_base(reg));
	release_firmware(entry);

	return 0;
}

int contexthub_download_bl(struct contexthub_ipc_info *ipc)
{
	int ret;

	ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_SHUTDOWN);

	if (!ret)
		ret = contexthub_download_image(ipc, 1);

	if (!ret)
		ret = contexthub_download_image(ipc, 0);

	if (!ret)
		ret = contexthub_ipc_write_event(ipc, MAILBOX_EVT_RESET);

	return ret;
}

int contexthub_download_kernel(struct contexthub_ipc_info *ipc)
{
	return contexthub_download_image(ipc, 0);
}

#ifdef CONFIG_CONTEXTHUB_DEBUG
static void handle_utc_work_func(struct work_struct *work)
{
	struct contexthub_ipc_info *ipc =
	    container_of(work, struct contexthub_ipc_info, utc_work);
	int trycnt = 0;

#ifdef USE_TIME_SYNC
	while (ipc->utc_run) {
		msleep(20000);
		ipc_write_val(AP, sched_clock());
		ipc_write_debug_event(AP, ipc->utc_run);
		ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_DEBUG);
		if (!(++trycnt % 10))
			log_flush(ipc->fw_log);
	};
#endif

	dev_dbg(ipc->dev, "%s is done with %d try\n", __func__, trycnt);
}
#endif

#define MAX_ERR_CNT (3)
/* handle errors of chub driver and fw  */
static void handle_debug_work_func(struct work_struct *work)
{
	struct contexthub_ipc_info *ipc =
	    container_of(work, struct contexthub_ipc_info, debug_work);
	enum ipc_debug_event event = 0;
	enum chub_err_type err = 0;
	bool need_reset = 0;
	bool alive = contexthub_lowlevel_alive(ipc);
	int err = 0;
#ifdef CHUB_RESET_ENABLE
	int ret;
#endif
	int i;

	if (!alive || ipc->err_cnt[CHUB_ERR_NANOHUB_WDT]) {
		need_reset = 1;
		err = CHUB_ERR_NANOHUB_WDT;
		goto do_err_handle;
	}

	/* check chub fw error */
	event = ipc_read_debug_event(AP);
	if (event) {
		switch (event) {
		case IPC_DEBUG_CHUB_FULL_LOG:
			dev_warn(ipc->dev,
				 "Contexthub notified that logbuf is full\n");
			break;
		case IPC_DEBUG_CHUB_PRINT_LOG:
			break;
		case IPC_DEBUG_CHUB_FAULT:
			dev_warn(ipc->dev, "Contexthub notified fault\n");
			err = CHUB_ERR_NANOHUB_FAULT;
			break;
		case IPC_DEBUG_CHUB_ASSERT:
			dev_warn(ipc->dev, "Contexthub notified assert\n");
			err = CHUB_ERR_NANOHUB_ASSERT;
			break;
		case IPC_DEBUG_CHUB_ERROR:
			dev_warn(ipc->dev, "Contexthub notified error\n");
			err = CHUB_ERR_NANOHUB_ERROR;
			break;
		default:
			break;
		}

		/* clear dbg event */
		ipc_write_debug_event(AP, 0);
		if (err)
			ipc->err_cnt[err]++;
		return;
	}

do_err_handle:
	dev_info(ipc->dev, "%s: active_err:0x%x, alive:%d\n",
		__func__, ipc->active_err, alive);

	/* print error status */
	for (i = 0; i < CHUB_ERR_CHUB_MAX; i++) {
		if (ipc->active_err & (1 << i)) {
			dev_info(ipc->dev, "%s: err%d-%d, active_err:0x%x\n",
				__func__, i, ipc->err_cnt[i], ipc->active_err);
			ipc->active_err &= ~(1 << i);
			err = i;
		}
	}

	/* print comms error status */
	for (i = CHUB_ERR_MAX + 1; i < CHUB_ERR_COMMS_MAX; i++) {
		if (ipc->err_cnt[i])
			dev_info(ipc->dev, "%s: comms: err%d-%d\n",
				__func__, i, ipc->err_cnt[i]);
	}

	dev_info(ipc->dev, "%s: error:%d, alive:%d\n",
		__func__, err, alive);

	/* dump hw & sram into file */
	chub_dbg_dump_hw(ipc, err);
	if (need_reset) {
#ifdef CHUB_RESET_ENABLE
		ret = contexthub_reset(ipc);
		if (ret)
			dev_warn(ipc->dev, "%s: fails to reset %d.\n",
				__func__, ret);
		else {
			/* TODO: recovery */
			dev_info(ipc->dev, "%s: chub reset! should be recovery\n",
				__func__);
			if (CHUB_ERR_NANOHUB_WDT == CHUB_ERR_NANOHUB_WDT)
				if (ipc->irq_wdt)
					enable_irq(ipc->irq_wdt);
		}
#else
		atomic_set(&ipc->chub_status, CHUB_ST_HANG);
#endif
	} else {
		/* dump log into file: DO NOT logbuf dueto sram corruption */
		log_dump_all(err);
	}
}

static void handle_irq(struct contexthub_ipc_info *ipc, enum irq_evt_chub evt)
{
#ifndef USE_IPC_BUF
	struct ipc_content *content;
#endif
	switch (evt) {
	case IRQ_EVT_C2A_DEBUG:
		enable_debug_workqueue(ipc, CHUB_ERR_NANOHUB);
		break;
	case IRQ_EVT_C2A_INT:
		if (atomic_read(&ipc->irq1_apInt) == C2A_OFF) {
			atomic_set(&ipc->irq1_apInt, C2A_ON);
			contexthub_notify_host(ipc);
		}
		break;
	case IRQ_EVT_C2A_INTCLR:
		atomic_set(&ipc->irq1_apInt, C2A_OFF);
		break;
	default:
		if (evt < IRQ_EVT_CH_MAX) {
			int lock;

#ifdef USE_IPC_BUF
			ipc->read_lock.flag++;
#else
			content = ipc_get_addr(IPC_REG_IPC_C2A, evt);
			ipc_update_channel_status(content, CS_AP_RECV);

			if (!ipc->read_lock.flag)
				ipc->recv_order.order = 1;	/* reset order */

			if (ipc->recv_order.container[evt])
				dev_warn(ipc->dev,
					 "%s: invalid order container[%d] = %lu, status:%x\n",
					 __func__, evt,
					 ipc->recv_order.container[evt],
					 content->status);

			ipc->recv_order.container[evt] =
			    ++ipc->recv_order.order;
			ipc->read_lock.flag |= (1 << evt);

			DEBUG_PRINT(KERN_DEBUG, "<-R%d(%d)(%d)\n", evt,
				    content->size, ipc->recv_order.order);
#endif
			/* TODO: requered.. ? */
			spin_lock(&ipc->read_lock.event.lock);
			lock = read_is_locked(ipc);
			spin_unlock(&ipc->read_lock.event.lock);
			if (lock)
				wake_up_interruptible_sync(&ipc->read_lock.event);
		} else {
			dev_warn(ipc->dev, "%s: invalid %d event",
				 __func__, evt);
		}
		break;
	};
}

static irqreturn_t contexthub_irq_handler(int irq, void *data)
{
	struct contexthub_ipc_info *ipc = data;
	int start_index = ipc_hw_read_int_start_index(AP);
	unsigned int status = ipc_hw_read_int_status_reg(AP);
	struct ipc_evt_buf *cur_evt;
	enum chub_err_type err = 0;
	enum irq_chub evt = 0;
	int irq_num = IRQ_EVT_CHUB_ALIVE + start_index;

	/* chub alive interrupt handle */
	if (status & (1 << irq_num)) {
		status &= ~(1 << irq_num);
		ipc_hw_clear_int_pend_reg(AP, irq_num);
		/* set wakeup flag for chub_alive_lock */
		ipc->chub_alive_lock.flag = 1;
		wake_up(&ipc->chub_alive_lock.event);
	}

	/* chub ipc interrupt handle */
	while (status) {
		cur_evt = ipc_get_evt(IPC_EVT_C2A);

		if (cur_evt) {
			evt = cur_evt->evt;
			irq_num = cur_evt->irq + start_index;

			/* check match evtq and hw interrupt pending */
			if (!(status & (1 << irq_num))) {
				err = CHUB_ERR_EVTQ_NO_HW_TRIGGER;
				break;
			}
		} else {
			err = CHUB_ERR_EVTQ_EMTPY;
			break;
		}

		handle_irq(ipc, (u32)evt);
		ipc_hw_clear_int_pend_reg(AP, irq_num);
		status &= ~(1 << irq_num);
	}

	if (err) {
		pr_err("inval irq err(%d):start_irqnum:%d,evt(%p):%d,irq_hw:%d,status_reg:0x%x(0x%x,0x%x)\n",
		       err, start_index, cur_evt, evt, irq_num,
		       status, ipc_hw_read_int_status_reg(AP),
		       ipc_hw_read_int_gen_reg(AP));
		ipc->err_cnt[err]++;
		ipc_hw_clear_all_int_pend_reg(AP);
		enable_debug_workqueue(ipc, err);
	}
	return IRQ_HANDLED;
}

#ifdef CHUB_RESET_ENABLE
static irqreturn_t contexthub_irq_wdt_handler(int irq, void *data)
{
	struct contexthub_ipc_info *ipc = data;

	dev_info(ipc->dev, "%s calledn", __func__);
	disable_irq_nosync(ipc->irq_wdt);
	enable_debug_workqueue(ipc, CHUB_ERR_NANOHUB_WDT);

	return IRQ_HANDLED;
}
#endif

static int contexthub_get_cmgp_clocks(struct device *dev)
{
#if defined(CONFIG_SOC_EXYNOS9610)
	struct clk *clk;
	int ret = 0;

	/* RPR0521, LIS3MDL */
	clk = devm_clk_get(dev, "cmgp_usi01");
	if (IS_ERR(clk)) {
		dev_err(dev, "[nanohub] cannot get cmgp_usi01\n");
		return -ENOENT;
	}
	ret = clk_prepare(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot prepare cmgp_usi01\n");
		return ret;
	}
	ret = clk_enable(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot enable cmgp_usi01\n");
		return ret;
	}
	dev_info(dev, "cmgp_usi01(%lu) is enabled\n", clk_get_rate(clk));

	/* BMP280 */
	clk = devm_clk_get(dev, "cmgp_usi03");
	if (IS_ERR(clk)) {
		dev_err(dev, "[nanohub] cannot get cmgp_usi03\n");
		return -ENOENT;
	}
	ret = clk_prepare(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot prepare cmgp_usi03\n");
		return ret;
	}
	ret = clk_enable(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot enable cmgp_usi03\n");
		return ret;
	}
	dev_info(dev, "cmgp_usi03(%lu) is enabled\n", clk_get_rate(clk));

	clk = devm_clk_get(dev, "cmgp_i2c");
	if (IS_ERR(clk)) {
		dev_err(dev, "[nanohub] cannot get cmgp_i2c\n");
		return -ENOENT;
	}
	ret = clk_prepare(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot prepare cmgp_i2c\n");
		return ret;
	}
	ret = clk_enable(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot enable cmgp_i2c\n");
		return ret;
	}
	dev_info(dev, "cmgp_i2c(%lu) is enabled\n", clk_get_rate(clk));
#endif

	return 0;
}

#if defined(CONFIG_SOC_EXYNOS9610)
extern int cal_dll_apm_enable(void);
#endif

static __init int contexthub_ipc_hw_init(struct platform_device *pdev,
					 struct contexthub_ipc_info *chub)
{
	int ret;
	int irq;
	struct resource *res;
	const char *os;
	const char *resetmode;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct clk *clk;

	if (!node) {
		dev_err(dev, "driver doesn't support non-dt\n");
		return -ENODEV;
	}

	/* get os type from dt */
	os = of_get_property(node, "os-type", NULL);
	if (!os || !strcmp(os, "none") || !strcmp(os, "pass")) {
		dev_err(dev, "no use contexthub\n");
		chub->os_load = 0;
		return -ENODEV;
	} else {
		chub->os_load = 1;
		strcpy(chub->os_name, os);
	}

	/* get resetmode from dt */
	resetmode = of_get_property(node, "reset-mode", NULL);
	if (!resetmode || !strcmp(resetmode, "block"))
		chub->block_reset = 1;
	else
		chub->block_reset = 0;

	/* get mailbox interrupt */
	chub->irq_mailbox = irq_of_parse_and_map(node, 0);
	if (chub->irq_mailbox < 0) {
		dev_err(dev, "failed to get irq:%d\n", irq);
		return -EINVAL;
	}

	/* request irq handler */
	ret = devm_request_irq(dev, chub->irq_mailbox, contexthub_irq_handler,
			       0, dev_name(dev), chub);
	if (ret) {
		dev_err(dev, "failed to request irq:%d, ret:%d\n",
			chub->irq_mailbox, ret);
		return ret;
	}

#ifdef CHUB_RESET_ENABLE
	/* get wdt interrupt optionally */
	chub->irq_wdt = irq_of_parse_and_map(node, 1);
	if (chub->irq_wdt > 0) {
		/* request irq handler */
		ret = devm_request_irq(dev, chub->irq_wdt,
				       contexthub_irq_wdt_handler, 0,
				       dev_name(dev), chub);
		if (ret) {
			dev_err(dev, "failed to request wdt irq:%d, ret:%d\n",
				chub->irq_wdt, ret);
			return ret;
		}
	} else {
		dev_info(dev, "don't use wdt irq:%d\n", irq);
	}
#endif

	/* get MAILBOX SFR */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mailbox");
	chub->mailbox = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->mailbox)) {
		dev_err(dev, "fails to get mailbox sfr\n");
		return PTR_ERR(chub->mailbox);
	}

	/* get SRAM base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	chub->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->sram)) {
		dev_err(dev, "fails to get sram\n");
		return PTR_ERR(chub->sram);
	}

	/* get chub gpr base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dumpgpr");
	chub->chub_dumpgrp = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->chub_dumpgrp)) {
		dev_err(dev, "fails to get dumpgrp\n");
		return PTR_ERR(chub->chub_dumpgrp);
	}

	/* get pmu reset base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "chub_reset");
	chub->pmu_chub_reset = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_chub_reset)) {
		dev_err(dev, "fails to get dumpgrp\n");
		return PTR_ERR(chub->pmu_chub_reset);
	}

	/* get chub baaw base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "chub_baaw");
	chub->chub_baaw = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->chub_baaw)) {
		pr_err("driver failed to get chub_baaw\n");
		chub->chub_baaw = 0;	/* it can be set on other-side (vts) */
	}

#if defined(CONFIG_SOC_EXYNOS9610)
	/* get cmu qch base */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cmu_chub_qch");
	chub->cmu_chub_qch = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->cmu_chub_qch)) {
		pr_err("driver failed to get cmu_chub_qch\n");
		return PTR_ERR(chub->cmu_chub_qch);
	}
#endif

	/* get addresses information to set BAAW */
	if (of_property_read_u32_index
		(node, "baaw,baaw-p-apm-chub", 0,
		 &chub->baaw_info.baaw_p_apm_chub_start)) {
		dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, start\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-apm-chub", 1,
		 &chub->baaw_info.baaw_p_apm_chub_end)) {
		dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, end\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index
		(node, "baaw,baaw-p-apm-chub", 2,
		 &chub->baaw_info.baaw_p_apm_chub_remap)) {
		dev_err(&pdev->dev,
			"driver failed to get baaw-p-apm-chub, remap\n");
		return -ENODEV;
	}

#if defined(CONFIG_SOC_EXYNOS9610)
	cal_dll_apm_enable();
#endif
	clk = devm_clk_get(dev, "chub_bus");
	if (IS_ERR(clk)) {
		dev_err(dev, "[nanohub] cannot get clock\n");
		return -ENOENT;
	}
#if defined(CONFIG_SOC_EXYNOS9610)
	ret = clk_prepare(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot prepare clock\n");
		return ret;
	}

	ret = clk_enable(clk);
	if (ret) {
		dev_err(dev, "[nanohub] cannot enable clock\n");
		return ret;
	}
#endif
	chub->clkrate = clk_get_rate(clk);

	ret = contexthub_get_cmgp_clocks(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "[nanohub] contexthub_get_cmgp_clocks failed\n");
		return ret;
	}

	return 0;
}

static ssize_t chub_poweron(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	int ret = contexthub_poweron(ipc);

	if (!ret)
		ret = ipc_check_reset_valid(ipc->ipc_map);

	return ret < 0 ? ret : count;
}

static ssize_t chub_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

	if (!ipc->block_reset)
		ret = contexthub_download_image(ipc, 0);

	if (!ret)
		ret = contexthub_reset(ipc);

	if (!ret)
		ret = ipc_check_reset_valid(ipc->ipc_map);

	return ret < 0 ? ret : count;
}

static struct device_attribute attributes[] = {
	__ATTR(poweron, 0220, NULL, chub_poweron),
	__ATTR(reset, 0220, NULL, chub_reset),
};

#ifdef CONFIG_EXYNOS_ITMON
static int chub_itmon_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct contexthub_ipc_info *data = container_of(nb, struct contexthub_ipc_info, itmon_nb);
	struct itmon_notifier *itmon_data = nb_data;

	if (itmon_data && itmon_data->master &&
		((!strncmp("CM4_SHUB_CD", itmon_data->master, sizeof("CM4_SHUB_CD") - 1)) ||
		(!strncmp("CM4_SHUB_P", itmon_data->master, sizeof("CM4_SHUB_P") - 1)) ||
		(!strncmp("PDMA_SHUB", itmon_data->master, sizeof("PDMA_SHUB") - 1)))) {
		dev_info(data->dev, "%s: chub(%s) itmon detected: action:%d!!\n",
			__func__, itmon_data->master, action);
		enable_debug_workqueue(data, CHUB_ERR_ITMON);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}
#endif

static int contexthub_ipc_probe(struct platform_device *pdev)
{
	struct contexthub_ipc_info *chub;
	int need_to_free = 0;
	int ret = 0;
	int i;
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	struct iio_dev *iio_dev;
#endif
	chub = chub_dbg_get_memory(DBG_NANOHUB_DD_AREA);
	if (!chub) {
		chub =
		    devm_kzalloc(&pdev->dev, sizeof(struct contexthub_ipc_info),
				 GFP_KERNEL);
		need_to_free = 1;
	}
	if (IS_ERR(chub)) {
		dev_err(&pdev->dev, "%s failed to get ipc memory\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	/* parse dt and hw init */
	ret = contexthub_ipc_hw_init(pdev, chub);
	if (ret) {
		dev_err(&pdev->dev, "%s failed to get init hw with ret %d\n",
			__func__, ret);
		goto err;
	}

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	/* nanohub probe */
	iio_dev = nanohub_probe(&pdev->dev, NULL);
	if (IS_ERR(iio_dev))
		goto err;

	/* set wakeup irq number on nanohub driver */
	chub->data = iio_priv(iio_dev);
	nanohub_mailbox_comms_init(&chub->data->comms);
	chub->pdata = chub->data->pdata;
	chub->pdata->mailbox_client = chub;
	chub->data->irq1 = IRQ_EVT_A2C_WAKEUP;
	chub->data->irq2 = 0;
#endif

	atomic_set(&chub->chub_status, CHUB_ST_NO_POWER);
	atomic_set(&chub->in_reset, 0);
	chub->powermode = 0; /* updated by fw bl */
	chub->active_err = 0;
	chub->dev = &pdev->dev;
	platform_set_drvdata(pdev, chub);
	contexthub_config_init(chub);

	for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(chub->dev, &attributes[i]);
		if (ret)
			dev_warn(chub->dev, "Failed to create file: %s\n",
				 attributes[i].attr.name);
	}

	init_waitqueue_head(&chub->read_lock.event);
	init_waitqueue_head(&chub->chub_alive_lock.event);
	INIT_WORK(&chub->debug_work, handle_debug_work_func);
#ifdef CONFIG_CONTEXTHUB_DEBUG
	INIT_WORK(&chub->utc_work, handle_utc_work_func);
#endif
#ifdef CONFIG_EXYNOS_ITMON
	chub->itmon_nb.notifier_call = chub_itmon_notifier;
	itmon_notifier_chain_register(&chub->itmon_nb);
#endif

	dev_info(chub->dev, "%s with %s FW and %lu clk is done\n",
					__func__, chub->os_name, chub->clkrate);
	return 0;
err:
	if (chub)
		if (need_to_free)
			devm_kfree(&pdev->dev, chub);

	dev_err(&pdev->dev, "%s is fail with ret %d\n", __func__, ret);
	return ret;
}

static int contexthub_ipc_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id contexthub_ipc_match[] = {
	{.compatible = "samsung,exynos-nanohub"},
	{},
};

static struct platform_driver samsung_contexthub_ipc_driver = {
	.probe = contexthub_ipc_probe,
	.remove = contexthub_ipc_remove,
	.driver = {
		   .name = "nanohub-ipc",
		   .owner = THIS_MODULE,
		   .of_match_table = contexthub_ipc_match,
		   },
};

int nanohub_mailbox_init(void)
{
	return platform_driver_register(&samsung_contexthub_ipc_driver);
}

static void __exit nanohub_mailbox_cleanup(void)
{
	platform_driver_unregister(&samsung_contexthub_ipc_driver);
}

module_init(nanohub_mailbox_init);
module_exit(nanohub_mailbox_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Exynos contexthub mailbox Driver");
MODULE_AUTHOR("Boojin Kim <boojin.kim@samsung.com>");
