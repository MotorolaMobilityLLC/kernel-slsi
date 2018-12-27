/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC MIPI-DSIM driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/module.h>
#include <video/mipi_display.h>
#if defined(CONFIG_CAL_IF)
#include <soc/samsung/cal-if.h>
#endif
#if defined(CONFIG_CPU_IDLE)
#include <soc/samsung/exynos-cpupm.h>
#endif
#if defined(CONFIG_SOC_EXYNOS9610)
#include <dt-bindings/clock/exynos9610.h>
#endif
#include <soc/samsung/exynos-pmu.h>
#if defined(CONFIG_SUPPORT_LEGACY_ION)
#include <linux/exynos_iovmm.h>
#endif
#include <linux/string.h>
#include <linux/of_reserved_mem.h>
#include "../../../../../mm/internal.h"

#include "decon.h"
#include "dsim.h"

int dsim_log_level = 6;

struct dsim_device *dsim_drvdata[MAX_DSIM_CNT];
EXPORT_SYMBOL(dsim_drvdata);

static char *dsim_state_names[] = {
	"INIT",
	"ON",
	"DOZE",
	"ULPS",
	"DOZE_SUSPEND",
	"OFF",
};

static int dsim_runtime_suspend(struct device *dev);
static int dsim_runtime_resume(struct device *dev);

static void dsim_dump(struct dsim_device *dsim)
{
	struct dsim_regs regs;

	dsim_info("=== DSIM SFR DUMP ===\n");

	dsim_to_regs_param(dsim, &regs);
	__dsim_dump(dsim->id, &regs);

	/* Show panel status */
	call_panel_ops(dsim, dump, dsim);
}

static void dsim_long_data_wr(struct dsim_device *dsim, unsigned long d0, u32 d1)
{
	unsigned int data_cnt = 0, payload = 0;

	/* in case that data count is more then 4 */
	for (data_cnt = 0; data_cnt < d1; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((d1 - data_cnt) < 4) {
			if ((d1 - data_cnt) == 3) {
				payload = *(u8 *)(d0 + data_cnt) |
				    (*(u8 *)(d0 + (data_cnt + 1))) << 8 |
					(*(u8 *)(d0 + (data_cnt + 2))) << 16;
			dsim_dbg("count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)));
			} else if ((d1 - data_cnt) == 2) {
				payload = *(u8 *)(d0 + data_cnt) |
					(*(u8 *)(d0 + (data_cnt + 1))) << 8;
			dsim_dbg("count = 2 payload = %x, %x %x\n", payload,
				*(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)));
			} else if ((d1 - data_cnt) == 1) {
				payload = *(u8 *)(d0 + data_cnt);
			}

			dsim_reg_wr_tx_payload(dsim->id, payload);
		/* send 4bytes per one time. */
		} else {
			payload = *(u8 *)(d0 + data_cnt) |
				(*(u8 *)(d0 + (data_cnt + 1))) << 8 |
				(*(u8 *)(d0 + (data_cnt + 2))) << 16 |
				(*(u8 *)(d0 + (data_cnt + 3))) << 24;

			dsim_dbg("count = 4 payload = %x, %x %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)),
				*(u8 *)(d0 + (data_cnt + 3)));

			dsim_reg_wr_tx_payload(dsim->id, payload);
		}
	}
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim, bool must_wait)
{
	int ret = 0;
	struct dsim_regs regs;

	if (!must_wait) {
		/* timer is running, but already command is transferred */
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			del_timer(&dsim->cmd_timer);

		dsim_dbg("%s Doesn't need to wait fifo_completion\n", __func__);
		return ret;
	} else {
		del_timer(&dsim->cmd_timer);
		dsim_dbg("%s Waiting for fifo_completion...\n", __func__);
	}

	if (!wait_for_completion_timeout(&dsim->ph_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_header_fifo_is_empty(dsim->id)) {
			reinit_completion(&dsim->ph_wr_comp);
			dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}
		ret = -ETIMEDOUT;
	}

	if (IS_DSIM_ON_STATE(dsim) && (ret == -ETIMEDOUT)) {
		dsim_err("%s have timed out\n", __func__);
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
	}
	return ret;
}

/* wait for until SFR fifo is empty */
int dsim_wait_for_cmd_done(struct dsim_device *dsim)
{
	int ret = 0;
	/* FIXME: hiber only support for DECON0 */
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	ret = dsim_wait_for_cmd_fifo_empty(dsim, true);
	mutex_unlock(&dsim->cmd_lock);

	decon_hiber_unblock(decon);

	return ret;
}

static bool dsim_fifo_empty_needed(struct dsim_device *dsim, unsigned int data_id,
	unsigned long data0)
{
	/* read case or partial update command */
	if (data_id == MIPI_DSI_DCS_READ
			|| data0 == MIPI_DCS_SET_COLUMN_ADDRESS
			|| data0 == MIPI_DCS_SET_PAGE_ADDRESS) {
		dsim_dbg("%s: id:%d, data=%ld\n", __func__, data_id, data0);
		return true;
	}

	/* Check a FIFO level whether writable or not */
	if (!dsim_reg_is_writable_fifo_state(dsim->id))
		return true;

	return false;
}

int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0, u32 d1)
{
	int ret = 0;
	bool must_wait = true;
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	if (!IS_DSIM_ON_STATE(dsim)) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}
	DPU_EVENT_LOG_CMD(&dsim->sd, id, d0);

	reinit_completion(&dsim->ph_wr_comp);
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Run write-fail dectector */
	mod_timer(&dsim->cmd_timer, jiffies + MIPI_WR_TIMEOUT);

	switch (id) {
	/* short packet types of packet types for command. */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
	case MIPI_DSI_DSC_PRA:
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, false);
		must_wait = dsim_fifo_empty_needed(dsim, id, d0);
		break;

	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, true);
		must_wait = dsim_fifo_empty_needed(dsim, id, d0);
		break;

	/* long packet types of packet types for command. */
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_DSC_PPS:
		dsim_long_data_wr(dsim, d0, d1);
		dsim_reg_wr_tx_header(dsim->id, id, d1 & 0xff,
				(d1 & 0xff00) >> 8, false);
		must_wait = dsim_fifo_empty_needed(dsim, id, *(u8 *)d0);
		break;

	default:
		dsim_info("data id %x is not supported.\n", id);
		ret = -EINVAL;
	}

	ret = dsim_wait_for_cmd_fifo_empty(dsim, must_wait);
	if (ret < 0)
		dsim_err("ID(%d): DSIM cmd wr timeout 0x%lx\n", id, d0);

err_exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

int dsim_read_data(struct dsim_device *dsim, u32 id, u32 addr, u32 cnt, u8 *buf)
{
	u32 rx_fifo, rx_size = 0;
	int i, j, ret = 0;
	u32 rx_fifo_depth = DSIM_RX_FIFO_MAX_DEPTH;
	struct decon_device *decon = get_decon_drvdata(0);
	struct dsim_regs regs;
	int ret2 = 0;

	decon_hiber_block_exit(decon);

	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		decon_hiber_unblock(decon);
		return -EINVAL;
	}

	reinit_completion(&dsim->rd_comp);

	/* Init RX FIFO before read and clear DSIM_INTSRC */
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_RX_DATA_DONE);

	/* Set the maximum packet size returned */
	dsim_write_data(dsim,
		MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, cnt, 0);

	/* Read request */
	dsim_write_data(dsim, id, addr, 0);
	if (!wait_for_completion_timeout(&dsim->rd_comp, MIPI_RD_TIMEOUT)) {
		dsim_err("MIPI DSIM read Timeout!\n");
		decon_handle_recovery(decon);
		if (decon->esd.thread) {
			ret2 = wake_up_process(decon->esd.thread);
			dsim_info("%s:%d, wakeup esd thread(%d)\n", __func__,
					__LINE__, ret2);
		}
		return -ETIMEDOUT;
	}

	mutex_lock(&dsim->cmd_lock);
	DPU_EVENT_LOG_CMD(&dsim->sd, id, (char)addr);

	do {
		rx_fifo = dsim_reg_get_rx_fifo(dsim->id);

		/* Parse the RX packet data types */
		switch (rx_fifo & 0xff) {
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			ret = dsim_reg_rx_err_handler(dsim->id, rx_fifo);
			if (ret < 0) {
				dsim_to_regs_param(dsim, &regs);
				__dsim_dump(dsim->id, &regs);
				goto exit;
			}
			break;
		case MIPI_DSI_RX_END_OF_TRANSMISSION:
			dsim_dbg("EoTp was received from LCD module.\n");
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			dsim_dbg("Short Packet was received from LCD module.\n");
			for (i = 0; i <= cnt; i++)
				buf[i] = (rx_fifo >> (8 + i * 8)) & 0xff;
			rx_size = cnt;
			break;
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			dsim_dbg("Long Packet was received from LCD module.\n");
			rx_size = (rx_fifo & 0x00ffff00) >> 8;
			dsim_dbg("rx fifo : %8x, response : %x, rx_size : %d\n",
					rx_fifo, rx_fifo & 0xff, rx_size);
			/* Read data from RX packet payload */
			for (i = 0; i < rx_size >> 2; i++) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < 4; j++)
					buf[(i*4)+j] = (u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			if (rx_size % 4) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < rx_size % 4; j++)
					buf[4 * i + j] =
						(u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			break;
		default:
			dsim_err("Packet format is invaild.\n");
			dsim_to_regs_param(dsim, &regs);
			__dsim_dump(dsim->id, &regs);
			ret = -EBUSY;
			goto exit;
		}
	} while (!dsim_reg_rx_fifo_is_empty(dsim->id) && --rx_fifo_depth);

	ret = rx_size;
	if (!rx_fifo_depth) {
		dsim_err("Check DPHY values about HS clk.\n");
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
		ret = -EBUSY;
	}
exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

static void dsim_cmd_fail_detector(unsigned long arg)
{
	struct dsim_device *dsim = (struct dsim_device *)arg;
	struct decon_device *decon = get_decon_drvdata(0);
	struct dsim_regs regs;

	decon_hiber_block(decon);

	dsim_dbg("%s +\n", __func__);
	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_err("%s: DSIM is not ready. state(%d)\n", __func__,
				dsim->state);
		goto exit;
	}

	/* If already FIFO empty even though the timer is no pending */
	if (!timer_pending(&dsim->cmd_timer)
			&& dsim_reg_header_fifo_is_empty(dsim->id)) {
		reinit_completion(&dsim->ph_wr_comp);
		dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
		goto exit;
	}

	dsim_to_regs_param(dsim, &regs);
	__dsim_dump(dsim->id, &regs);

exit:
	decon_hiber_unblock(decon);
	dsim_dbg("%s -\n", __func__);
	return;
}

#if defined(CONFIG_EXYNOS_BTS)
static void dsim_bts_print_info(struct bts_decon_info *info)
{
	int i;

	for (i = 0; i < BTS_DPP_MAX; ++i) {
		if (!info->dpp[i].used)
			continue;

		dsim_info("\t\tDPP[%d] b(%d) s(%d %d) d(%d %d %d %d) r(%d)\n",
				i, info->dpp[i].bpp,
				info->dpp[i].src_w, info->dpp[i].src_h,
				info->dpp[i].dst.x1, info->dpp[i].dst.x2,
				info->dpp[i].dst.y1, info->dpp[i].dst.y2,
				info->dpp[i].rotation);
	}
}
#endif

static void dsim_underrun_info(struct dsim_device *dsim)
{
#if defined(CONFIG_EXYNOS_BTS)
	int i, decon_cnt;
	struct decon_device *decon = get_decon_drvdata(0);

	dsim_info("\tMIF(%lu), INT(%lu), DISP(%lu)\n",
			cal_dfs_get_rate(ACPM_DVFS_MIF),
			cal_dfs_get_rate(ACPM_DVFS_INT),
			cal_dfs_get_rate(ACPM_DVFS_DISP));

	if (decon == NULL)
		return;

	decon_cnt = decon->dt.decon_cnt;

	for (i = 0; i < decon_cnt; ++i) {
		decon = get_decon_drvdata(i);

		if (decon) {
			dsim_info("\tDECON%d: bw(%u %u), disp(%u %u), p(%u)\n",
					decon->id,
					decon->bts.prev_total_bw,
					decon->bts.total_bw,
					decon->bts.prev_max_disp_freq,
					decon->bts.max_disp_freq,
					decon->bts.peak);
			dsim_bts_print_info(&decon->bts.bts_info);
		}
	}
#endif
}

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct dsim_device *dsim = dev_id;
	struct decon_device *decon = get_decon_drvdata(0);
	struct dsim_regs regs;
#ifdef CONFIG_EXYNOS_PD
	int active;
#endif

	spin_lock(&dsim->slock);

#ifdef CONFIG_EXYNOS_PD
	active = pm_runtime_active(dsim->dev);
	if (!active) {
		dsim_info("dsim power(%d), state(%d)\n", active, dsim->state);
		spin_unlock(&dsim->slock);
		return IRQ_HANDLED;
	}
#endif

	int_src = dsim_reg_get_int_and_clear(dsim->id);
	if (int_src & DSIM_INTSRC_SFR_PH_FIFO_EMPTY) {
		del_timer(&dsim->cmd_timer);
		complete(&dsim->ph_wr_comp);
		DPU_EVENT_LOG(DPU_EVT_DSIM_PH_FIFO_EMPTY, &dsim->sd, ktime_set(0, 0));
		dsim_dbg("dsim%d PH_FIFO_EMPTY irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_SFR_PL_FIFO_EMPTY)
		DPU_EVENT_LOG(DPU_EVT_DSIM_PL_FIFO_EMPTY, &dsim->sd, ktime_set(0, 0));
	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE)
		dsim_dbg("dsim%d framedone irq occurs\n", dsim->id);
	if (int_src & DSIM_INTSRC_ERR_RX_ECC)
		dsim_err("RX ECC Multibit error was detected!\n");

	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		dsim->total_underrun_cnt++;
		DPU_EVENT_LOG(DPU_EVT_DSIM_UNDER_RUN, &dsim->sd, ktime_set(0, 0));
		dsim_info("dsim%d underrun irq occurs(%d)\n", dsim->id,
				dsim->total_underrun_cnt);
		dsim_underrun_info(dsim);
		if (dsim->lcd_info.mode == DECON_VIDEO_MODE) {
			dsim_to_regs_param(dsim, &regs);
			__dsim_dump(dsim->id, &regs);
		}
	}
	if (int_src & DSIM_INTSRC_VT_STATUS) {
		dsim_dbg("dsim%d vt_status(vsync) irq occurs\n", dsim->id);
		if (decon) {
			decon->vsync.timestamp = ktime_get();
			wake_up_interruptible_all(&decon->vsync.wait);
		}
	}

	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

static void dsim_clocks_info(struct dsim_device *dsim)
{
}

static int dsim_get_clocks(struct dsim_device *dsim)
{
	dsim->res.aclk = devm_clk_get(dsim->dev, "aclk");
	if (IS_ERR_OR_NULL(dsim->res.aclk)) {
		dsim_err("failed to get aclk\n");
		return PTR_ERR(dsim->res.aclk);
	}

	return 0;
}

static int dsim_get_ddi_id(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;

	if (!dev->of_node) {
		dsim_warn("no device tree information\n");
		return -1;
	}

	dsim->ddi_id = 0;
	of_property_read_u32(dev->of_node, "ddi_id", &dsim->ddi_id);

	dsim_info("Transfered ddi id is [0x%08x]\n", dsim->ddi_id);

	return 0;
}

static int dsim_get_gpios(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct dsim_resources *res = &dsim->res;

	dsim_info("%s +\n", __func__);

	if (of_get_property(dev->of_node, "gpios", NULL) != NULL)  {
		/* panel reset */
		res->lcd_reset = of_get_gpio(dev->of_node, 0);
		if (res->lcd_reset < 0) {
			dsim_err("failed to get lcd reset GPIO");
			return -ENODEV;
		}
		res->lcd_power[0] = of_get_gpio(dev->of_node, 1);
		if (res->lcd_power[0] < 0) {
			res->lcd_power[0] = -1;
			dsim_info("This board doesn't support LCD power GPIO");
		}
		res->lcd_power[1] = of_get_gpio(dev->of_node, 2);
		if (res->lcd_power[1] < 0) {
			res->lcd_power[1] = -1;
			dsim_info("This board doesn't support 2nd LCD power GPIO");
		}
		res->lcd_power[2] = of_get_gpio(dev->of_node, 3);
		if (res->lcd_power[2] < 0) {
			res->lcd_power[2] = -1;
			dsim_info("This board doesn't support 3rd LCD power GPIO");
		}
	}

	dsim_info("%s -\n", __func__);
	return 0;
}

static int dsim_get_regulator(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct dsim_resources *res = &dsim->res;

	char *str_regulator_1p8v = NULL;
	char *str_regulator_3p3v = NULL;

	res->regulator_1p8v = NULL;
	res->regulator_3p3v = NULL;

	if(!of_property_read_string(dev->of_node, "regulator_1p8v",
				(const char **)&str_regulator_1p8v)) {
		res->regulator_1p8v = regulator_get(dev, str_regulator_1p8v);
		if (IS_ERR(res->regulator_1p8v)) {
			dsim_err("%s : dsim regulator 1.8V get failed\n", __func__);
			res->regulator_1p8v = NULL;
		}
	}

	if(!of_property_read_string(dev->of_node, "regulator_3p3v",
				(const char **)&str_regulator_3p3v)) {
		res->regulator_3p3v = regulator_get(dev, str_regulator_3p3v);
		if (IS_ERR(res->regulator_3p3v)) {
			dsim_err("%s : dsim regulator 3.3V get failed\n", __func__);
			res->regulator_3p3v = NULL;
		}
	}

	return 0;
}

int dsim_reset_panel(struct dsim_device *dsim)
{
	struct dsim_resources *res = &dsim->res;
	int ret;

	dsim_dbg("%s +\n", __func__);

	ret = gpio_request_one(res->lcd_reset, GPIOF_OUT_INIT_HIGH, "lcd_reset");
	if (ret < 0) {
		dsim_err("failed to get LCD reset GPIO\n");
		return -EINVAL;
	}

	usleep_range(5000, 6000);
	gpio_set_value(res->lcd_reset, 0);
	usleep_range(5000, 6000);
	gpio_set_value(res->lcd_reset, 1);

	gpio_free(res->lcd_reset);

	usleep_range(50000, 60000);

	dsim_dbg("%s -\n", __func__);
	return 0;
}

int dsim_set_panel_power(struct dsim_device *dsim, bool on)
{
	struct dsim_resources *res = &dsim->res;
	int ret;

	dsim_dbg("%s(%d) +\n", __func__, on);


	if (on) {
		ret = gpio_request_one(res->lcd_reset, GPIOF_OUT_INIT_LOW,
				"lcd_reset");
		if (ret < 0) {
			dsim_err("failed LCD reset off\n");
			return -EINVAL;
		}
		gpio_free(res->lcd_reset);
		usleep_range(30000, 35000);

		if (res->lcd_power[0] > 0) {
			ret = gpio_request_one(res->lcd_power[0],
					GPIOF_OUT_INIT_HIGH, "lcd_power0");
			if (ret < 0) {
				dsim_err("failed LCD power on\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[0]);
			usleep_range(10000, 11000);
		}

		if (res->lcd_power[1] > 0) {
			ret = gpio_request_one(res->lcd_power[1],
					GPIOF_OUT_INIT_HIGH, "lcd_power1");
			if (ret < 0) {
				dsim_err("failed 2nd LCD power on\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[1]);
			usleep_range(10000, 11000);
		}

		if (res->lcd_power[2] > 0) {
			ret = gpio_request_one(res->lcd_power[2],
					GPIOF_OUT_INIT_HIGH, "lcd_power2");
			if (ret < 0) {
				dsim_err("failed 3rd LCD power on\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[2]);
			usleep_range(10000, 11000);
		}

		if (res->regulator_1p8v > 0) {
			ret = regulator_enable(res->regulator_1p8v);
			if (ret) {
				dsim_err("%s : dsim regulator 1.8V enable failed\n", __func__);
				return ret;
			}
			usleep_range(5000, 6000);
		}

		if (res->regulator_3p3v > 0) {
			ret = regulator_enable(res->regulator_3p3v);
			if (ret) {
				dsim_err("%s : dsim regulator 3.3V enable failed\n", __func__);
				return ret;
			}
		}
	} else {
		if (res->lcd_power[2] > 0) {
			ret = gpio_request_one(res->lcd_power[2],
					GPIOF_OUT_INIT_LOW, "lcd_power2");
			if (ret < 0) {
				dsim_err("failed 3nd LCD power off\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[2]);
			usleep_range(5000, 6000);
		}

		if (dsim->res.pinctrl && dsim->res.lcd_reset_sleep) {
			if (pinctrl_select_state(dsim->res.pinctrl,
						dsim->res.lcd_reset_sleep)) {
				decon_err("failed to turn off dism reset\n");
			}
		}

		if (res->lcd_power[1] > 0) {
			ret = gpio_request_one(res->lcd_power[1],
					GPIOF_OUT_INIT_LOW, "lcd_power1");
			if (ret < 0) {
				dsim_err("failed 2nd LCD power off\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[1]);
			usleep_range(5000, 6000);
		}

		if (res->lcd_power[0] > 0) {
			ret = gpio_request_one(res->lcd_power[0],
					GPIOF_OUT_INIT_LOW, "lcd_power0");
			if (ret < 0) {
				dsim_err("failed LCD power off\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[0]);
			usleep_range(5000, 6000);
		}

		if (res->regulator_1p8v > 0) {
			ret = regulator_disable(res->regulator_1p8v);
			if (ret) {
				dsim_err("%s : dsim regulator 1.8V disable failed\n", __func__);
				return ret;
			}
		}

		if (res->regulator_3p3v > 0) {
			ret = regulator_disable(res->regulator_3p3v);
			if (ret) {
				dsim_err("%s : dsim regulator 3.3V disable failed\n", __func__);
				return ret;
			}
		}
	}

	dsim_dbg("%s(%d) -\n", __func__, on);

	return 0;
}

static void dsim_phy_status(struct phy *phy)
{
	void __iomem *phy_iso_regs;
	u32 phy_iso = 0;
	/* 1: Isolation bypassed, 0: Isolation enabled */

	dsim_dbg("%s, PHY count : %d\n", __func__, phy->power_count);
	phy_iso_regs = ioremap(MIPI_PHY_M4S4_CON, 0x10);
	phy_iso = readl(phy_iso_regs);
	if ((phy_iso & M4S4_TOP_ISO_BYPASS) != M4S4_TOP_ISO_BYPASS) {
		dsim_err("Isolation bypass should be set\n");
		phy_iso = M4S4_TOP_ISO_BYPASS;
		writel(phy_iso, phy_iso_regs);
		dsim_err("Isolation bypass was set\n");
	}
	iounmap(phy_iso_regs);
}

static int _dsim_enable(struct dsim_device *dsim, enum dsim_state state)
{
	bool panel_ctrl;

	if (IS_DSIM_ON_STATE(dsim)) {
		dsim_warn("%s dsim already on(%s)\n",
				__func__, dsim_state_names[dsim->state]);
		dsim->state = state;
		enable_irq(dsim->res.irq);
		return 0;
	}

	dsim_dbg("%s %s +\n", __func__, dsim_state_names[dsim->state]);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	pm_runtime_get_sync(dsim->dev);

	/* DPHY reset control from DSIM */
	dpu_sysreg_select_dphy_rst_control(dsim->res.ss_regs, dsim->id, 1);

	/* DPHY power on : iso release */
	phy_power_on(dsim->phy);
	if (dsim->phy_ex)
		phy_power_on(dsim->phy_ex);

	/* DPHY status */
	dsim_phy_status(dsim->phy);

	panel_ctrl = (state == DSIM_STATE_ON) ? true : false;
	dsim_reg_init(dsim->id, &dsim->lcd_info, &dsim->clks, panel_ctrl);
#if !defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_start(dsim->id);
#endif
	dsim_reg_set_int(dsim->id, 1);

	dsim->state = state;
	enable_irq(dsim->res.irq);

	return 0;
}

static int dsim_enable(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_ON;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	ret = _dsim_enable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}

	if (prev_state != DSIM_STATE_INIT) {
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
		dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif
		call_panel_ops(dsim, displayon, dsim);
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
		dsim_reg_start(dsim->id);
		dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif
	}

	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int dsim_doze(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_DOZE;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	ret = _dsim_enable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}
	if (prev_state != DSIM_STATE_INIT) {
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
		dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif
		call_panel_ops(dsim, doze, dsim);
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
		dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif
	}
	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int _dsim_disable(struct dsim_device *dsim, enum dsim_state state)
{
	struct dsim_regs regs;

	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_warn("%s dsim already off(%s)\n",
				__func__, dsim_state_names[dsim->state]);
		if (state == DSIM_STATE_OFF)
			dsim_set_panel_power(dsim, 0);
		dsim->state = state;
		return 0;
	}

	dsim_dbg("%s %s +\n", __func__, dsim_state_names[dsim->state]);

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = state;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->res.irq);
	if (dsim_reg_stop(dsim->id, dsim->data_lane) < 0) {
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
	}

	/* HACK */
	phy_power_off(dsim->phy);
	if (dsim->phy_ex)
		phy_power_off(dsim->phy_ex);

	if (state == DSIM_STATE_OFF)
		dsim_set_panel_power(dsim, 0);

	pm_runtime_put_sync(dsim->dev);

	dsim_dbg("%s %s -\n", __func__, dsim_state_names[dsim->state]);
#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	return 0;
}

static int dsim_disable(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_OFF;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif
	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	call_panel_ops(dsim, suspend, dsim);
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif
	ret = _dsim_disable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}
	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int dsim_doze_suspend(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_DOZE_SUSPEND;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif
	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	call_panel_ops(dsim, doze_suspend, dsim);
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif
	ret = _dsim_disable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}
	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int dsim_enter_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);

	if (!IS_DSIM_ON_STATE(dsim)) {
		ret = -EBUSY;
		goto err;
	}

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	dsim->state = DSIM_STATE_ULPS;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->res.irq);
	ret = dsim_reg_stop_and_enter_ulps(dsim->id, dsim->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim);

	phy_power_off(dsim->phy);
	if (dsim->phy_ex)
		phy_power_off(dsim->phy_ex);

	pm_runtime_put_sync(dsim->dev);
#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	DPU_EVENT_LOG(DPU_EVT_ENTER_ULPS, &dsim->sd, start);
err:
	dsim_dbg("%s -\n", __func__);
	return ret;
}

static int dsim_exit_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);

	if (dsim->state != DSIM_STATE_ULPS) {
		ret = -EBUSY;
		goto err;
	}
#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	pm_runtime_get_sync(dsim->dev);

	/* DPHY reset control from DSIM */
	dpu_sysreg_select_dphy_rst_control(dsim->res.ss_regs, dsim->id, 1);
	/* DPHY power on : iso release */
	phy_power_on(dsim->phy);
	if (dsim->phy_ex)
		phy_power_on(dsim->phy_ex);

	/* DPHY status */
	dsim_phy_status(dsim->phy);

	dsim_reg_init(dsim->id, &dsim->lcd_info, &dsim->clks, false);
	ret = dsim_reg_exit_ulps_and_start(dsim->id, dsim->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim);

	enable_irq(dsim->res.irq);

	dsim->state = DSIM_STATE_ON;
	DPU_EVENT_LOG(DPU_EVT_EXIT_ULPS, &dsim->sd, start);
err:
	dsim_dbg("%s -\n", __func__);

	return 0;
}

static int dsim_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);

	if (enable)
		return dsim_enable(dsim);
	else
		return dsim_disable(dsim);
}

static int dsim_free_fb_resource(struct dsim_device *dsim)
{
	dsim_info("one to one unmapping: 0x%x\n", dsim->phys_addr);

	/* unmap */
	iovmm_unmap_oto(dsim->dev, dsim->phys_addr);

	/* unreserve memory */
	of_reserved_mem_device_release(dsim->dev);

	/* update state */
	dsim->fb_reservation = false;
	dsim->phys_addr = 0xdead;
	dsim->phys_size = 0;

	return 0;
}

static int dsim_acquire_fb_resource(struct dsim_device *dsim)
{
	int ret = 0;

	ret = of_reserved_mem_device_init_by_idx(dsim->dev, dsim->dev->of_node, 0);
	if (ret) {
		dsim_err("failed reserved mem device init: %d\n", ret);
		dsim->fb_reservation = false;
		goto err;
	} else
		dsim->fb_reservation = true;

	ret = iovmm_map_oto(dsim->dev, dsim->phys_addr, dsim->phys_size);
	if (ret)
		dsim_err("failed one to one mapping: %d\n", ret);
	else
		dsim_info("one to one mapping: 0x%x, 0x%x\n",
				dsim->phys_addr,
				dsim->phys_size);
err:
	return ret;
}

static long dsim_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);
	int ret = 0;

	switch (cmd) {
	case DSIM_IOC_GET_LCD_INFO:
		v4l2_set_subdev_hostdata(sd, &dsim->lcd_info);
		break;

	case DSIM_IOC_ENTER_ULPS:
		if ((unsigned long)arg)
			ret = dsim_enter_ulps(dsim);
		else
			ret = dsim_exit_ulps(dsim);
		break;

	case DSIM_IOC_DUMP:
		dsim_dump(dsim);
		break;

	case DSIM_IOC_GET_WCLK:
		v4l2_set_subdev_hostdata(sd, &dsim->clks.word_clk);
		break;

	case EXYNOS_DPU_GET_ACLK:
		return clk_get_rate(dsim->res.aclk);

	case DSIM_IOC_FREE_FB_RES:
		ret = dsim_free_fb_resource(dsim);
		break;

	case DSIM_IOC_DOZE:
		ret = dsim_doze(dsim);
		break;

	case DSIM_IOC_DOZE_SUSPEND:
		ret = dsim_doze_suspend(dsim);
		break;

	case DSIM_IOC_HS_CLK_ENABLE:
		ret = dsim_reg_set_hs_clock(dsim->id, 1);
		break;

	case DSIM_IOC_LPDT_CMD:
		if ((unsigned long)arg)
			dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
		else
			dsim_reg_set_cmd_transfer_mode(dsim->id, 0);

		break;

	default:
		dsim_err("unsupported ioctl");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int dsim_reg_read(struct v4l2_subdev *sd, struct fb_regrw_access_t *rr)
{
	int ret = 0;
	u32 dsi_package_type = 0;
	int i = 0;

	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);
	dsim_err("dsim_reg_read:read reg addr = 0x%x,rw_cnt = %d,use_hs_mode = %d!\n",rr->address,rr->buffer_size,rr->use_hs_mode);

	switch (rr->buffer_size) {
	case 0:
	case 1:
	case 2:
		dsi_package_type = MIPI_DSI_DCS_READ;
		break;
	default:
		dsi_package_type = MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM;
		break;
	}
	dsi_package_type = MIPI_DSI_DCS_READ;
	/* dsim sends the request for the lcd id and gets it buffer */
	ret = dsim_read_data(dsim, dsi_package_type,
		rr->address, rr->buffer_size, rr->buffer);

	if (ret < 0){
		dsim_err("%s:Failed to read panel reg!\n",__func__);
	}else{
		dsim_info("%s:Suceeded to read panel reg :0x%x = 0x%x\n",__func__,rr->address,rr->buffer);
		ret = 0;
	}

	for(i=0;i<rr->buffer_size;i++)
		dsim_err("dsim_reg_read@@@@@@@@@@@:rr.buf[%d]=0x%x!\n",i,rr->buffer[i]);

	return ret;
}

static const struct v4l2_subdev_core_ops dsim_sd_core_ops = {
	.ioctl = dsim_ioctl,
};

static const struct v4l2_subdev_video_ops dsim_sd_video_ops = {
	.s_stream = dsim_s_stream,
	.reg_read = dsim_reg_read,
};

static const struct v4l2_subdev_ops dsim_subdev_ops = {
	.core = &dsim_sd_core_ops,
	.video = &dsim_sd_video_ops,
};

static void dsim_init_subdev(struct dsim_device *dsim)
{
	struct v4l2_subdev *sd = &dsim->sd;

	v4l2_subdev_init(sd, &dsim_subdev_ops);
	sd->owner = THIS_MODULE;
	sd->grp_id = dsim->id;
	snprintf(sd->name, sizeof(sd->name), "%s.%d", "dsim-sd", dsim->id);
	v4l2_set_subdevdata(sd, dsim);
}

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION) && defined(READ_ESD_SOLUTION_TEST)
static ssize_t dsim_esd_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	dsim->esd_test = cmd;

	return count;
}
static DEVICE_ATTR(esd_test, 0644, NULL, dsim_esd_test_store);

int dsim_create_esd_test_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_esd_test);
	if (ret)
		dsim_err("failed to create command read & write sysfs\n");

	return ret;
}
#endif

static int dsim_cmd_sysfs_write(struct dsim_device *dsim, bool on)
{
	int ret = 0;

	if (on)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_ON, 0);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_OFF, 0);
	if (ret < 0)
		dsim_err("Failed to write test data!\n");
	else
		dsim_dbg("Succeeded to write test data!\n");

	return ret;
}

static int dsim_cmd_sysfs_read(struct dsim_device *dsim)
{
	int ret = 0;
	unsigned int id;
	u8 buf[4];

	/* dsim sends the request for the lcd id and gets it buffer */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
		MIPI_DCS_GET_DISPLAY_ID, DSIM_DDI_ID_LEN, buf);
	id = *(unsigned int *)buf;
	if (ret < 0)
		dsim_err("Failed to read panel id!\n");
	else
		dsim_info("Suceeded to read panel id : 0x%08x\n", id);

	return ret;
}

static ssize_t dsim_cmd_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t dsim_cmd_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	switch (cmd) {
	case 1:
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif
		ret = dsim_cmd_sysfs_read(dsim);
		call_panel_ops(dsim, dump, dsim);
#if defined(CONFIG_EXYNOS_PANEL_INIT_LPDT)
	dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif
		if (ret)
			return ret;
		break;
	case 2:
		ret = dsim_cmd_sysfs_write(dsim, true);
		dsim_info("Dsim write command, display on!!\n");
		if (ret)
			return ret;
		break;
	case 3:
		ret = dsim_cmd_sysfs_write(dsim, false);
		dsim_info("Dsim write command, display off!!\n");
		if (ret)
			return ret;
		break;
	default :
		dsim_info("unsupportable command\n");
		break;
	}

	return count;
}
static DEVICE_ATTR(cmd_rw, 0644, dsim_cmd_sysfs_show, dsim_cmd_sysfs_store);

static ssize_t dsim_ddi_addr_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int size = 0;
	int count;

	size = (ssize_t)sprintf(buf, "addr : 0x%02x   ", dsim->ddi_seq[0]);
	size = (ssize_t)sprintf(buf + size, "size : %d\n", dsim->ddi_seq_size);

	count = strlen(buf);
	return count;
}

static ssize_t dsim_ddi_addr_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	unsigned int res;
	int ret;

	char *cnt;
	char *addr;

	cnt = (char *)buf;
	addr = strsep(&cnt, " ");
	if (addr == NULL) {
		dsim_err("Usage : echo addr size > sysfs\n");
		goto end_func;
	}

	ret = kstrtoint(addr, 0, &res);
	if ((ret != 0) || (res > 255)) {
		dsim_err("Fail : addr(0x%x) value should be less than 0xFF\n", res);
		goto end_func;
	}
	dsim->ddi_seq[0] = (unsigned char)res;

	ret = kstrtoint(cnt, 0, &res);
	if (ret != 0)  {
		dsim_err("Fail : cnt wrong value\n");
		goto end_func;
	}
	dsim->ddi_seq_size = res;

	dsim_info("ddi_addr : 0x%x\n", dsim->ddi_seq[0]);
	dsim_info("ddi_seq_size : 0x%x\n", dsim->ddi_seq_size);

end_func:
	return count;
}
static DEVICE_ATTR(ddi_addr, 0600, dsim_ddi_addr_sysfs_show, dsim_ddi_addr_sysfs_store);

static ssize_t dsim_ddi_read_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int ret = 0;
	int offset = 0;
	int i;
	int count;

	/* dsim read */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, dsim->ddi_seq[0], dsim->ddi_seq_size, &dsim->ddi_seq[1]);

	if (ret < 0) {
		dsim_err("Failed to write test data!\n");
		count = 0;
		goto end_func;
	} else
		dsim_dbg("Succeeded to write test data!\n");


	/* print */
	for (i = 1; i <= dsim->ddi_seq_size; i++) {
		ret = sprintf(buf + offset, "0x%02x ", dsim->ddi_seq[i]);
		offset = offset + ret;
	}
	ret = sprintf(buf + offset, "\n");
	count = strlen(buf);

end_func:
	return count;
}
static DEVICE_ATTR(ddi_read, 0400, dsim_ddi_read_sysfs_show, NULL);

static ssize_t dsim_ddi_write_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	char *start;
	char *find;
	char token[] = "\0\0\0\0\0\0\0\0\0\0";
	unsigned int num;
	unsigned int exit = 1;

	unsigned int val;
	int ret;
	int i = 0;

	start = (char *)buf;

	while (exit) {
		/* parsing */
		find = strchr(start, ' ');
		if (find == NULL) {
			find = strchr(start, '\0');
			exit = 0;
		}

		num = find - start;
		strncpy(token, start, num);
		token[num] = '\0';

		find++;
		start = find;

		/* convert str to number */
		if ((strncmp("0x", token, 2) == 0) || (strncmp("0X", token, 2) == 0))
			ret = kstrtouint(token+2, 16, &val);
		else
			ret = kstrtouint(token, 16, &val);

		if (ret != 0) {
			dsim_err("Fail : data(%d) wrong value (should 0 ~ 0xff)\n", (unsigned int)val);
			goto end_func;
		}

		if (val > 255) {
			dsim_err("Fail : data(%d) value should be less than 0xFF\n", (unsigned int)val);
			goto end_func;
		}

		dsim->ddi_seq[i] = (unsigned char)val;
		dsim_info("%d\n", (unsigned int)dsim->ddi_seq[i]); // for debug
		i++;
	}
	dsim->ddi_seq_size = i - 1; // for except addr

	/* dsim write */
	if (dsim->ddi_seq_size == 1)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE, dsim->ddi_seq[0], 0);
	else if (dsim->ddi_seq_size == 2)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM, dsim->ddi_seq[0], dsim->ddi_seq[1]);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long)dsim->ddi_seq, dsim->ddi_seq_size + 1);

	if (ret < 0)
		dsim_err("Failed to write test data!\n");
	else
		dsim_dbg("Succeeded to write test data!\n");

end_func:
	return count;
}
static DEVICE_ATTR(ddi_write, 0200, NULL, dsim_ddi_write_sysfs_store);

int dsim_create_cmd_rw_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_cmd_rw);
	if (ret) {
		dsim_err("failed to create command read & write sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_ddi_addr);
	if (ret) {
		dsim_err("failed to create ddi_addr sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_ddi_read);
	if (ret) {
		dsim_err("failed to create ddi_read sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_ddi_write);
	if (ret) {
		dsim_err("failed to create ddi_write sysfs\n");
		goto error;
	}

error:
	return ret;
}

static void dsim_parse_lcd_info(struct dsim_device *dsim)
{
	struct device_node *node = NULL;
	struct device *dev = dsim->dev;
	char *ddi_device_type;
	u32 res[14];
	unsigned int mres_num = 1;
	u32 mres_w[3] = {0, };
	u32 mres_h[3] = {0, };
	u32 mres_dsc_w[3] = {0, };
	u32 mres_dsc_h[3] = {0, };
	u32 mres_dsc_en[3] = {0, };
	u32 hdr_num = 0;
	u32 hdr_type[HDR_CAPA_NUM] = {0, };
	u32 hdr_mxl = 0;
	u32 hdr_mal = 0;
	u32 hdr_mnl = 0;
	int k;

	of_property_read_u32(dev->of_node, "ddi_id", &dsim->ddi_id);
	dsim_info("ddi id : 0x%08x\n", dsim->ddi_id);

	switch (dsim->ddi_id) {
	case 0x404024ff:
		ddi_device_type = "samsung-s6e3fa0-vdo";
		dsim_info("ddi type : %s\n", ddi_device_type);
		break;

	case 0x02a90160:
		ddi_device_type = "novatek-nt36672a";
		dsim_info("ddi type : %s\n", ddi_device_type);
		break;

	case 0x01047291:
		ddi_device_type = "hixmax-hix83112a";
		dsim_info("ddi type : %s\n", ddi_device_type);
		break;

	case 0x01011292:
		ddi_device_type = "novatek-nov36672a";
		dsim_info("ddi type : %s\n", ddi_device_type);
		break;

	default:
		dsim_info("read ddi_device_type default lcd\n");
		ddi_device_type = "default-lcd-vdo";
		dsim_info("ddi type : %s\n", ddi_device_type);
		break;
	}

	snprintf(dsim->ddi_device_type, DSIM_DDI_TYPE_LEN, "%s", ddi_device_type);

	node = of_find_node_by_type(node, ddi_device_type);

	of_property_read_u32(node, "mode", &dsim->lcd_info.mode);
	dsim_info("%s mode\n", dsim->lcd_info.mode ? "command" : "video");

	of_property_read_u32_array(node, "resolution", res, 2);
	dsim->lcd_info.xres = res[0];
	dsim->lcd_info.yres = res[1];
	dsim_info("LCD(%s) resolution: xres(%d), yres(%d)\n",
			of_node_full_name(node), res[0], res[1]);

	of_property_read_u32_array(node, "size", res, 2);
	dsim->lcd_info.width = res[0];
	dsim->lcd_info.height = res[1];
	dsim_dbg("LCD size: width(%d), height(%d)\n", res[0], res[1]);

	of_property_read_u32(node, "timing,refresh", &dsim->lcd_info.fps);
	dsim_dbg("LCD refresh rate(%d)\n", dsim->lcd_info.fps);

	of_property_read_u32_array(node, "timing,h-porch", res, 3);
	dsim->lcd_info.hbp = res[0];
	dsim->lcd_info.hfp = res[1];
	dsim->lcd_info.hsa = res[2];
	dsim_dbg("hbp(%d), hfp(%d), hsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32_array(node, "timing,v-porch", res, 3);
	dsim->lcd_info.vbp = res[0];
	dsim->lcd_info.vfp = res[1];
	dsim->lcd_info.vsa = res[2];
	dsim_dbg("vbp(%d), vfp(%d), vsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32(node, "timing,dsi-hs-clk", &dsim->lcd_info.hs_clk);
	dsim->clks.hs_clk = dsim->lcd_info.hs_clk;
	dsim_dbg("requested hs clock(%d)\n", dsim->lcd_info.hs_clk);

#if defined(CONFIG_EXYNOS_DSIM_DITHER)
	of_property_read_u32_array(node, "timing,pmsk", res, 14);
#else
	of_property_read_u32_array(node, "timing,pmsk", res, 4);
#endif
	dsim->lcd_info.dphy_pms.p = res[0];
	dsim->lcd_info.dphy_pms.m = res[1];
	dsim->lcd_info.dphy_pms.s = res[2];
	dsim->lcd_info.dphy_pms.k = res[3];
	dsim_dbg("p(%d), m(%d), s(%d), k(%d)\n", res[0], res[1], res[2], res[3]);
#if defined(CONFIG_EXYNOS_DSIM_DITHER)
	dsim->lcd_info.dphy_pms.mfr = res[4];
	dsim->lcd_info.dphy_pms.mrr = res[5];
	dsim->lcd_info.dphy_pms.sel_pf = res[6];
	dsim->lcd_info.dphy_pms.icp = res[7];
	dsim->lcd_info.dphy_pms.afc_enb = res[8];
	dsim->lcd_info.dphy_pms.extafc = res[9];
	dsim->lcd_info.dphy_pms.feed_en = res[10];
	dsim->lcd_info.dphy_pms.fsel = res[11];
	dsim->lcd_info.dphy_pms.fout_mask = res[12];
	dsim->lcd_info.dphy_pms.rsel = res[13];
	dsim_dbg(" mfr(%d), mrr(0x%x), sel_pf(%d), icp(%d)\n",
				res[4], res[5], res[6], res[7]);
	dsim_dbg(" afc_enb(%d), extafc(%d), feed_en(%d), fsel(%d)\n",
				res[8], res[9], res[10], res[11]);
	dsim_dbg(" fout_mask(%d), rsel(%d)\n", res[12], res[13]);
#endif

	of_property_read_u32(node, "timing,dsi-escape-clk",
			&dsim->lcd_info.esc_clk);
	dsim->clks.esc_clk = dsim->lcd_info.esc_clk;
	dsim_dbg("requested escape clock(%d)\n", dsim->lcd_info.esc_clk);

	of_property_read_u32(node, "mic_en", &dsim->lcd_info.mic_enabled);
	dsim_info("mic enabled (%d)\n", dsim->lcd_info.mic_enabled);

	of_property_read_u32(node, "type_of_ddi", &dsim->lcd_info.ddi_type);
	dsim_dbg("ddi type(%d)\n", dsim->lcd_info.ddi_type);

	of_property_read_u32(node, "dsc_en", &dsim->lcd_info.dsc_enabled);
	dsim_info("dsc is %s\n", dsim->lcd_info.dsc_enabled ? "enabled" : "disabled");

	if (dsim->lcd_info.dsc_enabled) {
		of_property_read_u32(node, "dsc_cnt", &dsim->lcd_info.dsc_cnt);
		dsim_info("dsc count(%d)\n", dsim->lcd_info.dsc_cnt);
		of_property_read_u32(node, "dsc_slice_num",
				&dsim->lcd_info.dsc_slice_num);
		dsim_info("dsc slice count(%d)\n", dsim->lcd_info.dsc_slice_num);
		of_property_read_u32(node, "dsc_slice_h",
				&dsim->lcd_info.dsc_slice_h);
		dsim_info("dsc slice height(%d)\n", dsim->lcd_info.dsc_slice_h);
	}

	of_property_read_u32(node, "data_lane", &dsim->data_lane_cnt);
	dsim_info("using data lane count(%d)\n", dsim->data_lane_cnt);

	dsim->lcd_info.data_lane = dsim->data_lane_cnt;

	of_property_read_u32(node, "mres_en", &dsim->lcd_info.dt_lcd_mres.mres_en);
	dsim_info("mres_en(%d)\n", dsim->lcd_info.dt_lcd_mres.mres_en);
	dsim->lcd_info.mres_mode = 0; /* 0=WQHD, 1=FHD, 2=HD */
	dsim->lcd_info.dt_lcd_mres.mres_number = mres_num; /* default = 1 */

	if (dsim->lcd_info.dt_lcd_mres.mres_en) {
		of_property_read_u32(node, "mres_number", &mres_num);
		dsim->lcd_info.dt_lcd_mres.mres_number = mres_num;
		dsim_info("mres_number(%d)\n", mres_num);

		of_property_read_u32_array(node, "mres_width", mres_w, mres_num);
		of_property_read_u32_array(node, "mres_height", mres_h, mres_num);
		of_property_read_u32_array(node, "mres_dsc_width", mres_dsc_w, mres_num);
		of_property_read_u32_array(node, "mres_dsc_height", mres_dsc_h, mres_num);
		of_property_read_u32_array(node, "mres_dsc_en", mres_dsc_en, mres_num);

		switch (mres_num) {
		case 3:
			dsim->lcd_info.dt_lcd_mres.res_info[2].width = mres_w[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].height = mres_h[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].dsc_en = mres_dsc_en[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].dsc_width = mres_dsc_w[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].dsc_height = mres_dsc_h[2];
		case 2:
			dsim->lcd_info.dt_lcd_mres.res_info[1].width = mres_w[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].height = mres_h[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].dsc_en = mres_dsc_en[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].dsc_width = mres_dsc_w[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].dsc_height = mres_dsc_h[1];
		case 1:
			dsim->lcd_info.dt_lcd_mres.res_info[0].width = mres_w[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].height = mres_h[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].dsc_en = mres_dsc_en[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].dsc_width = mres_dsc_w[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].dsc_height = mres_dsc_h[0];
			break;
		default:
			dsim->lcd_info.dt_lcd_mres.res_info[0].width = dsim->lcd_info.width;
			dsim->lcd_info.dt_lcd_mres.res_info[0].height = dsim->lcd_info.height;
			dsim_warn("check multi-resolution configurations at DT\n");
			break;
		}
		dsim_info("[LCD multi(%d)-resolution info] 1st(%dx%d), 2nd(%dx%d), 3rd(%dx%d)\n",
				mres_num, mres_w[0], mres_h[0],
				mres_w[1], mres_h[1], mres_w[2], mres_h[2]);
	} else {
		dsim->lcd_info.dt_lcd_mres.res_info[0].width = dsim->lcd_info.width;
		dsim->lcd_info.dt_lcd_mres.res_info[0].height = dsim->lcd_info.height;
	}

	if (dsim->lcd_info.mode == DECON_MIPI_COMMAND_MODE) {
		of_property_read_u32_array(node, "cmd_underrun_lp_ref",
				dsim->lcd_info.cmd_underrun_lp_ref,
				dsim->lcd_info.dt_lcd_mres.mres_number);
		for (k = 0; k < dsim->lcd_info.dt_lcd_mres.mres_number; k++)
			dsim_info("mres[%d] cmd_underrun_lp_ref(%d)\n", k,
					dsim->lcd_info.cmd_underrun_lp_ref[k]);
	} else {
		of_property_read_u32(node, "vt_compensation",
				&dsim->lcd_info.vt_compensation);
		dsim_info("vt_compensation(%d)\n", dsim->lcd_info.vt_compensation);
	}

	/* HDR info */
	of_property_read_u32(node, "hdr_num", &hdr_num);
	dsim->lcd_info.dt_lcd_hdr.hdr_num = hdr_num;
	dsim_info("hdr_num(%d)\n", hdr_num);

	if (hdr_num != 0) {
		of_property_read_u32_array(node, "hdr_type", hdr_type, hdr_num);
		for (k = 0; k < hdr_num; k++) {
			dsim->lcd_info.dt_lcd_hdr.hdr_type[k] = hdr_type[k];
			dsim_info("hdr_type[%d] = %d\n", k, hdr_type[k]);
		}

		of_property_read_u32(node, "hdr_max_luma", &hdr_mxl);
		of_property_read_u32(node, "hdr_max_avg_luma", &hdr_mal);
		of_property_read_u32(node, "hdr_min_luma", &hdr_mnl);
		dsim->lcd_info.dt_lcd_hdr.hdr_max_luma = hdr_mxl;
		dsim->lcd_info.dt_lcd_hdr.hdr_max_avg_luma = hdr_mal;
		dsim->lcd_info.dt_lcd_hdr.hdr_min_luma = hdr_mnl;
		dsim_info("hdr_max_luma(%d), hdr_max_avg_luma(%d), hdr_min_luma(%d)\n",
				hdr_mxl, hdr_mal, hdr_mnl);
	}
}

static int dsim_parse_dt(struct dsim_device *dsim, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dsim_err("no device tree information\n");
		return -EINVAL;
	}

	dsim->id = of_alias_get_id(dev->of_node, "dsim");
	dsim_info("dsim(%d) probe start..\n", dsim->id);

	dsim->phy = devm_phy_get(dev, "dsim_dphy");
	if (IS_ERR_OR_NULL(dsim->phy)) {
		dsim_err("failed to get phy\n");
		return PTR_ERR(dsim->phy);
	}

	dsim->phy_ex = devm_phy_get(dev, "dsim_dphy_extra");
	if (IS_ERR_OR_NULL(dsim->phy_ex)) {
		dsim_err("failed to get extra phy. It's not mandatary.\n");
		dsim->phy_ex = NULL;
	}

	dsim->dev = dev;
	dsim_get_gpios(dsim);
	dsim_get_regulator(dsim);
	dsim_parse_lcd_info(dsim);
	dsim_get_ddi_id(dsim);

	return 0;
}

static void dsim_register_panel(struct dsim_device *dsim)
{
	switch (dsim->ddi_id) {
	case 0x404024ff:
		dsim->panel_ops = &s6e3fa0_mipi_lcd_driver;
		dsim_info("panel ops : s6e3fa0_mipi_lcd_driver\n");
		break;
	case 0x02a90160:
		dsim->panel_ops = &nt36672a_mipi_lcd_driver;
		dsim_info("panel ops : nt36672a_mipi_lcd_driver\n");
		break;
	case 0x01047291:
		dsim->panel_ops = &hix83112a_mipi_lcd_driver;
		dsim_info("panel ops : hix83112a_mipi_lcd_driver\n");
		break;
	case 0x01011292:
		dsim->panel_ops = &nov36672a_mipi_lcd_driver;
		dsim_info("panel ops : nov36672a_mipi_lcd_driver\n");
		break;
	default:
		dsim_info("panel ops is default lcd\n");
		dsim->panel_ops = &default_mipi_lcd_driver;
		dsim_info("panel ops : default_mipi_lcd_driver\n");
		break;
	}
}

static int dsim_get_data_lanes(struct dsim_device *dsim)
{
	int i;

	if (dsim->data_lane_cnt > MAX_DSIM_DATALANE_CNT) {
		dsim_err("%d data lane couldn't be supported\n",
				dsim->data_lane_cnt);
		return -EINVAL;
	}

	dsim->data_lane = DSIM_LANE_CLOCK;
	for (i = 1; i < dsim->data_lane_cnt + 1; ++i)
		dsim->data_lane |= 1 << i;

	dsim_info("%s: lanes(0x%x)\n", __func__, dsim->data_lane);

	return 0;
}

static int dsim_init_resources(struct dsim_device *dsim, struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dsim_err("failed to get mem resource\n");
		return -ENOENT;
	}
	dsim_info("res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);

	dsim->res.regs = devm_ioremap_resource(dsim->dev, res);
	if (!dsim->res.regs) {
		dsim_err("failed to remap DSIM SFR region\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dsim_info("no 2nd mem resource\n");
		dsim->res.phy_regs = NULL;
	} else {
		dsim_info("dphy res: start(0x%x), end(0x%x)\n",
				(u32)res->start, (u32)res->end);

		dsim->res.phy_regs = devm_ioremap_resource(dsim->dev, res);
		if (!dsim->res.phy_regs) {
			dsim_err("failed to remap DSIM DPHY SFR region\n");
			return -EINVAL;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dsim_info("no extra dphy resource\n");
		dsim->res.phy_regs_ex = NULL;
	} else {
		dsim_info("dphy_extra res: start(0x%x), end(0x%x)\n",
				(u32)res->start, (u32)res->end);

		dsim->res.phy_regs_ex = devm_ioremap_resource(dsim->dev, res);
		if (!dsim->res.phy_regs_ex) {
			dsim_err("failed to remap DSIM DPHY(EXTRA) SFR region\n");
			return -EINVAL;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dsim_err("failed to get irq resource\n");
		return -ENOENT;
	}

	dsim->res.irq = res->start;
	ret = devm_request_irq(dsim->dev, res->start,
			dsim_irq_handler, 0, pdev->name, dsim);
	if (ret) {
		dsim_err("failed to install DSIM irq\n");
		return -EINVAL;
	}
	disable_irq(dsim->res.irq);

	dsim->res.ss_regs = dpu_get_sysreg_addr();
	if (IS_ERR_OR_NULL(dsim->res.ss_regs)) {
		dsim_err("failed to get sysreg addr\n");
		return -EINVAL;
	}

	return 0;
}
int dsim_init_pinctrl(struct dsim_device *dsim)
{
	int ret = 0;

	dsim->res.pinctrl = devm_pinctrl_get(dsim->dev);
	if (IS_ERR(dsim->res.pinctrl)) {
		decon_err("failed to get dsim-%d pinctrl\n", dsim->id);
		ret = PTR_ERR(dsim->res.pinctrl);
		dsim->res.pinctrl = NULL;
		goto err;
	}

	dsim->res.lcd_reset_sleep = pinctrl_lookup_state(dsim->res.pinctrl, "lcd_reset");
	if (IS_ERR(dsim->res.lcd_reset_sleep)) {
		decon_err("failed to get hw_te_on pin state\n");
		ret = PTR_ERR(dsim->res.lcd_reset_sleep);
		dsim->res.lcd_reset_sleep = NULL;
		goto err;
	}

err:
	return ret;
}

static int dsim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dsim_device *dsim = NULL;

	dsim = devm_kzalloc(dev, sizeof(struct dsim_device), GFP_KERNEL);
	if (!dsim) {
		dsim_err("failed to allocate dsim device.\n");
		ret = -ENOMEM;
		goto err;
	}
#if !defined(CONFIG_SUPPORT_LEGACY_ION)
	dma_set_mask(dev, DMA_BIT_MASK(36));
#endif
	ret = dsim_parse_dt(dsim, dev);
	if (ret)
		goto err_dt;

	dsim_drvdata[dsim->id] = dsim;
	ret = dsim_get_clocks(dsim);
	if (ret)
		goto err_dt;

	spin_lock_init(&dsim->slock);
	mutex_init(&dsim->cmd_lock);
	init_completion(&dsim->ph_wr_comp);
	init_completion(&dsim->rd_comp);

	ret = dsim_init_resources(dsim, pdev);
	if (ret)
		goto err_dt;

	dsim_init_subdev(dsim);
	platform_set_drvdata(pdev, dsim);
	dsim_register_panel(dsim);
	setup_timer(&dsim->cmd_timer, dsim_cmd_fail_detector,
			(unsigned long)dsim);

	dsim_err("%s: dsim_init_pinctrl\n", __func__);
	ret = dsim_init_pinctrl(dsim);
	if (ret) {
		dsim_err("%s: failed to get pin resources\n", __func__);
	}

#if defined(CONFIG_CPU_IDLE)
	dsim->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	dsim_info("dsim idle_ip_index[%d]\n", dsim->idle_ip_index);
	if (dsim->idle_ip_index < 0)
		dsim_warn("idle ip index is not provided for dsim\n");
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	if (dsim->lcd_info.mode == DECON_VIDEO_MODE)
		dsim_acquire_fb_resource(dsim);

	pm_runtime_enable(dev);

	ret = dsim_get_data_lanes(dsim);
	if (ret)
		goto err_dt;

	phy_init(dsim->phy);
	if (dsim->phy_ex)
		phy_init(dsim->phy_ex);

	dsim->state = DSIM_STATE_INIT;
	dsim_enable(dsim);

	ret = iovmm_activate(dev);
	if (ret) {
		dsim_err("failed to activate iovmm\n");
		goto err_dt;
	}
	iovmm_set_fault_handler(dev, dpu_sysmmu_fault_handler, NULL);

	/* TODO: If you want to enable DSIM BIST mode. you must turn on LCD here */
#if !defined(BRINGUP_DSIM_BIST)
	call_panel_ops(dsim, probe, dsim);
#else
	/* TODO: This is for dsim BIST mode in zebu emulator. only for test*/
	call_panel_ops(dsim, displayon, dsim);
	dsim_reg_set_bist(dsim->id, true);
#endif

	/* for debug */
	/* dsim_dump(dsim); */

	dsim_clocks_info(dsim);
	dsim_create_cmd_rw_sysfs(dsim);

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
	dsim->esd_recovering = false;
#endif
#if defined(READ_ESD_SOLUTION_TEST)
	dsim_create_esd_test_sysfs(dsim);
#endif

#ifdef DPHY_LOOP
	dsim_reg_set_dphy_loop_back_test(dsim->id);
#endif

	dsim_info("dsim%d driver(%s mode) has been probed.\n", dsim->id,
		dsim->lcd_info.mode == DECON_MIPI_COMMAND_MODE ? "cmd" : "video");
	return 0;

err_dt:
	kfree(dsim);
err:
	return ret;
}

static int dsim_remove(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	mutex_destroy(&dsim->cmd_lock);
	dsim_info("dsim%d driver removed\n", dsim->id);

	return 0;
}

static void dsim_shutdown(struct platform_device *pdev)
{
#if 0
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SHUTDOWN, &dsim->sd, ktime_set(0, 0));
	dsim_info("%s + state:%d\n", __func__, dsim->state);

	dsim_disable(dsim);

	dsim_info("%s -\n", __func__);
#else
	dsim_info("%s +-\n", __func__);
#endif
}

static int dsim_runtime_suspend(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SUSPEND, &dsim->sd, ktime_set(0, 0));
	dsim_dbg("%s +\n", __func__);
	clk_disable_unprepare(dsim->res.aclk);
	dsim_dbg("%s -\n", __func__);
	return 0;
}

static int dsim_runtime_resume(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_RESUME, &dsim->sd, ktime_set(0, 0));
	dsim_dbg("%s: +\n", __func__);
	clk_prepare_enable(dsim->res.aclk);
	dsim_dbg("%s -\n", __func__);
	return 0;
}

static const struct of_device_id dsim_of_match[] = {
	{ .compatible = "samsung,exynos9-dsim" },
	{},
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static const struct dev_pm_ops dsim_pm_ops = {
	.runtime_suspend	= dsim_runtime_suspend,
	.runtime_resume		= dsim_runtime_resume,
};

static struct platform_driver dsim_driver __refdata = {
	.probe			= dsim_probe,
	.remove			= dsim_remove,
	.shutdown		= dsim_shutdown,
	.driver = {
		.name		= DSIM_MODULE_NAME,
		.owner		= THIS_MODULE,
		.pm		= &dsim_pm_ops,
		.of_match_table	= of_match_ptr(dsim_of_match),
		.suppress_bind_attrs = true,
	}
};

static int __init dsim_init(void)
{
	int ret = platform_driver_register(&dsim_driver);
	if (ret)
		pr_err("dsim driver register failed\n");

	return ret;
}
late_initcall(dsim_init);

static void __exit dsim_exit(void)
{
	platform_driver_unregister(&dsim_driver);
}

module_exit(dsim_exit);

static int rmem_device_init(struct reserved_mem *rmem, struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	dsim->phys_addr = rmem->base;
	dsim->phys_size = rmem->size;

	return 0;
}

/* of_reserved_mem_device_release(dev) when reserved memory is no logner required */
static void rmem_device_release(struct reserved_mem *rmem, struct device *dev)
{
	struct page *first = phys_to_page(PAGE_ALIGN(rmem->base));
	struct page *last = phys_to_page((rmem->base + rmem->size) & PAGE_MASK);
	struct page *page;

	pr_info("%s: base=%pa, size=%pa, first=%pa, last=%pa\n",
			__func__, &rmem->base, &rmem->size, first, last);

	for (page = first; page != last; page++) {
		__ClearPageReserved(page);
		set_page_count(page, 1);
		__free_pages(page, 0);
		adjust_managed_page_count(page, 1);
	}
}

static const struct reserved_mem_ops rmem_ops = {
	.device_init	= rmem_device_init,
	.device_release = rmem_device_release,
};

static int __init fb_rmem_setup(struct reserved_mem *rmem)
{
	pr_info("%s: base=%pa, size=%pa\n", __func__, &rmem->base, &rmem->size);

	rmem->ops = &rmem_ops;
	return 0;
}
RESERVEDMEM_OF_DECLARE(fb_rmem, "exynos,fb_rmem", fb_rmem_setup);

MODULE_AUTHOR("Yeongran Shin <yr613.shin@samsung.com>");
MODULE_DESCRIPTION("Samusung EXYNOS DSIM driver");
MODULE_LICENSE("GPL");
