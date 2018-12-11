/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *
 * Samsung EXYNOS SoC series DQE driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/fs.h>

#include "dqe.h"
#include "decon.h"
#if defined(CONFIG_SOC_EXYNOS9610)
#include "./cal_9610/regs-dqe.h"
#endif

struct dqe_device *dqe_drvdata;
struct class *dqe_class;

u32 gamma_lut[3][65];
u32 cgc_lut[50];
u32 hsc_lut[35];

#define TUNE_MODE_SIZE 3
u32 tune_mode[TUNE_MODE_SIZE][280];

int dqe_log_level = 6;
module_param(dqe_log_level, int, 0644);

const u32 bypass_cgc_tune[50] = {
	255,0,0,0,255,0,0,0,255,0,255,255,255,0,255,255,255,0,255,255,255,0,0,0,0,0,255,0,0,0,255,0,0,0,255,0,255,255,255,0,255,255,255,0,255,255,255,0,0,0
};

const u32 bypass_gamma_tune[3][65] = {
	{0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,68,72,76,80,84,88,92,96,100,104,108,112,116,120,124,128,132,136,140,144,148,152,156,160,164,168,172,176,180,184,188,192,196,200,204,208,212,216,220,224,228,232,236,240,244,248,252,256},
	{0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,68,72,76,80,84,88,92,96,100,104,108,112,116,120,124,128,132,136,140,144,148,152,156,160,164,168,172,176,180,184,188,192,196,200,204,208,212,216,220,224,228,232,236,240,244,248,252,256},
	{0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,68,72,76,80,84,88,92,96,100,104,108,112,116,120,124,128,132,136,140,144,148,152,156,160,164,168,172,176,180,184,188,192,196,200,204,208,212,216,220,224,228,232,236,240,244,248,252,256},
};

const s32 bypass_hsc_tune[35] = {
	0,0,0,0,0,0,0,1,15,1,0,1,0,0,0,0,0,0,0,5,-10,0,10,30,170,230,240,155,70,32,1,10,64,51,204
};

static void dqe_load_context(void)
{
	int i, j;
	struct dqe_device *dqe = dqe_drvdata;

	dqe_info("%s\n", __func__);

	for (i = 0; i < DQECGC1LUT_MAX + 1; i++) {
		dqe->ctx.cgc[i].addr = DQECGC1_RGB_BASE + (i * 4);
		dqe->ctx.cgc[i].val = dqe_read(dqe->ctx.cgc[i].addr);
	}

	for (i = DQECGC1LUT_MAX + 1, j = 0; i < (DQECGC1LUT_MAX + DQECGC2LUT_MAX); i++, j++) {
		dqe->ctx.cgc[i].addr = DQECGC2_RGB_BASE + (j * 4);
		dqe->ctx.cgc[i].val = dqe_read(dqe->ctx.cgc[i].addr);
	}

	for (i = 0; i < DQEGAMMALUT_MAX; i++) {
		dqe->ctx.gamma[i].addr = DQEGAMMALUT_X_Y_BASE + (i * 4);
		dqe->ctx.gamma[i].val = dqe_read(dqe->ctx.gamma[i].addr);
	}

	for (i = 0; i < DQEHSCLUT_MAX - 1; i++) {
		dqe->ctx.hsc[i].addr = DQEHSCLUT_BASE + (i * 4);
		dqe->ctx.hsc[i].val = dqe_read(dqe->ctx.hsc[i].addr);
	}
	dqe->ctx.hsc[DQEHSCLUT_MAX - 1].addr = DQEHSC_SKIN_H;
	dqe->ctx.hsc[DQEHSCLUT_MAX - 1].val = dqe_read(dqe->ctx.hsc[DQEHSCLUT_MAX - 1].addr);

	for (i = 0; i < DQECGC1LUT_MAX; i++) {
		dqe_dbg("0x%04x %d %d %d",
			dqe->ctx.cgc[i].addr,
			DQECGCLUT_R_GET(dqe->ctx.cgc[i].val),
			DQECGCLUT_G_GET(dqe->ctx.cgc[i].val),
			DQECGCLUT_B_GET(dqe->ctx.cgc[i].val));
	}

	dqe_dbg("0x%04x %08x ",
		dqe->ctx.cgc[DQECGC1LUT_MAX].addr, dqe->ctx.cgc[DQECGC1LUT_MAX].val);

	for (i = DQECGC1LUT_MAX + 1; i < (DQECGC1LUT_MAX + DQECGC2LUT_MAX); i++) {
		dqe_dbg("0x%04x %d %d %d",
			dqe->ctx.cgc[i].addr,
			DQECGCLUT_R_GET(dqe->ctx.cgc[i].val),
			DQECGCLUT_G_GET(dqe->ctx.cgc[i].val),
			DQECGCLUT_B_GET(dqe->ctx.cgc[i].val));
	}

	for (i = 0; i < DQEGAMMALUT_MAX; i++) {
		dqe_dbg("0x%04x %d %d ",
			dqe->ctx.gamma[i].addr,
			DQEGAMMALUT_X_GET(dqe->ctx.gamma[i].val),
			DQEGAMMALUT_Y_GET(dqe->ctx.gamma[i].val));
	}

	for (i = 0; i < DQEHSCLUT_MAX; i++) {
		dqe_dbg("0x%04x %08x ",
			dqe->ctx.hsc[i].addr, dqe->ctx.hsc[i].val);
	}
}

static void dqe_init_context(void)
{
	int i, j, k, val;
	struct dqe_device *dqe = dqe_drvdata;

	dqe_info("%s\n", __func__);

	dqe->ctx.cgc[0].val = 0x0ff00000; /* DQECGC1_RED */
	dqe->ctx.cgc[1].val = 0x0003fc00; /* DQECGC1_GREEN */
	dqe->ctx.cgc[2].val = 0x000000ff; /* DQECGC1_BLUE */
	dqe->ctx.cgc[3].val = 0x0003fcff; /* DQECGC1_CYAN */
	dqe->ctx.cgc[4].val = 0x0ff000ff; /* DQECGC1_MAGENTA */
	dqe->ctx.cgc[5].val = 0x0ff3fc00; /* DQECGC1_YELLOW */
	dqe->ctx.cgc[6].val = 0x0ff3fcff; /* DQECGC1_WHITE */
	dqe->ctx.cgc[7].val = 0x00000000; /* DQECGC1_BLACK */
	dqe->ctx.cgc[8].val = 0x00000000; /* DQECGC_MC_CONTROL*/
	dqe->ctx.cgc[9].val = 0x0ff00000; /* DQECGC2_RED */
	dqe->ctx.cgc[10].val = 0x0003fc00; /* DQECGC2_GREEN */
	dqe->ctx.cgc[11].val = 0x000000ff; /* DQECGC2_BLUE */
	dqe->ctx.cgc[12].val = 0x0003fcff; /* DQECGC2_CYAN */
	dqe->ctx.cgc[13].val = 0x0ff000ff; /* DQECGC2_MAGENTA */
	dqe->ctx.cgc[14].val = 0x0ff3fc00; /* DQECGC2_YELLOW */
	dqe->ctx.cgc[15].val = 0x0ff3fcff; /* DQECGC2_WHITE */
	dqe->ctx.cgc[16].val = 0x00000000; /* DQECGC12_BLACK */

	/* DQEGAMMALUT_R_01_00 -- DQEGAMMALUT_B_64 */
	for (j = 0, k = 0; j < 3; j++) {
		val = 0;
		for (i = 0; i < 64; i += 2) {
			dqe->ctx.gamma[k++].val = (DQEGAMMALUT_X(val) | DQEGAMMALUT_Y(val + 4));
			val += 8;
		}
		dqe->ctx.gamma[k++].val = DQEGAMMALUT_X(val);
	}

	dqe->ctx.hsc[0].val = 0x00000000; /* DQEHSC_PPSCGAIN_RGB */
	dqe->ctx.hsc[1].val = 0x00000000; /* DQEHSC_PPSCGAIN_CMY */
	dqe->ctx.hsc[2].val = 0x00007605; /* DQEHSC_ALPHASCALE_SHIF */
	dqe->ctx.hsc[3].val = 0x0aa0780a; /* DQEHSC_POLY_CURVE0 */
	dqe->ctx.hsc[4].val = 0x09b3c0e6; /* DQEHSC_POLY_CURVE1 */
	dqe->ctx.hsc[5].val = 0x00ce0030; /* DQEHSC_SKIN_S */
	dqe->ctx.hsc[6].val = 0x00000000; /* DQEHSC_PPHCGAIN_RGB */
	dqe->ctx.hsc[7].val = 0x00000000; /* DQEHSC_PPHCGAIN_CMY */
	dqe->ctx.hsc[8].val = 0x00007605; /* DQEHSC_TSC_YCOMP */
	dqe->ctx.hsc[9].val = 0x00008046; /* DQEHSC_POLY_CURVE2 */
	dqe->ctx.hsc[10].val = 0x0040000a; /* DQEHSC_SKIN_H */

	dqe->ctx.cgc_on = 0;
	dqe->ctx.gamma_on = 0;
	dqe->ctx.hsc_on = 0;
	dqe->ctx.hsc_control = 0;

	dqe->ctx.need_udpate = true;

	k = 0;
	for (i = 0; i < 50; i++)
		tune_mode[0][k++] = bypass_cgc_tune[i];

	for (i = 0; i < 3; i++)
		for (j = 0; j < 65; j++)
			tune_mode[0][k++] = bypass_gamma_tune[i][j];

	for (i = 0; i < 35; i++)
		tune_mode[0][k++] = (u32)bypass_hsc_tune[i];

	for (i = 0; i < 280; i++)
		dqe_dbg("%04x, %d", tune_mode[0][i], tune_mode[0][i]);

	for (i = 1; i < TUNE_MODE_SIZE; i++)
		for (k = 0; k < 280; k++)
			tune_mode[i][k] = tune_mode[i-1][k];
}

static int dqe_save_context(void)
{
	int i;
	struct dqe_device *dqe = dqe_drvdata;

	if (dqe->ctx.need_udpate)
		return 0;

	dqe_dbg("%s\n", __func__);

	for (i = 0; i < (DQECGC1LUT_MAX + DQECGC2LUT_MAX); i++)
		dqe->ctx.cgc[i].val =
			dqe_read(dqe->ctx.cgc[i].addr);

	dqe->ctx.cgc_on = dqe_reg_get_cgc_on();

	for (i = 0; i < DQEGAMMALUT_MAX; i++)
		dqe->ctx.gamma[i].val =
			dqe_read(dqe->ctx.gamma[i].addr);

	dqe->ctx.gamma_on = dqe_reg_get_gamma_on();

	for (i = 0; i < DQEHSCLUT_MAX; i++)
		dqe->ctx.hsc[i].val =
			dqe_read(dqe->ctx.hsc[i].addr);

	dqe->ctx.hsc_on = dqe_reg_get_hsc_on();
	dqe->ctx.hsc_control = dqe_reg_get_hsc_control();

	return 0;
}

static int dqe_restore_context(void)
{
	int i;
	struct dqe_device *dqe = dqe_drvdata;

	dqe_dbg("%s\n", __func__);

	for (i = 0; i < (DQECGC1LUT_MAX + DQECGC2LUT_MAX); i++)
		dqe_write(dqe->ctx.cgc[i].addr,
				dqe->ctx.cgc[i].val);

	if (dqe->ctx.cgc_on)
		dqe_reg_set_cgc_on(1);

	for (i = 0; i < DQEGAMMALUT_MAX; i++)
		dqe_write(dqe->ctx.gamma[i].addr,
				dqe->ctx.gamma[i].val);

	if (dqe->ctx.gamma_on)
		dqe_reg_set_gamma_on(1);

	for (i = 0; i < DQEHSCLUT_MAX; i++)
		dqe_write(dqe->ctx.hsc[i].addr,
				dqe->ctx.hsc[i].val);

	if (dqe->ctx.hsc_on) {
		dqe_reg_set_hsc_control_all_reset();
		dqe_reg_set_hsc_on(1);
		dqe_reg_set_hsc_control(dqe->ctx.hsc_control);
	}

	dqe->ctx.need_udpate = false;

	return 0;
}

static void dqe_gamma_lut_set(void)
{
	int i, j, k;
	struct dqe_device *dqe = dqe_drvdata;

	for (j = 0, k = 0; j < 3; j++) {
		for (i = 0; i < 64; i += 2) {
			dqe->ctx.gamma[k++].val = (
				DQEGAMMALUT_X(gamma_lut[j][i]) |
				DQEGAMMALUT_Y(gamma_lut[j][i+1]));
		}
		dqe->ctx.gamma[k++].val =
			DQEGAMMALUT_X(gamma_lut[j][64]);
	}
}

static void dqe_cgc_lut_set(void)
{
	int i, j;
	struct dqe_device *dqe = dqe_drvdata;

	/* DQECGC1_RED -- DQECGC1_BLACK*/
	for (i = 0, j = 0; i < 8; i++) {
		dqe->ctx.cgc[i].val = (
			DQECGCLUT_R(cgc_lut[j]) |
			DQECGCLUT_G(cgc_lut[j + 1]) |
			DQECGCLUT_B(cgc_lut[j + 2]));
		j += 3;
	}

	/* DQECGC_MC_CONTROL */
	dqe->ctx.cgc[8].val = (DQECGC_MC_GAIN(cgc_lut[24]) | DQECGC_MC_EN(cgc_lut[25]));

	/* DQECGC2_RED -- DQECGC2_BLACK*/
	for (i = 9, j = 26; i < 17; i++) {
		dqe->ctx.cgc[i].val = (
			DQECGCLUT_R(cgc_lut[j]) |
			DQECGCLUT_G(cgc_lut[j + 1]) |
			DQECGCLUT_B(cgc_lut[j + 2]));
		j += 3;
	}
}

static void dqe_hsc_lut_set(void)
{
	struct dqe_device *dqe = dqe_drvdata;

	/* DQEHSC_PPSCGAIN_RGB */
	dqe->ctx.hsc[0].val = (
		DQEHSCLUT_R(hsc_lut[1]) | DQEHSCLUT_G(hsc_lut[2]) | DQEHSCLUT_B(hsc_lut[3]));
	/* DQEHSC_PPSCGAIN_CMY */
	dqe->ctx.hsc[1].val = (
		DQEHSCLUT_C(hsc_lut[4]) | DQEHSCLUT_M(hsc_lut[5]) | DQEHSCLUT_Y(hsc_lut[6]));
	/* DQEHSC_ALPHASCALE_SHIFT */
	dqe->ctx.hsc[2].val = (
		DQEHSCLUT_ALPHA_SHIFT2(hsc_lut[21]) | DQEHSCLUT_ALPHA_SHIFT1(hsc_lut[20]) |
		DQEHSCLUT_ALPHA_SCALE(hsc_lut[19]));
	/* DQEHSC_POLY_CURVE0 */
	dqe->ctx.hsc[3].val = (
		DQEHSCLUT_POLY_CURVE3(hsc_lut[24]) | DQEHSCLUT_POLY_CURVE2(hsc_lut[23]) |
		DQEHSCLUT_POLY_CURVE1(hsc_lut[22]));
	/* DQEHSC_POLY_CURVE1 */
	dqe->ctx.hsc[4].val = (
		DQEHSCLUT_POLY_CURVE6(hsc_lut[27]) | DQEHSCLUT_POLY_CURVE5(hsc_lut[26]) |
		DQEHSCLUT_POLY_CURVE4(hsc_lut[25]));
	/* DQEHSC_SKIN_S */
	dqe->ctx.hsc[5].val = (
		DQEHSCLUT_SKIN_S2(hsc_lut[34]) | DQEHSCLUT_SKIN_S1(hsc_lut[33]));
	/* DQEHSC_PPHCGAIN_RGB */
	dqe->ctx.hsc[6].val = (
		DQEHSCLUT_R(hsc_lut[13]) | DQEHSCLUT_G(hsc_lut[14]) | DQEHSCLUT_B(hsc_lut[15]));
	/* DQEHSC_PPHCGAIN_CMY */
	dqe->ctx.hsc[7].val = (
		DQEHSCLUT_C(hsc_lut[16]) | DQEHSCLUT_M(hsc_lut[17]) | DQEHSCLUT_Y(hsc_lut[18]));
	/* DQEHSC_TSC_YCOMP */
	dqe->ctx.hsc[8].val = (
		DQEHSCLUT_YCOMP_RATIO(hsc_lut[8]) | DQEHSCLUT_TSC_GAIN(hsc_lut[10]));
	/* DQEHSC_POLY_CURVE2 */
	dqe->ctx.hsc[9].val = (
		DQEHSCLUT_POLY_CURVE8(hsc_lut[29]) | DQEHSCLUT_POLY_CURVE7(hsc_lut[28]));
	/* DQEHSC_SKIN_H */
	dqe->ctx.hsc[10].val = (
		DQEHSCLUT_SKIN_S2(hsc_lut[32]) | DQEHSCLUT_SKIN_S1(hsc_lut[31]));
}

static ssize_t decon_dqe_gamma_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t count = 0;
	struct dqe_device *dqe = dev_get_drvdata(dev);

	dqe_info("%s\n", __func__);

	mutex_lock(&dqe->lock);

	for (i = 0; i < DQEGAMMALUT_MAX; i++) {
		dqe_info("%d %d ",
			DQEGAMMALUT_X_GET(dqe->ctx.gamma[i].val),
			DQEGAMMALUT_Y_GET(dqe->ctx.gamma[i].val));
		dqe_dbg("0x%04x %08x ",
			dqe->ctx.gamma[i].addr, dqe->ctx.gamma[i].val);
	}

	mutex_unlock(&dqe->lock);

	count = snprintf(buf, PAGE_SIZE, "gamma_on = %d\n", dqe->ctx.gamma_on);

	return count;
}

static ssize_t decon_dqe_gamma_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int i, j, k;
	int ret = 0;
	char *head = NULL;
	char *ptr = NULL;
	struct dqe_device *dqe = dev_get_drvdata(dev);
	struct decon_device *decon = get_decon_drvdata(0);

	dqe_info("%s +\n", __func__);

	mutex_lock(&dqe->lock);

	if (count <= 0) {
		dqe_err("gamma write count error\n");
		ret = -1;
		goto err;
	}

	if (decon) {
		if ((decon->state == DECON_STATE_OFF) ||
			(decon->state == DECON_STATE_INIT)) {
			dqe_err("decon is not enabled!(%d)\n", decon->state);
			ret = -1;
			goto err;
		}
	} else {
		dqe_err("decon is NULL!\n");
		ret = -1;
		goto err;
	}

	head = (char *)buffer;
	if  (*head != 0) {
		dqe_dbg("%s\n", head);
		for (i = 0; i < 3; i++) {
			k = (i == 2) ? 64 : 65;
			for (j = 0; j < k; j++) {
				ptr = strchr(head, ',');
				if (ptr == NULL) {
					dqe_err("not found comma.(%d, %d)\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				*ptr = 0;
				ret = kstrtou32(head, 0, &gamma_lut[i][j]);
				if (ret) {
					dqe_err("strtou32(%d, %d) error.\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				head = ptr + 1;
			}
		}
		k = 0;
		while (*(head + k) >= '0' && *(head + k) <= '9')
			k++;
		*(head + k) = 0;
		ret = kstrtou32(head, 0, &gamma_lut[i-1][j]);
		if (ret) {
			dqe_err("strtou32(%d, %d) error.\n", i-1, j);
			ret = -EINVAL;
			goto err;
		}
	} else {
		dqe_err("buffer is null.\n");
		goto err;
	}


	for (i = 0; i < 3; i++)
		for (j = 0; j < 65; j++)
			dqe_dbg("%d ", gamma_lut[i][j]);

	dqe_gamma_lut_set();

	dqe->ctx.gamma_on = DQE_GAMMA_ON_MASK;
	dqe->ctx.need_udpate = true;

	dqe_restore_context();
	decon_reg_update_req_dqe(decon->id);

	mutex_unlock(&dqe->lock);

	dqe_info("%s -\n", __func__);

	return count;
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;
}

static DEVICE_ATTR(gamma, 0660,
	decon_dqe_gamma_show,
	decon_dqe_gamma_store);

static ssize_t decon_dqe_cgc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t count = 0;
	struct dqe_device *dqe = dev_get_drvdata(dev);

	dqe_info("%s\n", __func__);

	mutex_lock(&dqe->lock);

	for (i = 0; i < DQECGC1LUT_MAX; i++) {
		dqe_info("%d %d %d ",
			DQECGCLUT_R_GET(dqe->ctx.cgc[i].val),
			DQECGCLUT_G_GET(dqe->ctx.cgc[i].val),
			DQECGCLUT_B_GET(dqe->ctx.cgc[i].val));
		dqe_dbg("0x%04x %08x",
			dqe->ctx.cgc[i].addr, dqe->ctx.cgc[i].val);
	}

	mutex_unlock(&dqe->lock);

	count = snprintf(buf, PAGE_SIZE, "cgc_on = %d\n", dqe->ctx.cgc_on);

	return count;
}

static ssize_t decon_dqe_cgc_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int i, j, k;
	int ret = 0;
	char *head = NULL;
	char *ptr = NULL;
	struct dqe_device *dqe = dev_get_drvdata(dev);
	struct decon_device *decon = get_decon_drvdata(0);

	dqe_info("%s +\n", __func__);

	mutex_lock(&dqe->lock);

	if (count <= 0) {
		dqe_err("cgc write count error\n");
		ret = -1;
		goto err;
	}

	if (decon) {
		if ((decon->state == DECON_STATE_OFF) ||
			(decon->state == DECON_STATE_INIT)) {
			dqe_err("decon is not enabled!(%d)\n", decon->state);
			ret = -1;
			goto err;
		}
	} else {
		dqe_err("decon is NULL!\n");
		ret = -1;
		goto err;
	}

	head = (char *)buffer;
	if  (*head != 0) {
		dqe_dbg("%s\n", head);
		for (i = 0; i < 1; i++) {
			k = 49;
			for (j = 0; j < k; j++) {
				ptr = strchr(head, ',');
				if (ptr == NULL) {
					dqe_err("not found comma.(%d, %d)\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				*ptr = 0;
				ret = kstrtou32(head, 0, &cgc_lut[j]);
				if (ret) {
					dqe_err("strtou32(%d, %d) error.\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				head = ptr + 1;
			}
		}
		k = 0;
		while (*(head + k) >= '0' && *(head + k) <= '9')
			k++;
		*(head + k) = 0;
		ret = kstrtou32(head, 0, &cgc_lut[j]);
		if (ret) {
			dqe_err("strtou32(%d, %d) error.\n", i-1, j);
			ret = -EINVAL;
			goto err;
		}
	} else {
		dqe_err("buffer is null.\n");
		goto err;
	}

	for (j = 0; j < 50; j++)
		dqe_dbg("%d ", cgc_lut[j]);

	dqe_cgc_lut_set();

	dqe->ctx.cgc_on = DQE_CGC_ON_MASK;
	dqe->ctx.need_udpate = true;

	dqe_restore_context();
	decon_reg_update_req_dqe(decon->id);

	mutex_unlock(&dqe->lock);

	dqe_info("%s -\n", __func__);

	return count;
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;
}

static DEVICE_ATTR(cgc, 0660,
	decon_dqe_cgc_show,
	decon_dqe_cgc_store);

static ssize_t decon_dqe_hsc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct dqe_device *dqe = dev_get_drvdata(dev);

	dqe_info("%s\n", __func__);

	mutex_lock(&dqe->lock);

	dqe_info("%d %d %d %d %d %d",
		DQEHSCLUT_R_GET(dqe->ctx.hsc[0].val),
		DQEHSCLUT_G_GET(dqe->ctx.hsc[0].val),
		DQEHSCLUT_B_GET(dqe->ctx.hsc[0].val),
		DQEHSCLUT_C_GET(dqe->ctx.hsc[1].val),
		DQEHSCLUT_M_GET(dqe->ctx.hsc[1].val),
		DQEHSCLUT_Y_GET(dqe->ctx.hsc[1].val));

	dqe_info("%d %d %d %d %d %d",
		DQEHSCLUT_R_GET(dqe->ctx.hsc[6].val),
		DQEHSCLUT_G_GET(dqe->ctx.hsc[6].val),
		DQEHSCLUT_B_GET(dqe->ctx.hsc[6].val),
		DQEHSCLUT_C_GET(dqe->ctx.hsc[7].val),
		DQEHSCLUT_M_GET(dqe->ctx.hsc[7].val),
		DQEHSCLUT_Y_GET(dqe->ctx.hsc[7].val));

	mutex_unlock(&dqe->lock);

	count = snprintf(buf, PAGE_SIZE, "hsc_on = %d\n", dqe->ctx.hsc_on);

	return count;
}

static ssize_t decon_dqe_hsc_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int i, j, k;
	int ret = 0;
	char *head = NULL;
	char *ptr = NULL;
	struct dqe_device *dqe = dev_get_drvdata(dev);
	struct decon_device *decon = get_decon_drvdata(0);
	s32 hsc_lut_signed[35] = {0,};

	dqe_info("%s +\n", __func__);

	mutex_lock(&dqe->lock);

	if (count <= 0) {
		dqe_err("hsc write count error\n");
		ret = -1;
		goto err;
	}

	if (decon) {
		if ((decon->state == DECON_STATE_OFF) ||
			(decon->state == DECON_STATE_INIT)) {
			dqe_err("decon is not enabled!(%d)\n", decon->state);
			ret = -1;
			goto err;
		}
	} else {
		dqe_err("decon is NULL!\n");
		ret = -1;
		goto err;
	}

	head = (char *)buffer;
	if  (*head != 0) {
		dqe_dbg("%s\n", head);
		for (i = 0; i < 1; i++) {
			k = 34;
			for (j = 0; j < k; j++) {
				ptr = strchr(head, ',');
				if (ptr == NULL) {
					dqe_err("not found comma.(%d, %d)\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				*ptr = 0;
				ret = kstrtos32(head, 0, &hsc_lut_signed[j]);
				if (ret) {
					dqe_err("strtos32(%d, %d) error.\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				head = ptr + 1;
			}
		}
		k = 0;
		while (*(head + k) >= '0' && *(head + k) <= '9')
			k++;
		*(head + k) = 0;
		ret = kstrtos32(head, 0, &hsc_lut_signed[j]);
		if (ret) {
			dqe_err("strtos32(%d, %d) error.\n", i-1, j);
			ret = -EINVAL;
			goto err;
		}
	} else {
		dqe_err("buffer is null.\n");
		goto err;
	}

	for (j = 0; j < 35; j++)
		dqe_dbg("%d ", hsc_lut_signed[j]);

	/* signed -> unsigned */
	for (j = 0; j < 35; j++) {
		hsc_lut[j] = (u32)hsc_lut_signed[j];
		dqe_dbg("%04x %d", hsc_lut[j], hsc_lut_signed[j]);
	}

	dqe_hsc_lut_set();

	dqe->ctx.hsc_on = DQE_HSC_ON_MASK;
	dqe->ctx.hsc_control = (
		HSC_PPSC_ON(hsc_lut[0]) | HSC_YCOMP_ON(hsc_lut[7]) | HSC_TSC_ON(hsc_lut[9]) |
		HSC_DITHER_ON(hsc_lut[11]) | HSC_PPHC_ON(hsc_lut[12]) | HSC_SKIN_ON(hsc_lut[30]));
	dqe->ctx.need_udpate = true;

	dqe_info("%s: hsc_control=0x%04x\n", __func__, dqe->ctx.hsc_control);

	dqe_restore_context();
	decon_reg_update_req_dqe(decon->id);

	mutex_unlock(&dqe->lock);

	dqe_info("%s -\n", __func__);

	return count;
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;
}

static DEVICE_ATTR(hsc, 0660,
	decon_dqe_hsc_show,
	decon_dqe_hsc_store);

static ssize_t decon_dqe_xml_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dqe_device *dqe = dev_get_drvdata(dev);
	char *p = buf;
	int len;

	dqe_info("%s\n", __func__);

	mutex_lock(&dqe->lock);

	len = sprintf(p, "vendor/etc/dqe/calib_data.xml\n");

	mutex_unlock(&dqe->lock);

	return len;
}

static DEVICE_ATTR(xml, 0440, decon_dqe_xml_show, NULL);

static int dqe_tune_mode_set(char *head, int mode)
{
	int ret = 0;
	int i = 0, j, k;
	char *ptr = NULL;
	s32 tune_mode_signed[280] = {0,};

	dqe_info("%s: %d \n", __func__, mode);

	if  (*head != 0) {
		dqe_dbg("%s\n", head);
			k = 280;
			for (j = 0; j < k; j++) {
				ptr = strchr(head, ',');
				if (ptr == NULL) {
					dqe_err("not found comma.(%d, %d)\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				*ptr = 0;
				ret = kstrtos32(head, 0, &tune_mode_signed[j]);
				if (ret) {
					dqe_err("strtos32(%d, %d) error.\n", i, j);
					ret = -EINVAL;
					goto err;
				}
				head = ptr + 1;
			}

	} else {
		dqe_err("buffer is null.\n");
		goto err;
	}

	/* signed -> unsigned */
	for (i = 0; i < 280; i++) {
		tune_mode[mode][i] = (u32)tune_mode_signed[i];
		dqe_dbg("%04x %d", tune_mode[mode][i], tune_mode_signed[i]);
	}
err:
	if (ret)
		dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;

}

static ssize_t decon_dqe_tune_mode1_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int ret = 0;
	char *head = NULL;
	struct dqe_device *dqe = dev_get_drvdata(dev);

	dqe_info("%s +\n", __func__);

	mutex_lock(&dqe->lock);

	if (count <= 0) {
		dqe_err("tune mode1 write count error\n");
		ret = -1;
		goto err;
	}

	head = (char *)buffer;

	ret = dqe_tune_mode_set(head, 1);
	if (ret)
		goto err;

	mutex_unlock(&dqe->lock);

	dqe_info("%s -\n", __func__);

	return count;
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;
}

static DEVICE_ATTR(tune_mode1, 0660,
	NULL,
	decon_dqe_tune_mode1_store);

static ssize_t decon_dqe_tune_mode2_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int ret = 0;
	char *head = NULL;
	struct dqe_device *dqe = dev_get_drvdata(dev);

	dqe_info("%s +\n", __func__);

	mutex_lock(&dqe->lock);

	if (count <= 0) {
		dqe_err("tune mode2 write count error\n");
		ret = -1;
		goto err;
	}

	head = (char *)buffer;

	ret = dqe_tune_mode_set(head, 2);
	if (ret)
		goto err;

	mutex_unlock(&dqe->lock);

	dqe_info("%s -\n", __func__);

	return count;
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;
}

static DEVICE_ATTR(tune_mode2, 0660,
	NULL,
	decon_dqe_tune_mode2_store);

static ssize_t decon_dqe_aosp_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	ssize_t count = 0;

	dqe_info("%s\n", __func__);

	count = snprintf(buf, PAGE_SIZE, "aosp_colors = %d\n", val);

	return count;
}

static ssize_t decon_dqe_aosp_colors_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int i, ret = 0;
	unsigned int value = 0;
	struct dqe_device *dqe = dev_get_drvdata(dev);
	struct decon_device *decon = get_decon_drvdata(0);

	dqe_info("%s +\n", __func__);

	mutex_lock(&dqe->lock);

	if (count <= 0) {
		dqe_err("aosp colors write count error\n");
		ret = -1;
		goto err;
	}

	if (decon) {
		if ((decon->state == DECON_STATE_OFF) ||
			(decon->state == DECON_STATE_INIT)) {
			dqe_err("decon is not enabled!(%d)\n", decon->state);
			ret = -1;
			goto err;
		}
	} else {
		dqe_err("decon is NULL!\n");
		ret = -1;
		goto err;
	}

	ret = kstrtouint(buffer, 0, &value);
	if (ret < 0)
		goto err;

	dqe_info("%s: %d\n", __func__, value);

	switch (value) {
	case 0: /* Natural */
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[1][i];
		break;

	case 1:/* Boosted */
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[1][i];
		break;

	case 2:/* Saturated */
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[0][i];
		break;

	case 90:/* mode0 */
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[0][i];
		break;

	case 91:/* mode1 */
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[1][i];
		break;

	case 92:/* mode1 */
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[2][i];
		break;

	default:
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[0][i];
		break;
	}

	dqe_cgc_lut_set();

	dqe->ctx.cgc_on = DQE_CGC_ON_MASK;
	dqe->ctx.need_udpate = true;

	dqe_restore_context();
	decon_reg_update_req_dqe(decon->id);

	mutex_unlock(&dqe->lock);

	return count;
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : err(%d)\n", __func__, ret);

	return ret;
}

static DEVICE_ATTR(aosp_colors, 0660,
	decon_dqe_aosp_color_show,
	decon_dqe_aosp_colors_store);

static struct attribute *dqe_attrs[] = {
	&dev_attr_gamma.attr,
	&dev_attr_cgc.attr,
	&dev_attr_hsc.attr,
	&dev_attr_xml.attr,
	&dev_attr_tune_mode1.attr,
	&dev_attr_tune_mode2.attr,
	&dev_attr_aosp_colors.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dqe);

int decon_dqe_set_color_mode(struct decon_color_mode_with_render_intent_info *mode)
{
	int i, ret = 0;
	struct dqe_device *dqe = dqe_drvdata;
	struct decon_device *decon = get_decon_drvdata(0);

	mutex_lock(&dqe->lock);

	if (decon) {
		if ((decon->state == DECON_STATE_OFF) ||
			(decon->state == DECON_STATE_INIT)) {
			dqe_err("decon is not enabled!(%d)\n", decon->state);
			ret = -1;
			goto err;
		}
	} else {
		dqe_err("decon is NULL!\n");
		ret = -1;
		goto err;
	}

	dqe_info("%s : %d\n", __func__, mode->color_mode);

	switch (mode->color_mode) {
	case HAL_COLOR_MODE_SRGB:
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[1][i];
		break;

	case HAL_COLOR_MODE_DCI_P3:
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[2][i];
		break;

	default:
		for (i = 0; i < 50; i++)
			cgc_lut[i] = tune_mode[0][i];
		break;
	}

	dqe_cgc_lut_set();
	if (!dqe->ctx.night_light_on)
		dqe_gamma_lut_set();

	dqe->ctx.color_mode = mode->color_mode;

	if (!dqe->ctx.night_light_on)
		dqe->ctx.cgc_on = DQE_CGC_ON_MASK;
	else
		dqe->ctx.cgc_on = 0;

	dqe->ctx.need_udpate = true;

	dqe_restore_context();
	decon_reg_update_req_dqe(decon->id);
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : ret(%d), cgc:%d, night_light:%d\n", __func__,
		ret, dqe->ctx.cgc_on, dqe->ctx.night_light_on);

	return ret;
}

static void dqe_color_transform_make_sfr(int coeff[16], int inR, int inG, int inB, int *outR, int *outG, int *outB)
{
	*outR = (coeff[ 0] * inR) >> 15;
	*outR = (*outR & 0x1) ? (*outR >> 1) + 1 : *outR >> 1;
	*outG = (coeff[ 5] * inG) >> 15;
	*outG = (*outG & 0x1) ? (*outG >> 1) + 1 : *outG >> 1;
	*outB = (coeff[10] * inB) >> 15;
	*outB = (*outB & 0x1) ? (*outB >> 1) + 1 : *outB >> 1;
}

static void dqe_color_transform_night_light(int in[16], int out[16])
{
	int kk, jj;
	long long int inverse_i[16] = {60847, 1269, 1269, 0, 4260, 63838, 4260, 0, 429, 429, 60007, 0, 0, 0, 0, 65536};

	for (kk = 0; kk < 4; kk++)
		for (jj = 0; jj < 4; jj++)
			out[kk * 4 + jj] = (inverse_i[kk * 4] * in[jj] + inverse_i[kk * 4 + 1] * in[jj + 4] + inverse_i[kk * 4 + 2] * in[jj + 8] + inverse_i[kk * 4 + 3] * in[jj + 12]) >> 16;
}

int decon_dqe_set_color_transform(struct decon_color_transform_info *transform)
{
	int ret = 0;
	int i, j;
	int temp[16];
	struct dqe_device *dqe = dqe_drvdata;
	struct decon_device *decon = get_decon_drvdata(0);

	mutex_lock(&dqe->lock);

	if (decon) {
		if ((decon->state == DECON_STATE_OFF) ||
			(decon->state == DECON_STATE_INIT)) {
			dqe_err("decon is not enabled!(%d)\n", decon->state);
			ret = -1;
			goto err;
		}
	} else {
		dqe_err("decon is NULL!\n");
		ret = -1;
		goto err;
	}

	if (transform->matrix[0] != 65536/*transform->hint*/) {
		for (i = 0; i < 16; i++)
			dqe_dbg("matrix[%d] = %d\n", i, transform->matrix[i]);

		for (i = 0; i < 16; i++)
			temp[i] = transform->matrix[i];

		dqe_color_transform_night_light(temp, transform->matrix);

		for (i = 0; i < 16; i++)
			dqe_dbg("night[%d] = %d\n", i, transform->matrix[i]);
	}

	if (transform->matrix[0] == 65536 &&
		transform->matrix[5] == 65536 &&
		transform->matrix[10] == 65536)
		dqe->ctx.night_light_on = 0;
	else
		dqe->ctx.night_light_on = 1;

		for (i = 0; i < 3; i++)
			for (j = 0; j < 65; j++)
				gamma_lut[i][j] = bypass_gamma_tune[i][j];

	for (j = 0; j < 65; j++) {
		int inR, inG, inB, outR, outG, outB;

		inR = gamma_lut[0][j];
		inG = gamma_lut[1][j];
		inB = gamma_lut[2][j];

		dqe_color_transform_make_sfr(transform->matrix, inR, inG, inB, &outR, &outG, &outB);

		gamma_lut[0][j] = (u32)outR;
		gamma_lut[1][j] = (u32)outG;
		gamma_lut[2][j] = (u32)outB;
	}

	for (i = 0; i < 3; i++)
		for (j = 0; j < 65; j++)
			dqe_dbg("%d ", gamma_lut[i][j]);

	dqe_gamma_lut_set();

	dqe->ctx.gamma_on = DQE_GAMMA_ON_MASK;
	dqe->ctx.need_udpate = true;

	dqe_restore_context();
	decon_reg_update_req_dqe(decon->id);
err:
	mutex_unlock(&dqe->lock);

	dqe_info("%s : ret=%d color_mode=%d, hint=%d\n",
		__func__,ret , dqe->ctx.color_mode, transform->hint);

	return ret;
}

void decon_dqe_enable(struct decon_device *decon)
{
	u32 val;

	if (decon->id)
		return;

	dqe_dbg("%s\n", __func__);

	dqe_restore_context();
	dqe_reg_start(decon->id, decon->lcd_info);

	val = dqe_read(DQECON);
	dqe_info("dqe gamma:%d cgc:%d hsc:%d\n",
			DQE_GAMMA_ON_GET(val),
			DQE_CGC_ON_GET(val),
			DQE_HSC_ON_GET(val));
}

void decon_dqe_disable(struct decon_device *decon)
{
	if (decon->id)
		return;

	dqe_dbg("%s\n", __func__);

	dqe_save_context();
	dqe_reg_stop(decon->id);
}

int decon_dqe_create_interface(struct decon_device *decon)
{
	int ret = 0;
	struct dqe_device *dqe;

	if (decon->id || (decon->dt.out_type != DECON_OUT_DSI)) {
		dqe_info("decon%d doesn't need dqe interface\n", decon->id);
		return 0;
	}

	dqe = kzalloc(sizeof(struct dqe_device), GFP_KERNEL);
	if (!dqe) {
		ret = -ENOMEM;
		goto exit0;
	}

	dqe_drvdata = dqe;
	dqe->decon = decon;

	if (IS_ERR_OR_NULL(dqe_class)) {
		dqe_class = class_create(THIS_MODULE, "dqe");
		if (IS_ERR_OR_NULL(dqe_class)) {
			pr_err("failed to create dqe class\n");
			ret = -EINVAL;
			goto exit1;
		}

		dqe_class->dev_groups = dqe_groups;
	}

	dqe->dev = device_create(dqe_class, decon->dev, 0,
			&dqe, "dqe", 0);
	if (IS_ERR_OR_NULL(dqe->dev)) {
		pr_err("failed to create dqe device\n");
		ret = -EINVAL;
		goto exit2;
	}

	mutex_init(&dqe->lock);
	dev_set_drvdata(dqe->dev, dqe);

	dqe_load_context();

	dqe_init_context();

	dqe_info("decon_dqe_create_interface done.\n");

	return ret;
exit2:
	class_destroy(dqe_class);
exit1:
	kfree(dqe);
exit0:
	return ret;
}
