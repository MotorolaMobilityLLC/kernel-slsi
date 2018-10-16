/* drivers/video/fbdev/exynos/dpu20/panels/default_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2018 Samsung Electronics
 *
 * Hwangjae Lee, <hj-yo.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "default_param.h"
#include "lcd_ctrl.h"

/* use FW_TEST definition when you test CAL on firmware */
/* #define FW_TEST */
#ifdef FW_TEST
#include "../dsim_fw.h"
#include "mipi_display.h"
#else
#include "../dsim.h"
#include <video/mipi_display.h>
#endif

#define GAMMA_PARAM_SIZE 26

#define DEFAULT_VIDEO_VBP	6
#define DEFAULT_VIDEO_VFP	36
#define DEFAULT_VIDEO_VSA	2
#define DEFAULT_VIDEO_HBP	44
#define DEFAULT_VIDEO_HFP	44
#define DEFAULT_VIDEO_HSA	12

#define DEFAULT_HORIZONTAL	1080
#define DEFAULT_VERTICAL	2270

#ifdef FW_TEST /* This information is moved to DT */
#define CONFIG_FB_I80_COMMAND_MODE

struct decon_lcd default_lcd_info = {
	.mode = DECON_VIDEO_MODE,
	.vfp = DEFAULT_VIDEO_VFP,
	.vbp = DEFAULT_VIDEO_VBP,
	.hfp = DEFAULT_VIDEO_HFP,
	.hbp = DEFAULT_VIDEO_HBP,
	.vsa = DEFAULT_VIDEO_VSA,
	.hsa = DEFAULT_VIDEO_HSA,
	.xres = DEFAULT_HORIZONTAL,
	.yres = DEFAULT_VERTICAL,

	/* Maybe, width and height will be removed */
	.width = 70,
	.height = 121,

	/* Mhz */
	.hs_clk = 1100,
	.esc_clk = 20,

	.fps = 60,
	.mic_enabled = 0,
	.mic_ver = MIC_VER_1_2,
};
#endif

/*
 * DEFAULT lcd init sequence
 *
 * Parameters
 *	- mic : if mic is enabled, MIC_ENABLE command must be sent
 *	- mode : LCD init sequence depends on command or video mode
 */
void default_lcd_init(int id, struct decon_lcd *lcd)
{
	mdelay(12);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_0[0],
				SEQ_CMD_0[1]) < 0)
		dsim_err("fail to send SEQ_CMD_0 command.\n");
	mdelay(12);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_1[0],
				SEQ_CMD_1[1]) < 0)
		dsim_err("fail to send SEQ_CMD_1 command.\n");
	mdelay(12);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_2[0],
				SEQ_CMD_2[1]) < 0)
		dsim_err("fail to send SEQ_CMD_2 command.\n");
	mdelay(12);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_SLEEP_OUT[0], 0) < 0)
		dsim_err("fail to send SEQ_SLEEP_OUT command.\n");
	mdelay(120);
}

void default_lcd_enable(int id)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_DISPLAY_ON[0], 0) < 0)
		dsim_err("fail to send SEQ_DISPLAY_ON command.\n");
}

void default_lcd_disable(int id)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_DISPLAY_OFF[0], 0) < 0)
		dsim_err("fail to send SEQ_DISPLAY_OFF command.\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_SLEEP_IN[0], 0) < 0)
		dsim_err("fail to send SEQ_SLEEP_IN command.\n");
	mdelay(100);
}

/*
 * Set gamma values
 *
 * Parameter
 *	- backlightlevel : It is from 0 to 26.
 */
int default_lcd_gamma_ctrl(int id, u32 backlightlevel)
{
	return 0;
}

int default_lcd_gamma_update(int id)
{
	return 0;
}
