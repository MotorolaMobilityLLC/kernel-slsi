/* drivers/video/exynos/panels/nt36672a_lcd_ctrl.c
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
#include "nt36672a_param.h"
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

#define NT36672A_VIDEO_VBP	2
#define NT36672A_VIDEO_VFP	10
#define NT36672A_VIDEO_VSA	10
#define NT36672A_VIDEO_HBP	20
#define NT36672A_VIDEO_HFP	40
#define NT36672A_VIDEO_HSA	40

#define NT36672A_HORIZONTAL	1080
#define NT36672A_VERTICAL	2246

#ifdef FW_TEST /* This information is moved to DT */
#define CONFIG_FB_I80_COMMAND_MODE

struct decon_lcd nt36672a_lcd_info = {
	.mode = DECON_VIDEO_MODE,
	.vfp = NT36672A_VIDEO_VFP,
	.vbp = NT36672A_VIDEO_VBP,
	.hfp = NT36672A_VIDEO_HFP,
	.hbp = NT36672A_VIDEO_HBP,
	.vsa = NT36672A_VIDEO_VSA,
	.hsa = NT36672A_VIDEO_HSA,
	.xres = NT36672A_HORIZONTAL,
	.yres = NT36672A_VERTICAL,

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
 * NT36672A lcd init sequence
 *
 * Parameters
 *	- mic : if mic is enabled, MIC_ENABLE command must be sent
 *	- mode : LCD init sequence depends on command or video mode
 */
void nt36672a_lcd_init(int id, struct decon_lcd *lcd)
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

void nt36672a_lcd_enable(int id)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_DISPLAY_ON[0], 0) < 0)
		dsim_err("fail to send SEQ_DISPLAY_ON command.\n");
}

void nt36672a_lcd_disable(int id)
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
int nt36672a_lcd_gamma_ctrl(int id, u32 backlightlevel)
{
	return 0;
}

int nt36672a_lcd_gamma_update(int id)
{
	return 0;
}
