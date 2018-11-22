/* drivers/video/fbdev/exynos/dpu20/panels/nov36672a_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2018 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "nov36672a_param.h"
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

#define NOV36672A_VIDEO_VBP	8
#define NOV36672A_VIDEO_VFP	10
#define NOV36672A_VIDEO_VSA	2
#define NOV36672A_VIDEO_HBP	40
#define NOV36672A_VIDEO_HFP	40
#define NOV36672A_VIDEO_HSA	20

#define NOV36672A_HORIZONTAL	1080
#define NOV36672A_VERTICAL	2520

#ifdef FW_TEST /* This information is moved to DT */
#define CONFIG_FB_I80_COMMAND_MODE

struct decon_lcd nov36672a_lcd_info = {
	.mode = DECON_VIDEO_MODE,
	.vfp = NOV36672A_VIDEO_VFP,
	.vbp = NOV36672A_VIDEO_VBP,
	.hfp = NOV36672A_VIDEO_HFP,
	.hbp = NOV36672A_VIDEO_HBP,
	.vsa = NOV36672A_VIDEO_VSA,
	.hsa = NOV36672A_VIDEO_HSA,
	.xres = NOV36672A_HORIZONTAL,
	.yres = NOV36672A_VERTICAL,

	/* Maybe, width and height will be removed */
	.width = 80,
	.height = 120,

	/* Mhz */
	.hs_clk = 1440,
	.esc_clk = 10,

	.fps = 60,
	.mic_enabled = 0,
	.mic_ver = MIC_VER_1_2,
};
#endif

/*
 * NOV36672A lcd init sequence
 *
 * Parameters
 *	- mic : if mic is enabled, MIC_ENABLE command must be sent
 *	- mode : LCD init sequence depends on command or video mode
 */
void nov36672a_lcd_init(int id, struct decon_lcd *lcd)
{

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_0[0],
				SEQ_CMD_0[1]) < 0)
		dsim_err("fail to send SEQ_CMD_0 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_1[0],
				SEQ_CMD_1[1]) < 0)
		dsim_err("fail to send SEQ_CMD_1 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				SEQ_CMD_2[0],
				SEQ_CMD_2[1]) < 0)
		dsim_err("fail to send SEQ_CMD_2 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_SLEEP_OUT[0], 0) < 0)
		dsim_err("fail to send SEQ_SLEEP_OUT command.\n");
	mdelay(125);
}

void nov36672a_lcd_enable(int id)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_DISPLAY_ON[0], 0) < 0)
		dsim_err("fail to send SEQ_DISPLAY_ON command.\n");
	mdelay(20);
}

void nov36672a_lcd_disable(int id)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_DISPLAY_OFF[0], 0) < 0)
		dsim_err("fail to send SEQ_DISPLAY_OFF command.\n");
	mdelay(20);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_SLEEP_IN[0], 0) < 0)
		dsim_err("fail to send SEQ_SLEEP_IN command.\n");
	mdelay(125);
}

/*
 * Set gamma values
 *
 * Parameter
 *	- backlightlevel : It is from 0 to 26.
 */
int nov36672a_lcd_gamma_ctrl(int id, u32 backlightlevel)
{
	return 0;
}

int nov36672a_lcd_gamma_update(int id)
{
	return 0;
}
