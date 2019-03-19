/* drivers/video/fbdev/exynos/dpu20/panels/hix83112a_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2018 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "hix83112a_param.h"
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

#define HIX83112A_VIDEO_VBP	36
#define HIX83112A_VIDEO_VFP	27
#define HIX83112A_VIDEO_VSA	5
#define HIX83112A_VIDEO_HBP	8
#define HIX83112A_VIDEO_HFP	22
#define HIX83112A_VIDEO_HSA	5

#define HIX83112A_HORIZONTAL	1080
#define HIX83112A_VERTICAL	2520

#ifdef FW_TEST /* This information is moved to DT */
#define CONFIG_FB_I80_COMMAND_MODE

struct decon_lcd hix83112a_lcd_info = {
	.mode = DECON_VIDEO_MODE,
	.vfp = HIX83112A_VIDEO_VFP,
	.vbp = HIX83112A_VIDEO_VBP,
	.hfp = HIX83112A_VIDEO_HFP,
	.hbp = HIX83112A_VIDEO_HBP,
	.vsa = HIX83112A_VIDEO_VSA,
	.hsa = HIX83112A_VIDEO_HSA,
	.xres = HIX83112A_HORIZONTAL,
	.yres = HIX83112A_VERTICAL,

	/* Maybe, width and height will be removed */
	.width = 80,
	.height = 120,

	/* Mhz */
	.hs_clk = 1300,
	.esc_clk = 20,

	.fps = 60,
	.mic_enabled = 0,
	.mic_ver = MIC_VER_1_2,
};
#endif

/*
 * HIX83112A lcd init sequence
 *
 * Parameters
 *	- mic : if mic is enabled, MIC_ENABLE command must be sent
 *	- mode : LCD init sequence depends on command or video mode
 */
void hix83112a_lcd_init(int id, struct decon_lcd *lcd)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_CMD_0,
				ARRAY_SIZE(SEQ_CMD_0)) < 0)
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

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_PASSWORD,
		ARRAY_SIZE(SEQ_PASSWORD)) < 0)
	dsim_err("fail to send SEQ_PASSWORD command.\n");
	mdelay(1);
	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_CMD_3,
		ARRAY_SIZE(SEQ_CMD_3)) < 0)
	dsim_err("fail to send SEQ_CMD_3 command.\n");
	mdelay(1);
	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_CMD_4,
		ARRAY_SIZE(SEQ_CMD_4)) < 0)
	dsim_err("fail to send SEQ_CMD_4 command.\n");
	mdelay(1);
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_5[0],
		SEQ_CMD_5[1]) < 0)
	dsim_err("fail to send SEQ_CMD_5 command.\n");
	mdelay(1);
	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_CMD_6,
		ARRAY_SIZE(SEQ_CMD_6)) < 0)
	dsim_err("fail to send SEQ_CMD_6 command.\n");
	mdelay(1);
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_7[0],
		SEQ_CMD_7[1]) < 0)
	dsim_err("fail to send SEQ_CMD_7 command.\n");
	mdelay(1);
	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_OTP_DISABLE,
		ARRAY_SIZE(SEQ_OTP_DISABLE)) < 0)
	dsim_err("fail to send SEQ_OTP_DISABLE command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_8[0],
		SEQ_CMD_8[1]) < 0)
	dsim_err("fail to send SEQ_CMD_8 command.\n");
	mdelay(5);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_12[0],
		SEQ_CMD_12[1]) < 0)
	dsim_err("fail to send SEQ_CMD_12 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_13[0],
		SEQ_CMD_13[1]) < 0)
	dsim_err("fail to send SEQ_CMD_13 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_14[0],
		SEQ_CMD_14[1]) < 0)
	dsim_err("fail to send SEQ_CMD_14 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_15[0],
		SEQ_CMD_15[1]) < 0)
	dsim_err("fail to send SEQ_CMD_15 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		SEQ_CMD_16[0],
		SEQ_CMD_16[1]) < 0)
	dsim_err("fail to send SEQ_CMD_16 command.\n");
	mdelay(1);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_SLEEP_OUT[0], 0) < 0)
		dsim_err("fail to send SEQ_SLEEP_OUT command.\n");
	mdelay(125);
}

void hix83112a_lcd_enable(int id)
{
	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, SEQ_DISPLAY_ON[0], 0) < 0)
		dsim_err("fail to send SEQ_DISPLAY_ON command.\n");
	mdelay(20);
}

void hix83112a_lcd_disable(int id)
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
int hix83112a_lcd_gamma_ctrl(int id, u32 backlightlevel)
{
	return 0;
}

int hix83112a_lcd_gamma_update(int id)
{
	return 0;
}
