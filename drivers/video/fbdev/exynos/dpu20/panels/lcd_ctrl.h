/* linux/drivers/video/decon_display/s6e3fa0_gamma.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *
 * Haowe Li <haowei.li@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __LCD_CTRL_H__
#define __LCD_CTRL_H__

#include "decon_lcd.h"

void s6e3fa0_lcd_init(int id, struct decon_lcd *lcd);
void s6e3fa0_lcd_enable(int id);
void s6e3fa0_lcd_disable(int id);
void s6e3fa0_lcd_sleepin(int id);
void s6e3fa0_lcd_sleepout(int id);
int s6e3fa0_lcd_gamma_ctrl(int id, unsigned int backlightlevel);
int s6e3fa0_lcd_gamma_update(int id);
int s6e3fa0_lcd_dump(int id);
void s6e3fa0_lcd_mres(int id, int mres_idx, int dsc_en);
void s6e3fa0_lcd_lane_ctl(int id, unsigned int lane_num);

void nt36672a_lcd_init(int id, struct decon_lcd *lcd);
void nt36672a_lcd_enable(int id);
void nt36672a_lcd_disable(int id);
void nt36672a_lcd_sleepin(int id);
void nt36672a_lcd_sleepout(int id);
int nt36672a_lcd_gamma_ctrl(int id, unsigned int backlightlevel);
int nt36672a_lcd_gamma_update(int id);
int nt36672a_lcd_dump(int id);
void nt36672a_lcd_mres(int id, int mres_idx, int dsc_en);
void nt36672a_lcd_lane_ctl(int id, unsigned int lane_num);

void default_lcd_init(int id, struct decon_lcd *lcd);
void default_lcd_enable(int id);
void default_lcd_disable(int id);
void default_lcd_sleepin(int id);
void default_lcd_sleepout(int id);
int default_lcd_gamma_ctrl(int id, unsigned int backlightlevel);
int default_lcd_gamma_update(int id);
int default_lcd_dump(int id);
void default_lcd_mres(int id, int mres_idx, int dsc_en);
void default_lcd_lane_ctl(int id, unsigned int lane_num);

#endif /* __LCD_CTRL_H__ */
