/*
 * Copyright (C) 2019 Motorola Mobility LLC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MOT_BU24218_FACTORY_H
#define MOT_BU24218_FACTORY_H
#include "fimc-is-core.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-helper-ois-i2c.h"
#include "fimc-is-ois.h"

#define  MOT_BU24218_REG_MODE                   0x6020
#define  MOT_BU24218_REG_STATUS                 0x6024
#define  MOT_BU24218_REG_GYRO_ACCESS            0x6023
#define  MOT_BU24218_REG_FW_VERSION             0x6010
#define  MOT_BU24218_REG_GYRO_SELF_TEST         0x6104
#define  MOT_BU24218_REG_GYRO_RESULT            0x6105
#define  MOT_BU24218_REG_LINEARITY              0x614F

#define  MOT_BU24218_REG_MAX_XCH                0x6100
#define  MOT_BU24218_REG_MIN_XCH                0x6101
#define  MOT_BU24218_REG_MAX_YCH                0x6102
#define  MOT_BU24218_REG_MIN_YCH                0x6103
#define  MOT_BU24218_REG_POS_XCH                0x6064
#define  MOT_BU24218_REG_POS_YCH                0x6066
#define  MOT_BU24218_REG_LENS_XCH               0x6058
#define  MOT_BU24218_REG_LENS_YCH               0x6059
#define  MOT_BU24218_REG_ADC_VAL                0x6062
#define  MOT_BU24218_REG_ADC_LATCH              0x6060

#define  MOT_BU24218_VCM_DRV_MAX_HI             0x03
#define  MOT_BU24218_VCM_DRV_MAX_LO             0x00
#define  MOT_BU24218_VCM_DRV_MIN_HI             0x0D
#define  MOT_BU24218_VCM_DRV_MIN_LO             0x00
#define  MOT_BU24218_VCM_DRV_ZERO_HI            0x00
#define  MOT_BU24218_VCM_DRV_ZERO_LO            0x00

#define  MOT_BU24218_ADC_LATCH_X                0
#define  MOT_BU24218_ADC_LATCH_Y                1

#define OIS_HEA_NUM_VALUES                      4

int fimc_is_factory_ois_get_fw_rev(struct v4l2_subdev  *subdev, uint32_t *result);
int fimc_is_factory_ois_get_hea(struct v4l2_subdev  *subdev, struct fimc_is_ois_hea_parameters *result);
int fimc_is_factory_ois_poll(struct i2c_client *client, u16 addr, uint8_t expdata);

int fimc_is_factory_ois_calibration_mode(struct i2c_client *client);
int fimc_is_factory_ois_set_zero_pos(struct i2c_client *client);
int fimc_is_factory_ois_set_x_pos(struct i2c_client *client, uint8_t hi, uint8_t lo);
int fimc_is_factory_ois_set_y_pos(struct i2c_client *client, uint8_t hi, uint8_t lo);
int fimc_is_factory_ois_get_x_adc(struct i2c_client *client, uint32_t* data);
int fimc_is_factory_ois_get_y_adc(struct i2c_client *client, uint32_t* data);

#endif
