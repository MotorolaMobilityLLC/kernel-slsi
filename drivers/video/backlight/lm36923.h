#ifndef _LM36923_H_
#define _LM36923_H_

#include <linux/delay.h>
#include <linux/i2c.h>

#define BK_I2C_RETRIES 1   //times for try i2c write/read operation
#define BK_I2C_RETRY_DELAY 1  //time between the current and the last try

//reg addr used for compatibility
#define BACKLIGHT_CHIP_ID_REG       0x10 //chip id
#define BACKLIGHT_CHIP_ID_REG_MASK  0x10
#define LM_LSB_MASK 0x07

//lm
#define LM_REG_NUM 13
#define LM_REV_REG 0x00
#define LM_SFRESET_REG 0x01
#define LM_ENABLE_REG 0x10
#define LM_BRCTL_REG 0x11
#define LM_PWMCTL_REG 0x12
#define LM_BSTCTL_REG 0x13
#define LM_AUTOFRQ_HIGH_TH 0x15
#define LM_AUTOFRQ_LOW_TH 0x16
#define LM_BK_AD_TRH 0x17
#define LM_BR_LSB_REG 0x18
#define LM_BR_MSB_REG 0x19
#define LM_FLTCTL_REG 0x1E
#define LM_FLTFLAG_REG 0x1F

unsigned char lm_reg[LM_REG_NUM]={
	LM_REV_REG,
	LM_SFRESET_REG,
	LM_ENABLE_REG,
	LM_BRCTL_REG,
	LM_PWMCTL_REG,
	LM_BSTCTL_REG,
	LM_AUTOFRQ_HIGH_TH,
	LM_AUTOFRQ_LOW_TH,
	LM_BK_AD_TRH,
	LM_BR_LSB_REG,
	LM_BR_MSB_REG,
	LM_FLTCTL_REG,
	LM_FLTFLAG_REG
};

struct lm36923_data {
	struct i2c_client *bk_i2c_client;
	struct mutex lock;
	unsigned char mode;
	char* name;
};

//lm 36923
unsigned char lm_nr_data[][2]=
{
    {0x18, 0x06},
    {0x19, 0xCC},
};

unsigned char lm_hl_data[][2]=
{
    {0x18, 0x07},
    {0x19, 0xFF},
};
static int backlight_i2c_write(struct lm36923_data *i2c_data,
         unsigned char reg_addr, unsigned char reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < BK_I2C_RETRIES) {
        ret = i2c_smbus_write_byte_data(i2c_data->bk_i2c_client, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        }else {
            break;
        }
        cnt ++;
        msleep(BK_I2C_RETRY_DELAY);
    }

    return ret;
}

static int backlight_i2c_read(struct lm36923_data *i2c_data,
        unsigned char reg_addr, unsigned char *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < BK_I2C_RETRIES) {
        ret = i2c_smbus_read_byte_data(i2c_data->bk_i2c_client, reg_addr);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        }else {
            *reg_data = ret;
            break;
        }
        cnt ++;
        msleep(BK_I2C_RETRY_DELAY);
    }

    return ret;
}

static int backlight_i2c_write_bits(struct lm36923_data *i2c_data,
         unsigned char reg_addr, unsigned char mask, unsigned char reg_data)
{
    int ret = -1;
    unsigned char reg_val = 0;

    ret = backlight_i2c_read(i2c_data, reg_addr, &reg_val);
    if(ret < 0) {
        pr_err("%s: read reg[0x%x] last value error\n", __func__, reg_addr);
    }
    reg_val &= (~mask);
    reg_val |= (reg_data&mask);
    msleep(1);
    ret = backlight_i2c_write(i2c_data, reg_addr, reg_val);
    if(ret < 0) {
        pr_err("%s: write new value[0x%x] to reg[0x%x] error\n", __func__, reg_val ,reg_addr);
    }
    return ret;
}
#endif