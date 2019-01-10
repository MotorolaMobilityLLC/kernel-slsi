#ifndef _SGM37603A_H_
#define _SGM37603A_H_

#include <linux/delay.h>
#include <linux/i2c.h>

#define BK_I2C_RETRIES 1   //times for try i2c write/read operation
#define BK_I2C_RETRY_DELAY 1  //time between the current and the last try

//reg addr used for compatibility
#define BACKLIGHT_CHIP_ID_REG       0x10 //chip id
#define BACKLIGHT_CHIP_ID_REG_MASK  0x10
#define SGM_LSB_MASK 0x0F

//sgm
#define SGM_REG_NUM 7
#define SGM_SFRESET_REG 0x01
#define SGM_ENABLE_REG 0x10
#define SGM_BRCTL_REG 0x11
#define SGM_BR_LSB_REG 0x1A
#define SGM_BR_MSB_REG 0x19
#define SGM_FLTFLAG_REG 0x1F
#define SGM_MAXLEDCURR_REG 0x1B

unsigned char sgm_reg[SGM_REG_NUM]={
	SGM_SFRESET_REG,
	SGM_ENABLE_REG,
	SGM_BRCTL_REG,
	SGM_BR_LSB_REG,
	SGM_BR_MSB_REG,
	SGM_FLTFLAG_REG,
	SGM_MAXLEDCURR_REG
};

struct sgm37603a_data {
	struct i2c_client *bk_i2c_client;
	struct mutex lock;
	unsigned char mode;
	char* name;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	unsigned char is_suspend;
};

unsigned char sgm_nr_data[][2]=
{
    {0x1A, 0x0C},
    {0x19, 0xCC},
};

unsigned char sgm_hl_data[][2]=
{
    {0x1A, 0x0F},
    {0x19, 0xFF},
};

static int backlight_i2c_write(struct sgm37603a_data *i2c_data,
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

static int backlight_i2c_read(struct sgm37603a_data *i2c_data,
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

static int backlight_i2c_write_bits(struct sgm37603a_data *i2c_data,
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