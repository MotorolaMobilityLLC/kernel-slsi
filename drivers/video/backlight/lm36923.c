/*  Date: 2018/11/9 10:00:00
 *  Revision: 1.0
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/backlight.h>
#include "lm36923.h"

#define LM_I2C_NAME "lm"

static int chipid=-1;   //used for compatibility
static unsigned char lm_data[LM_REG_NUM];

static struct lm36923_data* lmdata;
static struct backlight_device *bd;
//LM36923
static void lm_get_reg_val(struct lm36923_data *i2c_data){
    int i;
    for(i=0; i<LM_REG_NUM; i++){
        backlight_i2c_read(i2c_data,lm_reg[i],&lm_data[i]);
        pr_err("%s: lm_data[%d]=0x%x\n", __func__, i, lm_data[i]);
        mdelay(2);
    }
}

/************************************************************
*
*   backlight i2c device attributes
*
*
*************************************************************/
static ssize_t lm36923_chip_name_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct lm36923_data *i2c_data = lmdata;
    if(i2c_data != NULL)
        return scnprintf(buf, PAGE_SIZE, "%s\n", i2c_data->name);
    else
        return scnprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t lm36923_mode_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct lm36923_data *i2c_data = lmdata;
    if(i2c_data != NULL)
        return scnprintf(buf, PAGE_SIZE, "%d\n", i2c_data->mode);
    else
        return scnprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t lm36923_mode_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret;
    unsigned int value = 0;
    struct lm36923_data *i2c_data = lmdata;

    if(!count || i2c_data==NULL){
        pr_err("count=0 or i2c_data is NULL pointer\n");
        return -EINVAL;
    }

    ret = kstrtouint(buf, 0, &value);
    if (ret < 0 || (value!=0&&value!=1)){
        pr_err("wrong params. should be 0 or 1\n");
        return ret;
    }

    pr_err("%s: current mode=%d, now write to %d\n", __func__, i2c_data->mode, value);

    if(1 == value){
        mutex_lock(&i2c_data->lock);
        ret = backlight_i2c_write_bits(i2c_data,lm_hl_data[0][0],LM_LSB_MASK,lm_hl_data[0][1]);  //write LSB
        if(ret < 0){
            pr_err("HL mode:write lm chip LSB bit error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }

        ret = backlight_i2c_write(i2c_data,lm_hl_data[1][0],lm_hl_data[1][1]); //write MSB
        if(ret < 0){
            pr_err("HL mode:write lm chip MSB error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }
        i2c_data->mode = value;
        mutex_unlock(&i2c_data->lock);
    }
    else if(0 == value){
        mutex_lock(&i2c_data->lock);
        ret = backlight_i2c_write_bits(i2c_data,lm_nr_data[0][0],LM_LSB_MASK,lm_nr_data[0][1]);  //write LSB
        if(ret < 0){
            pr_err("NR mode:write lm chip LSB bit error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }

        ret = backlight_i2c_write(i2c_data,lm_nr_data[1][0],lm_nr_data[1][1]); //write MSB
        if(ret < 0){
            pr_err("NR mode:write lm chip MSB error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }
        i2c_data->mode = value;
        mutex_unlock(&i2c_data->lock);
    }
    else{
        pr_err("the error echo value, 0 or 1 is allowed only\n");
    }
exit:
    return count;
}

static ssize_t lm36923_chip_reg_show(struct device *dev,
        struct device_attribute *attr,char *buf)
{
    int i;
    char reg_data_show[180]="";
    char reg_tmp[13];
    struct lm36923_data *i2c_data = lmdata;

    if(i2c_data == NULL)
        return scnprintf(buf, PAGE_SIZE, "NULL\n");
    lm_get_reg_val(i2c_data);
    for(i=0; i<LM_REG_NUM; i++){
        snprintf(reg_tmp, sizeof(reg_tmp), "[0x%2x]=0x%2x\n", lm_reg[i],lm_data[i]);
        //pr_err("reg_tmp=%s\0",reg_tmp);
        strncat(reg_data_show,reg_tmp,strlen(reg_tmp));
    }

    return scnprintf(buf, PAGE_SIZE, "%s\n", reg_data_show);
}

static ssize_t lm36923_chip_reg_store(struct device *dev,
        struct device_attribute *attr,const char *buf, size_t count)
{
    int ret = -1;
    unsigned char reg;
    unsigned char val;
    struct lm36923_data *i2c_data = lmdata;
    if(!count || i2c_data==NULL){
        pr_err("%s:count=0 or i2c_data is NULL pointer\n",__func__);
        return -EINVAL;
    }

    if (sscanf(buf, "%x %x", &reg, &val) == 2)
    {
        pr_err("Notice chip id and write the corrent reg\n");
        pr_err("%s,reg = 0x%02x, val = 0x%02x\n", __func__, reg, val);
        ret = backlight_i2c_write(i2c_data,reg,val);
        if(ret < 0)
            pr_err("write chip reg[0x%x] error\n",reg);
    }
    else{
        pr_err("write echo command error; like this: echo 0x19 0xFF > chip_reg\n");
    }
    return count;
}

static DEVICE_ATTR(chip_reg, 0644, lm36923_chip_reg_show, lm36923_chip_reg_store);
static DEVICE_ATTR(hbm_mode, 0644, lm36923_mode_show, lm36923_mode_store);
static DEVICE_ATTR(chip_name, 0444, lm36923_chip_name_show, NULL);

static struct attribute *lm36923_attributes[] = {
    &dev_attr_hbm_mode.attr,
    &dev_attr_chip_name.attr,
    &dev_attr_chip_reg.attr,
    NULL
};

static struct attribute_group lm36923_attribute_group = {
    .attrs = lm36923_attributes
};

/************************************************************
*
*   backlight i2c device init
*
*
*************************************************************/
static int lm36923_parse_dt(struct device *dev,
                struct lm36923_data *pdata, struct device_node *np)
{
    u32 regaddr;
    int ret;

    ret = of_property_read_u32(np, "reg", &regaddr);
    if(ret){
        printk(KERN_ERR "parse backlight_i2c dtsi node failed");
        return -EINVAL;
    }

    return 0;
}

static int getChipId(struct device *dev){
    int ret;
    unsigned char value = 0;
    struct lm36923_data *i2c_data = lmdata;

    if(i2c_data == NULL){
        pr_err("%s: i2c_data is NULL\n");
        return 1;
    }

    ret = backlight_i2c_read(i2c_data,BACKLIGHT_CHIP_ID_REG,&value);
    if(ret < 0){
        pr_err("%s: get 0x10 register for compatibility error\n");
        return 1;
    }
    printk(KERN_ERR "%s:read chip id reg:0x%x, value=0x%x\n",__func__,BACKLIGHT_CHIP_ID_REG,value);
    return (value&BACKLIGHT_CHIP_ID_REG_MASK)>>4;
}


static int lm36923_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    struct lm36923_data *data;
    struct device_node *np=client->dev.of_node;

    pr_err("%s: enter\n",__func__);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_info(KERN_ERR "i2c_check_functionality error\n");
        return -EIO;
    }

    data = devm_kzalloc(&client->dev,sizeof(struct lm36923_data), GFP_KERNEL);
    if (!data) {
        pr_err("%s: kzalloc error\n",__func__);
        return -ENOMEM;
    }

    data->bk_i2c_client = client;
    mutex_init(&data->lock);
    data->mode = 0;

    i2c_set_clientdata(client, data);
    lmdata=data;

    if (np) {
        ret = lm36923_parse_dt(&client->dev, data, np);
        if (ret) {
            dev_err(&client->dev,"Unable to parse platfrom data err=%d\n", ret);
            goto kfree_exit;
        }
    }else{
        dev_err(&client->dev,"backlight_i2c dtsi node not exists\n");
        ret = -ENODEV;
        goto kfree_exit;

    }

    chipid=getChipId(&client->dev);
    printk(KERN_ERR "%s: chipid=%d\n", __func__,chipid);

    if(0==chipid){
        printk(KERN_ERR "lm36923 chip\n");
        mutex_lock(&data->lock);
        data->name="lm36923\0";
        ret = backlight_i2c_write_bits(data,lm_nr_data[0][0],LM_LSB_MASK,lm_nr_data[0][1]);
        if(ret < 0){
            pr_err("%s:NR mode:write lm chip LSB bit error\n",__func__);
        }

        ret = backlight_i2c_write(data,lm_nr_data[1][0],lm_nr_data[1][1]); //write MSB
        if(ret < 0){
            pr_err("%s:NR mode:write lm chip MSB error\n",__func__);
        }
        mutex_unlock(&data->lock);
    }
    else{
        dev_err(&client->dev,"wrong chipid\n");
        ret = -1;
        goto kfree_exit;
    }

    bd = backlight_device_register("hbm", NULL, NULL, NULL, NULL);
    if (IS_ERR(bd)){
        pr_err("sgm failed to register backlight device!\n");
        ret= -1;
        goto kfree_node;
    }

    ret = sysfs_create_group(&bd->dev.kobj, &lm36923_attribute_group);
    if(ret < 0){
        dev_err(&bd->dev,"Unable to create backlight_i2c_attribute\n");
        goto kfree_sysfs;
    }
    return 0;

kfree_sysfs:
    sysfs_remove_group(&bd->dev.kobj, &lm36923_attribute_group);
kfree_node:
    backlight_device_unregister(bd);
kfree_exit:
    mutex_destroy(&data->lock);
    return ret;
}

static int lm36923_remove(struct i2c_client *client)
{
    struct lm36923_data *data = lmdata;
    if(lmdata != NULL)
        mutex_destroy(&data->lock);
    sysfs_remove_group(&bd->dev.kobj, &lm36923_attribute_group);
    backlight_device_unregister(bd);
    return 0;
}

static const struct i2c_device_id lm36923_id[] = {
    { LM_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, lm36923_id);

static struct of_device_id lm36923_match_table[] = {
    { .compatible = "backlight_i2c", },
    { },
};

static struct i2c_driver lm36923_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = LM_I2C_NAME,
        .of_match_table = lm36923_match_table,
    },
    .id_table   = lm36923_id,
    .probe      = lm36923_probe,
    .remove     = lm36923_remove,

};

static int __init lm36923_init(void)
{
    int ret=0;
    pr_info("lm36923_init\n");

    ret = i2c_add_driver(&lm36923_driver);
    pr_info("lm36923_init ret=%d\n",ret);
    if(ret){
        pr_err("fail to add lm36923_driver\n");
        return ret;
    }

    return 0;
}

static void __exit lm36923_exit(void)
{
    pr_info("lm36923_exit\n");
    i2c_del_driver(&lm36923_driver);
}

MODULE_DESCRIPTION("LCD_BACKLIGHT davicom ic driver");
MODULE_LICENSE("GPL");

module_init(lm36923_init);
module_exit(lm36923_exit);