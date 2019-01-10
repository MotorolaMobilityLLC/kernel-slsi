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
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#endif
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/backlight.h>
#include "sgm37603a.h"

#define SGM_I2C_NAME "sgm"

static int chipid=-1;   //used for compatibility
static unsigned char sgm_data[SGM_REG_NUM];

static struct sgm37603a_data* sgmdata;
static struct backlight_device *bd;

//SGM37603A
static void sgm_get_reg_val(struct sgm37603a_data *i2c_data){
    int i;
    for(i=0; i<SGM_REG_NUM; i++){
        backlight_i2c_read(i2c_data,sgm_reg[i],&sgm_data[i]);
        pr_err("%s: sgm_data[%d]=0x%x\n", __func__, i, sgm_data[i]);
        mdelay(2);
    }
}

/************************************************************
*
*   backlight i2c device attributes
*
*
*************************************************************/
static ssize_t sgm37603a_chip_name_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sgm37603a_data *i2c_data = sgmdata;
    if(i2c_data != NULL)
        return scnprintf(buf, PAGE_SIZE, "%s\n", i2c_data->name);
    else
        return scnprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t sgm37603a_mode_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct sgm37603a_data *i2c_data = sgmdata;
    if(i2c_data != NULL)
        return scnprintf(buf, PAGE_SIZE, "%d\n", i2c_data->mode);
    else
        return scnprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t sgm37603a_mode_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret;
    unsigned int value = 0;
    struct sgm37603a_data *i2c_data = sgmdata;

    if(!count || i2c_data==NULL){
        pr_err("count=0 or i2c_data is NULL pointer\n");
        return -EINVAL;
    }

    ret = kstrtouint(buf, 0, &value);
    if (ret < 0 || (value!=0&&value!=1)){
        pr_err("wrong params. should be 0 or 1\n");
        return ret;
    }

    i2c_data->mode = value;

    if(i2c_data->is_suspend){
        pr_err("device is suspend now, set HBM mode on next power on\n");
        goto exit;
    }

    pr_err("%s: current mode=%d, now write to %d\n", __func__, i2c_data->mode, value);

    if(1 == value){
        if(mutex_trylock(&i2c_data->lock)==0){
            pr_err("%s: sgm37603a dev is busy\n",__func__);
            goto exit;
        }

        ret = backlight_i2c_write_bits(i2c_data,sgm_hl_data[0][0],SGM_LSB_MASK,sgm_hl_data[0][1]);  //write LSB
        if(ret < 0){
            pr_err("HL mode:write sgm chip LSB bit error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }

        ret = backlight_i2c_write(i2c_data,sgm_hl_data[1][0],sgm_hl_data[1][1]); //write MSB
        if(ret < 0){
            pr_err("HL mode:write sgm chip MSB error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }
        mutex_unlock(&i2c_data->lock);
    }
    else if(0 == value){
        if(mutex_trylock(&i2c_data->lock)==0){
            pr_err("%s: sgm37603a dev is busy\n",__func__);
            goto exit;
        }

        ret = backlight_i2c_write_bits(i2c_data,sgm_nr_data[0][0],SGM_LSB_MASK,sgm_nr_data[0][1]);  //write LSB
        if(ret < 0){
            pr_err("NR mode:write sgm chip LSB bit error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }

        ret = backlight_i2c_write(i2c_data,sgm_nr_data[1][0],sgm_nr_data[1][1]); //write MSB
        if(ret < 0){
            pr_err("NR mode:write sgm chip MSB error\n");
            mutex_unlock(&i2c_data->lock);
            goto exit;
        }
        mutex_unlock(&i2c_data->lock);
    }
    else{
        pr_err("the error echo value, 0 or 1 is allowed only\n");
    }
exit:
    return count;
}

static ssize_t sgm37603a_chip_reg_show(struct device *dev,
        struct device_attribute *attr,char *buf)
{
    int i;
    char reg_data_show[180]="";
    char reg_tmp[13];
    struct sgm37603a_data *i2c_data = sgmdata;

    if(i2c_data == NULL)
        return scnprintf(buf, PAGE_SIZE, "NULL\n");

    sgm_get_reg_val(i2c_data);
    for(i=0; i<SGM_REG_NUM; i++){
        snprintf(reg_tmp, sizeof(reg_tmp), "[0x%2x]=0x%2x\n", sgm_reg[i],sgm_data[i]);
        //pr_err("reg_tmp=%s\0",reg_tmp);
        strncat(reg_data_show,reg_tmp,strlen(reg_tmp));
    }

    return scnprintf(buf, PAGE_SIZE, "%s\n", reg_data_show);
}

static ssize_t sgm37603a_chip_reg_store(struct device *dev,
        struct device_attribute *attr,const char *buf, size_t count)
{
    int ret = -1;
    unsigned char reg;
    unsigned char val;
    struct sgm37603a_data *i2c_data = sgmdata;

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

static DEVICE_ATTR(chip_reg, 0644, sgm37603a_chip_reg_show, sgm37603a_chip_reg_store);
static DEVICE_ATTR(hbm_mode, 0644, sgm37603a_mode_show, sgm37603a_mode_store);
static DEVICE_ATTR(chip_name, 0444, sgm37603a_chip_name_show, NULL);

static struct attribute *sgm37603a_attributes[] = {
    &dev_attr_hbm_mode.attr,
    &dev_attr_chip_name.attr,
    &dev_attr_chip_reg.attr,
    NULL
};

static struct attribute_group sgm37603a_attribute_group = {
    .attrs = sgm37603a_attributes
};

/************************************************************
*
*   backlight i2c device init
*
*
*************************************************************/
static int sgm37603a_parse_dt(struct device *dev,
                struct sgm37603a_data *pdata, struct device_node *np)
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
    struct sgm37603a_data *i2c_data = sgmdata;
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

#if defined(CONFIG_FB)
static int sgm37603a_resume(struct sgm37603a_data *dev_data)
{
    int ret = 0;

    dev_data->is_suspend = 0;
    if (!dev_data) {
	    pr_err("%s: kzalloc error\n",__func__);
	    return -ENOMEM;
    }

    pr_info("%s:now write mode to %d\n",__func__,dev_data->mode);
    if(1 == dev_data->mode){
	        if(mutex_trylock(&dev_data->lock)==0){
	            pr_err("%s: sgm37603a dev is busy\n",__func__);
	            goto exit;
	        }

	        ret = backlight_i2c_write_bits(dev_data,sgm_hl_data[0][0],SGM_LSB_MASK,sgm_hl_data[0][1]);  //write LSB
	        if(ret < 0){
	            pr_err("HL mode:write sgm chip LSB bit error\n");
	            mutex_unlock(&dev_data->lock);
	            goto exit;
	        }

	        ret = backlight_i2c_write(dev_data,sgm_hl_data[1][0],sgm_hl_data[1][1]); //write MSB
	        if(ret < 0){
	            pr_err("HL mode:write sgm chip MSB error\n");
	            mutex_unlock(&dev_data->lock);
	            goto exit;
	        }
	        mutex_unlock(&dev_data->lock);
    }else if(0 == dev_data->mode){
	        if(mutex_trylock(&dev_data->lock)==0){
	            pr_err("%s: sgm37603a dev is busy\n",__func__);
	            goto exit;
	        }

	        ret = backlight_i2c_write_bits(dev_data,sgm_nr_data[0][0],SGM_LSB_MASK,sgm_nr_data[0][1]);  //write LSB
	        if(ret < 0){
	            pr_err("NR mode:write sgm chip LSB bit error\n");
	            mutex_unlock(&dev_data->lock);
	            goto exit;
	        }

	        ret = backlight_i2c_write(dev_data,sgm_nr_data[1][0],sgm_nr_data[1][1]); //write MSB
	        if(ret < 0){
	            pr_err("NR mode:write sgm chip MSB error\n");
	            mutex_unlock(&dev_data->lock);
	            goto exit;
	        }
	        mutex_unlock(&dev_data->lock);
    }
    else{
        pr_err("the error echo value, 0 or 1 is allowed only\n");
    }
exit:
    return ret;
}
static int sgm37603a_suspend(struct sgm37603a_data *dev_data)
{
    int ret = 0;
    dev_data->is_suspend = 1;
    return ret;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
    int *blank;
    struct fb_event *evdata = data;
    struct sgm37603a_data *sgm =
		container_of(self, struct sgm37603a_data, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK ) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			sgm37603a_resume(sgm);
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			sgm37603a_suspend(sgm);
			break;
		default:
			break;
		}
    }
    return 0;
}
#endif
static int sgm37603a_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    struct sgm37603a_data *sgm;
    struct device_node *np=client->dev.of_node;

    pr_err("%s: enter\n",__func__);
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_info(KERN_ERR "i2c_check_functionality error\n");
        return -EIO;
    }

    sgm = devm_kzalloc(&client->dev,sizeof(struct sgm37603a_data), GFP_KERNEL);
    if (!sgm) {
        pr_err("%s: kzalloc error\n",__func__);
        return -ENOMEM;
    }

    sgm->bk_i2c_client = client;
    mutex_init(&sgm->lock);
    sgm->mode = 0;
	sgm->is_suspend = 0;
    i2c_set_clientdata(client, sgm);
    sgmdata=sgm;

    if (np) {
        ret = sgm37603a_parse_dt(&client->dev, sgm, np);
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

    if(1==chipid){
        printk(KERN_ERR "sgm37603 chip\n");
        mutex_lock(&sgm->lock);
        sgm->name="sgm37603\0";
        ret = backlight_i2c_write_bits(sgm,sgm_nr_data[0][0],SGM_LSB_MASK,sgm_nr_data[0][1]);
        if(ret < 0){
            pr_err("%s:NR mode:write sgm chip LSB bit error\n",__func__);
        }
        //backlight_i2c_write(data,sgm_nr_data[0][0],sgm_nr_data[0][1]); //write LSB
        ret = backlight_i2c_write(sgm,sgm_nr_data[1][0],sgm_nr_data[1][1]); //write MSB
        if(ret < 0){
            pr_err("%s:NR mode:write sgm chip MSB error\n",__func__);
        }
        mutex_unlock(&sgm->lock);
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

    ret = sysfs_create_group(&bd->dev.kobj, &sgm37603a_attribute_group);
    if(ret < 0){
        dev_err(&bd->dev,"Unable to create backlight_i2c_attribute\n");
        goto kfree_sysfs;
    }
#if defined(CONFIG_FB)
    sgm->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&sgm->fb_notif);
    if (ret) {
		pr_err("Unable to register fb_notifier: %d\n", ret);
		goto err_register_fb_notif_failed;
    }
#endif
    return 0;
#if defined(CONFIG_FB)
err_register_fb_notif_failed:
#endif
kfree_sysfs:
    sysfs_remove_group(&bd->dev.kobj, &sgm37603a_attribute_group);
kfree_node:
    backlight_device_unregister(bd);
kfree_exit:
    mutex_destroy(&sgm->lock);
    return ret;
}

static int sgm37603a_remove(struct i2c_client *client)
{
    struct sgm37603a_data *sgm = sgmdata;
    if(sgmdata != NULL)
        mutex_destroy(&sgm->lock);
    sysfs_remove_group(&bd->dev.kobj, &sgm37603a_attribute_group);
    backlight_device_unregister(bd);
    return 0;
}

static const struct i2c_device_id sgm37603a_id[] = {
    { SGM_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sgm37603a_id);

static struct of_device_id sgm37603a_match_table[] = {
    { .compatible = "backlight_i2c", },
    { },
};

static struct i2c_driver sgm37603a_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = SGM_I2C_NAME,
        .of_match_table = sgm37603a_match_table,
    },
    .id_table   = sgm37603a_id,
    .probe      = sgm37603a_probe,
    .remove     = sgm37603a_remove,

};

static int __init sgm37603a_init(void)
{
    int ret=0;
    pr_info("sgm37603a_init\n");

    ret = i2c_add_driver(&sgm37603a_driver);
    pr_info("sgm37603a_init ret=%d\n",ret);
    if(ret){
        pr_err("fail to add sgm37603 driver\n");
        return ret;
    }

    return 0;
}

static void __exit sgm37603a_exit(void)
{
    pr_info("sgm37603a_exit\n");
    i2c_del_driver(&sgm37603a_driver);
}

MODULE_DESCRIPTION("LCD_BACKLIGHT davicom ic driver");
MODULE_LICENSE("GPL");

module_init(sgm37603a_init);
module_exit(sgm37603a_exit);
