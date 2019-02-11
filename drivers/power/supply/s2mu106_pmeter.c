/*
 * s2mu106_pmeter.c - S2MU106 Power Meter Driver
 *
 * Copyright (C) 2016 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/mfd/samsung/s2mu106.h>
#include <linux/power/s2mu106_pmeter.h>
#include <linux/version.h>


struct s2mu106_pmeter_data *g_pmeter;
int s2mu106_powermeter_get_vchg_voltage(void)
{
	struct s2mu106_pmeter_data *pmeter;
	u8 ret;
	u8 v_12 = 0, v_13 = 0;
	int chg_voltage = 0;

	pmeter = g_pmeter;

	s2mu106_read_reg(pmeter->i2c, 0x12, &v_12);
	s2mu106_read_reg(pmeter->i2c, 0x13, &v_13);

	pr_info("%s: S2MU106_POWERMETER 0x57: 0x%x 0x12:0x%x 0x13:0x%x\n", __func__, ret, v_12, v_13);
	chg_voltage = ((v_12 << 4) + (v_13 >> 4)) * 5;	//mV

	pr_info("%s: S2MU106_POWERMETER  chg_voltage: %d mV\n", __func__, chg_voltage);
	return chg_voltage;

}

int s2mu106_powermeter_get_vchg_current(void)
{
	struct s2mu106_pmeter_data *pmeter;
	u8 v_20 = 0, v_21 = 0;
	int chg_current = 0;

	pmeter = g_pmeter;

	s2mu106_read_reg(pmeter->i2c, 0x20, &v_20);
	s2mu106_read_reg(pmeter->i2c, 0x21, &v_21);

	chg_current = ((v_20 << 4) + (v_21 >> 4));	//mA

	pr_info("%s: S2MU106_POWERMETER  chg_current: %d mA\n", __func__, chg_current);
	return chg_current;

}

static const struct of_device_id s2mu106_pmeter_match_table[] = {
	{ .compatible = "samsung,s2mu106-pmeter",},
	{},
};

static int s2mu106_pm_enable(struct s2mu106_pmeter_data *pmeter)
{
	u8 data1;

	s2mu106_read_reg(pmeter->i2c, 0x5F, &data1);
	data1 |= 0x80;
	s2mu106_write_reg(pmeter->i2c, 0x5F, data1);
	return 0;
}

static int s2mu106_pmeter_probe(struct platform_device *pdev)
{
	struct s2mu106_dev *s2mu106 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu106_pmeter_data *pmeter;
	int ret = 0;

	pr_info("%s:[BATT] S2MU106 Power meter driver probe\n", __func__);
	pmeter = kzalloc(sizeof(*pmeter), GFP_KERNEL);
	if (!pmeter)
		return -ENOMEM;

	pmeter->dev = &pdev->dev;
	pmeter->i2c = s2mu106->muic; // share the i2c slave address with MUIC

	platform_set_drvdata(pdev, pmeter);

	g_pmeter = pmeter;
	pr_info("%s:[BATT] S2MU106 pmeter driver loaded OK\n", __func__);

	s2mu106_pm_enable(pmeter);

	return ret;
}

static int s2mu106_pmeter_remove(struct platform_device *pdev)
{
	struct s2mu106_pmeter_data *pmeter =
		platform_get_drvdata(pdev);

	kfree(pmeter);
	return 0;
}

#if defined CONFIG_PM
static int s2mu106_pmeter_suspend(struct device *dev)
{
	return 0;
}

static int s2mu106_pmeter_resume(struct device *dev)
{
	return 0;
}
#else
#define s2mu106_pmeter_suspend NULL
#define s2mu106_pmeter_resume NULL
#endif

static void s2mu106_pmeter_shutdown(struct device *dev)
{
	pr_info("%s: S2MU106 PowerMeter driver shutdown\n", __func__);
}

static SIMPLE_DEV_PM_OPS(s2mu106_pmeter_pm_ops, s2mu106_pmeter_suspend,
		s2mu106_pmeter_resume);

static struct platform_driver s2mu106_pmeter_driver = {
	.driver         = {
		.name   = "s2mu106-powermeter",
		.owner  = THIS_MODULE,
		.of_match_table = s2mu106_pmeter_match_table,
		.pm     = &s2mu106_pmeter_pm_ops,
		.shutdown   =   s2mu106_pmeter_shutdown,
	},
	.probe          = s2mu106_pmeter_probe,
	.remove     = s2mu106_pmeter_remove,
};

static int __init s2mu106_pmeter_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&s2mu106_pmeter_driver);

	return ret;
}
module_init(s2mu106_pmeter_init);

static void __exit s2mu106_pmeter_exit(void)
{
	platform_driver_unregister(&s2mu106_pmeter_driver);
}
module_exit(s2mu106_pmeter_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("PowerMeter driver for S2MU106");
