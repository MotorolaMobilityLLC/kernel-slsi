/*
 * drivers/soc/samsung/exynos-sleepgpio.c
 *
 * The driver for setting exynos gpio configuration
 *
 * Copyright (C) 2018, Samsung Electronics.
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

//int is_set_pdn_conf;

#ifdef CONFIG_OF
static const struct of_device_id exynos_sleepgpio_dt_match[] = {
	{.compatible = "samsung,exynos-sleepgpio",
     .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_gpio_dt_match);
#endif

//#if 0
#ifdef CONFIG_PM_SLEEP
static int exynos_sleepgpio_suspend(struct device *dev)
{
	struct pinctrl *pinctrl;

	pinctrl = pinctrl_get_select(dev, PINCTRL_STATE_SLEEP);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: Do not have sleep pinctrl state\n", __func__);
		return IS_ERR(pinctrl);
	}

	dev_info(dev, "Set sleep gpio \n");

	return 0;
}

static int exynos_sleepgpio_resume(struct device *dev)
{
	struct pinctrl *pinctrl = pinctrl_get(dev);

	pinctrl = devm_pinctrl_get_select(dev, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pinctrl))
		pr_err("%s: Do not have default pinctrl state.\n", __func__);

	dev_info(dev, "Release sleep gpio \n");

	return 0;
}

const struct dev_pm_ops exynos_sleepgpio_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
			exynos_sleepgpio_suspend,
			exynos_sleepgpio_resume)
};
#endif

static int exynos_sleepgpio_probe(struct platform_device *pdev)
{
	pr_info("%s: Exynos sleepgpio driver probe is done.\n", __func__);
	return 0;
}

static int exynos_sleepgpio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver exynos_sleepgpio = {
	.driver = {
		.name 	= "exynos_sleepgpio",
		.owner 	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(exynos_sleepgpio_dt_match),
#endif
#ifdef CONFIG_PM_SLEEP
		.pm		= &exynos_sleepgpio_pm_ops,
#endif
	},
	.probe = exynos_sleepgpio_probe,
	.remove = exynos_sleepgpio_remove,
};

module_platform_driver(exynos_sleepgpio);

MODULE_AUTHOR("soomin.kim@samsung.com");
MODULE_DESCRIPTION("GPIO setting driver for exynos");
MODULE_LICENSE("GPL");
