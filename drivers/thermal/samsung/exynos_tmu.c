/*
 * exynos_tmu.c - Samsung EXYNOS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2014 Samsung Electronics
 *  Bartlomiej Zolnierkiewicz <b.zolnierkie@samsung.com>
 *  Lukasz Majewski <l.majewski@samsung.com>
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>

#include "exynos_tmu.h"
#include "../thermal_core.h"

/* Exynos generic registers */
#define EXYNOS_TMU_REG_TRIMINFO		0x0
#define EXYNOS_TMU_REG_TRIMINFO1	0x4
#define EXYNOS_TMU_REG_CONTROL		0x20
#define EXYNOS_TMU_REG_STATUS		0x28
#define EXYNOS_TMU_REG_CURRENT_TEMP	0x40
#define EXYNOS_TMU_REG_INTEN		0x110
#define EXYNOS_TMU_REG_INTSTAT		0x74
#define EXYNOS_TMU_REG_INTCLEAR		0x78

#define EXYNOS_TMU_REF_VOLTAGE_SHIFT	24
#define EXYNOS_TMU_REF_VOLTAGE_MASK	0x1f
#define EXYNOS_TMU_BUF_SLOPE_SEL_MASK	0xf
#define EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT	8
#define EXYNOS_TMU_CORE_EN_SHIFT	0

#define EXYNOS_TMU_TRIP_MODE_SHIFT	13
#define EXYNOS_TMU_TRIP_MODE_MASK	0x7
#define EXYNOS_TMU_THERM_TRIP_EN_SHIFT	12

#define EXYNOS_TMU_INTEN_RISE0_SHIFT	0
#define EXYNOS_TMU_INTEN_FALL0_SHIFT	16

#define EXYNOS_EMUL_TIME	0x57F0
#define EXYNOS_EMUL_TIME_MASK	0xffff
#define EXYNOS_EMUL_TIME_SHIFT	16
#define EXYNOS_EMUL_DATA_SHIFT	7
#define EXYNOS_EMUL_DATA_MASK	0x1FF
#define EXYNOS_EMUL_ENABLE	0x1

#define EXYNOS_THD_TEMP_RISE7_6			0x50
#define EXYNOS_THD_TEMP_FALL7_6			0x60
#define EXYNOS_TMU_INTEN_RISE0_SHIFT		0
#define EXYNOS_TMU_INTEN_RISE1_SHIFT		1
#define EXYNOS_TMU_INTEN_RISE2_SHIFT		2
#define EXYNOS_TMU_INTEN_RISE3_SHIFT		3
#define EXYNOS_TMU_INTEN_RISE4_SHIFT		4
#define EXYNOS_TMU_INTEN_RISE5_SHIFT		5
#define EXYNOS_TMU_INTEN_RISE6_SHIFT		6
#define EXYNOS_TMU_INTEN_RISE7_SHIFT		7

#define EXYNOS_TMU_CALIB_SEL_SHIFT		(23)
#define EXYNOS_TMU_CALIB_SEL_MASK		(0x1)
#define EXYNOS_TMU_TEMP_MASK			(0x1ff)
#define EXYNOS_TMU_TRIMINFO_85_P0_SHIFT		(9)
#define EXYNOS_TRIMINFO_ONE_POINT_TRIMMING	(0)
#define EXYNOS_TRIMINFO_TWO_POINT_TRIMMING	(1)
#define EXYNOS_TMU_T_BUF_VREF_SEL_SHIFT		(18)
#define EXYNOS_TMU_T_BUF_VREF_SEL_MASK		(0x1F)
#define EXYNOS_TMU_T_BUF_SLOPE_SEL_SHIFT	(18)
#define EXYNOS_TMU_T_BUF_SLOPE_SEL_MASK		(0xF)

#define EXYNOS_TMU_REG_INTPEND			(0x118)
#define EXYNOS_TMU_REG_EMUL_CON			(0x160)

#define MCELSIUS	1000
/**
 * struct exynos_tmu_data : A structure to hold the private data of the TMU
	driver
 * @id: identifier of the one instance of the TMU controller.
 * @pdata: pointer to the tmu platform/configuration data
 * @base: base address of the single instance of the TMU controller.
 * @base_second: base address of the common registers of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @temp_error1: fused value of the first point trim.
 * @temp_error2: fused value of the second point trim.
 * @regulator: pointer to the TMU regulator structure.
 * @reg_conf: pointer to structure to register with core thermal.
 * @ntrip: number of supported trip points.
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 * @tmu_clear_irqs: SoC specific TMU interrupts clearing method
 */
struct exynos_tmu_data {
	int id;
	struct exynos_tmu_platform_data *pdata;
	void __iomem *base;
	int irq;
	enum soc_type soc;
	struct work_struct irq_work;
	struct mutex lock;
	u16 temp_error1, temp_error2;
	struct thermal_zone_device *tzd;
	unsigned int ntrip;
	struct thermal_cooling_device *cool_dev;

	int (*tmu_initialize)(struct platform_device *pdev);
	void (*tmu_control)(struct platform_device *pdev, bool on);
	int (*tmu_read)(struct exynos_tmu_data *data);
	void (*tmu_set_emulation)(struct exynos_tmu_data *data, int temp);
	void (*tmu_clear_irqs)(struct exynos_tmu_data *data);
};

static void exynos_report_trigger(struct exynos_tmu_data *p)
{
	char data[10], *envp[] = { data, NULL };
	struct thermal_zone_device *tz = p->tzd;
	int temp;
	unsigned int i;

	if (!tz) {
		pr_err("No thermal zone device defined\n");
		return;
	}

	thermal_zone_device_update(tz, THERMAL_EVENT_UNSPECIFIED);

	mutex_lock(&tz->lock);
	/* Find the level for which trip happened */
	for (i = 0; i < of_thermal_get_ntrips(tz); i++) {
		tz->ops->get_trip_temp(tz, i, &temp);
		if (tz->last_temperature < temp)
			break;
	}

	snprintf(data, sizeof(data), "%u", i);
	kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, envp);
	mutex_unlock(&tz->lock);
}

/*
 * TMU treats temperature as a mapped temperature code.
 * The temperature is converted differently depending on the calibration type.
 */
static int temp_to_code(struct exynos_tmu_data *data, u8 temp)
{
	struct exynos_tmu_platform_data *pdata = data->pdata;
	int temp_code;

	switch (pdata->cal_type) {
	case TYPE_TWO_POINT_TRIMMING:
		temp_code = (temp - pdata->first_point_trim) *
			(data->temp_error2 - data->temp_error1) /
			(pdata->second_point_trim - pdata->first_point_trim) +
			data->temp_error1;
		break;
	case TYPE_ONE_POINT_TRIMMING:
		temp_code = temp + data->temp_error1 - pdata->first_point_trim;
		break;
	default:
		temp_code = temp + pdata->default_temp_offset;
		break;
	}

	return temp_code;
}

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(struct exynos_tmu_data *data, u16 temp_code)
{
	struct exynos_tmu_platform_data *pdata = data->pdata;
	int temp;

	switch (pdata->cal_type) {
	case TYPE_TWO_POINT_TRIMMING:
		temp = (temp_code - data->temp_error1) *
			(pdata->second_point_trim - pdata->first_point_trim) /
			(data->temp_error2 - data->temp_error1) +
			pdata->first_point_trim;
		break;
	case TYPE_ONE_POINT_TRIMMING:
		temp = temp_code - data->temp_error1 + pdata->first_point_trim;
		break;
	default:
		temp = temp_code - pdata->default_temp_offset;
		break;
	}

	return temp;
}

static int exynos_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	int ret;

	if (of_thermal_get_ntrips(data->tzd) > data->ntrip) {
		dev_info(&pdev->dev,
			 "More trip points than supported by this TMU.\n");
		dev_info(&pdev->dev,
			 "%d trip points should be configured in polling mode.\n",
			 (of_thermal_get_ntrips(data->tzd) - data->ntrip));
	}

	mutex_lock(&data->lock);
	ret = data->tmu_initialize(pdev);
	mutex_unlock(&data->lock);

	return ret;
}

static u32 get_con_reg(struct exynos_tmu_data *data, u32 con)
{
	struct exynos_tmu_platform_data *pdata = data->pdata;

	con &= ~(EXYNOS_TMU_REF_VOLTAGE_MASK << EXYNOS_TMU_REF_VOLTAGE_SHIFT);
	con |= pdata->reference_voltage << EXYNOS_TMU_REF_VOLTAGE_SHIFT;

	con &= ~(EXYNOS_TMU_BUF_SLOPE_SEL_MASK << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);
	con |= (pdata->gain << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);

	if (pdata->noise_cancel_mode) {
		con &= ~(EXYNOS_TMU_TRIP_MODE_MASK << EXYNOS_TMU_TRIP_MODE_SHIFT);
		con |= (pdata->noise_cancel_mode << EXYNOS_TMU_TRIP_MODE_SHIFT);
	}

	return con;
}

static void exynos_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	mutex_lock(&data->lock);
	data->tmu_control(pdev, on);
	mutex_unlock(&data->lock);
}

static int exynos8890_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tz = data->tzd;
	struct exynos_tmu_platform_data *pdata = data->pdata;
	unsigned int rising_threshold = 0, falling_threshold = 0;
	int temp, temp_hist;
	unsigned int trim_info;
	unsigned int reg_off, bit_off;
	int threshold_code, i;

	/* Check tmu core ready status */
	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);

	/* Check thermal calibration type */
	pdata->cal_type = (trim_info >> EXYNOS_TMU_CALIB_SEL_SHIFT)
			& EXYNOS_TMU_CALIB_SEL_MASK;

	/* Check temp_error1 and error2 value */
	data->temp_error1 = trim_info & EXYNOS_TMU_TEMP_MASK;
	data->temp_error2 = (trim_info >> EXYNOS_TMU_TRIMINFO_85_P0_SHIFT)
				& EXYNOS_TMU_TEMP_MASK;

	if (!data->temp_error1)
		data->temp_error1 = pdata->efuse_value & EXYNOS_TMU_TEMP_MASK;
	if (!data->temp_error2)
		data->temp_error2 = (pdata->efuse_value >>
					EXYNOS_TMU_TRIMINFO_85_P0_SHIFT)
					& EXYNOS_TMU_TEMP_MASK;
	/* Write temperature code for rising and falling threshold */
	for (i = (of_thermal_get_ntrips(tz) - 1); i >= 0; i--) {
		/*
		 * On exynos7 there are 4 rising and 4 falling threshold
		 * registers (0x50-0x5c and 0x60-0x6c respectively). Each
		 * register holds the value of two threshold levels (at bit
		 * offsets 0 and 16). Based on the fact that there are atmost
		 * eight possible trigger levels, calculate the register and
		 * bit offsets where the threshold levels are to be written.
		 *
		 * e.g. EXYNOS_THD_TEMP_RISE7_6 (0x50)
		 * [24:16] - Threshold level 7
		 * [8:0] - Threshold level 6
		 * e.g. EXYNOS_THD_TEMP_RISE5_4 (0x54)
		 * [24:16] - Threshold level 5
		 * [8:0] - Threshold level 4
		 *
		 * and similarly for falling thresholds.
		 *
		 * Based on the above, calculate the register and bit offsets
		 * for rising/falling threshold levels and populate them.
		 */
		reg_off = ((7 - i) / 2) * 4;
		bit_off = ((8 - i) % 2);

		tz->ops->get_trip_temp(tz, i, &temp);
		temp /= MCELSIUS;

		tz->ops->get_trip_hyst(tz, i, &temp_hist);
		temp_hist = temp - (temp_hist / MCELSIUS);

		/* Set 9-bit temperature code for rising threshold levels */
		threshold_code = temp_to_code(data, temp);
		rising_threshold = readl(data->base +
			EXYNOS_THD_TEMP_RISE7_6 + reg_off);
		rising_threshold &= ~(EXYNOS_TMU_TEMP_MASK << (16 * bit_off));
		rising_threshold |= threshold_code << (16 * bit_off);
		writel(rising_threshold,
		       data->base + EXYNOS_THD_TEMP_RISE7_6 + reg_off);

		/* Set 9-bit temperature code for falling threshold levels */
		threshold_code = temp_to_code(data, temp_hist);
		falling_threshold &= ~(EXYNOS_TMU_TEMP_MASK << (16 * bit_off));
		falling_threshold |= threshold_code << (16 * bit_off);
		writel(falling_threshold,
		       data->base + EXYNOS_THD_TEMP_FALL7_6 + reg_off);
	}

	data->tmu_clear_irqs(data);

	return 0;
}

static void exynos8890_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tz = data->tzd;
	unsigned int con, interrupt_en, trim_info, trim_info1;
	unsigned int t_buf_vref_sel, t_buf_slope_sel;

	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);
	trim_info1 = readl(data->base + EXYNOS_TMU_REG_TRIMINFO1);

	/* Save fuse buf_vref_sel, calib_sel value to TRIMINFO and 1 register */
	t_buf_vref_sel = (trim_info >> EXYNOS_TMU_T_BUF_VREF_SEL_SHIFT)
				& (EXYNOS_TMU_T_BUF_VREF_SEL_MASK);
	t_buf_slope_sel = (trim_info1 >> EXYNOS_TMU_T_BUF_SLOPE_SEL_SHIFT)
				& (EXYNOS_TMU_T_BUF_SLOPE_SEL_MASK);

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on) {
		con |= (t_buf_vref_sel << EXYNOS_TMU_REF_VOLTAGE_SHIFT);
		con |= (t_buf_slope_sel << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);
		con |= (1 << EXYNOS_TMU_CORE_EN_SHIFT);
		con |= (1 << EXYNOS_TMU_THERM_TRIP_EN_SHIFT);
		interrupt_en =
			(of_thermal_is_trip_valid(tz, 7)
			<< EXYNOS_TMU_INTEN_RISE7_SHIFT) |
			(of_thermal_is_trip_valid(tz, 6)
			<< EXYNOS_TMU_INTEN_RISE6_SHIFT) |
			(of_thermal_is_trip_valid(tz, 5)
			<< EXYNOS_TMU_INTEN_RISE5_SHIFT) |
			(of_thermal_is_trip_valid(tz, 4)
			<< EXYNOS_TMU_INTEN_RISE4_SHIFT) |
			(of_thermal_is_trip_valid(tz, 3)
			<< EXYNOS_TMU_INTEN_RISE3_SHIFT) |
			(of_thermal_is_trip_valid(tz, 2)
			<< EXYNOS_TMU_INTEN_RISE2_SHIFT) |
			(of_thermal_is_trip_valid(tz, 1)
			<< EXYNOS_TMU_INTEN_RISE1_SHIFT) |
			(of_thermal_is_trip_valid(tz, 0)
			<< EXYNOS_TMU_INTEN_RISE0_SHIFT);

		interrupt_en |=
			interrupt_en << EXYNOS_TMU_INTEN_FALL0_SHIFT;
	} else {
		con &= ~(1 << EXYNOS_TMU_CORE_EN_SHIFT);
		con &= ~(1 << EXYNOS_TMU_THERM_TRIP_EN_SHIFT);
		interrupt_en = 0; /* Disable all interrupts */
	}

	writel(interrupt_en, data->base + EXYNOS_TMU_REG_INTEN);
	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

static int exynos_get_temp(void *p, int *temp)
{
	struct exynos_tmu_data *data = p;

	if (!data || !data->tmu_read)
		return -EINVAL;

	mutex_lock(&data->lock);

	*temp = code_to_temp(data, data->tmu_read(data)) * MCELSIUS;

	mutex_unlock(&data->lock);

	return 0;
}

static int exynos_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	struct exynos_tmu_data *data = p;
	struct thermal_zone_device *tz = data->tzd;
	int trip_temp, ret = 0;

	ret = tz->ops->get_trip_temp(tz, trip, &trip_temp);
	if (ret < 0)
		return ret;

	if (tz->temperature >= trip_temp)
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

#ifdef CONFIG_THERMAL_EMULATION
static u32 get_emul_con_reg(struct exynos_tmu_data *data, unsigned int val,
			    int temp)
{
	if (temp) {
		temp /= MCELSIUS;
		val &= ~(EXYNOS_EMUL_DATA_MASK <<
			EXYNOS_EMUL_DATA_SHIFT);
		val |= (temp_to_code(data, temp) <<
			EXYNOS_EMUL_DATA_SHIFT) |
			EXYNOS_EMUL_ENABLE;
	} else {
		val &= ~EXYNOS_EMUL_ENABLE;
	}

	return val;
}

static void exynos8890_tmu_set_emulation(struct exynos_tmu_data *data,
					 int temp)
{
	unsigned int val;
	u32 emul_con;

	emul_con = EXYNOS_TMU_REG_EMUL_CON;

	val = readl(data->base + emul_con);
	val = get_emul_con_reg(data, val, temp);
	writel(val, data->base + emul_con);
}

static int exynos_tmu_set_emulation(void *drv_data, int temp)
{
	struct exynos_tmu_data *data = drv_data;
	int ret = -EINVAL;

	if (temp && temp < MCELSIUS)
		goto out;

	mutex_lock(&data->lock);
	data->tmu_set_emulation(data, temp);
	mutex_unlock(&data->lock);
	return 0;
out:
	return ret;
}
#else
#define exynos8890_tmu_set_emulation NULL
static int exynos_tmu_set_emulation(void *drv_data, int temp)
	{ return -EINVAL; }
#endif /* CONFIG_THERMAL_EMULATION */

static int exynos8890_tmu_read(struct exynos_tmu_data *data)
{
	return readw(data->base + EXYNOS_TMU_REG_CURRENT_TEMP) &
		EXYNOS_TMU_TEMP_MASK;
}

static void exynos_tmu_work(struct work_struct *work)
{
	struct exynos_tmu_data *data = container_of(work,
			struct exynos_tmu_data, irq_work);

	exynos_report_trigger(data);
	mutex_lock(&data->lock);

	/* TODO: take action based on particular interrupt */
	data->tmu_clear_irqs(data);

	mutex_unlock(&data->lock);
	enable_irq(data->irq);
}

static void exynos8890_tmu_clear_irqs(struct exynos_tmu_data *data)
{
	unsigned int val_irq;


	val_irq = readl(data->base + EXYNOS_TMU_REG_INTPEND);
	writel(val_irq, data->base + EXYNOS_TMU_REG_INTPEND);
}

static irqreturn_t exynos_tmu_irq(int irq, void *id)
{
	struct exynos_tmu_data *data = id;

	disable_irq_nosync(irq);
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static const struct of_device_id exynos_tmu_match[] = {
	{ .compatible = "samsung,exynos8890-tmu", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, exynos_tmu_match);

static int exynos_of_get_soc_type(struct device_node *np)
{
	if (of_device_is_compatible(np, "samsung,exynos8890-tmu"))
		return SOC_ARCH_EXYNOS8890;

	return -EINVAL;
}

static int exynos_of_sensor_conf(struct device_node *np,
				 struct exynos_tmu_platform_data *pdata)
{
	u32 value;
	int ret;

	of_node_get(np);

	ret = of_property_read_u32(np, "samsung,tmu_gain", &value);
	pdata->gain = (u8)value;
	of_property_read_u32(np, "samsung,tmu_reference_voltage", &value);
	pdata->reference_voltage = (u8)value;
	of_property_read_u32(np, "samsung,tmu_noise_cancel_mode", &value);
	pdata->noise_cancel_mode = (u8)value;

	of_property_read_u32(np, "samsung,tmu_efuse_value",
			     &pdata->efuse_value);

	of_property_read_u32(np, "samsung,tmu_first_point_trim", &value);
	pdata->first_point_trim = (u8)value;
	of_property_read_u32(np, "samsung,tmu_second_point_trim", &value);
	pdata->second_point_trim = (u8)value;
	of_property_read_u32(np, "samsung,tmu_default_temp_offset", &value);
	pdata->default_temp_offset = (u8)value;

	of_property_read_u32(np, "samsung,tmu_cal_type", &pdata->cal_type);

	of_node_put(np);
	return 0;
}

static int exynos_map_dt_data(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_tmu_platform_data *pdata;
	struct resource res;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->id = of_alias_get_id(pdev->dev.of_node, "tmuctrl");
	if (data->id < 0)
		data->id = 0;

	data->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (data->irq <= 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENODEV;
	}

	if (of_address_to_resource(pdev->dev.of_node, 0, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 0\n");
		return -ENODEV;
	}

	data->base = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (!data->base) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -EADDRNOTAVAIL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct exynos_tmu_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	exynos_of_sensor_conf(pdev->dev.of_node, pdata);
	data->pdata = pdata;
	data->soc = exynos_of_get_soc_type(pdev->dev.of_node);

	switch (data->soc) {
	case SOC_ARCH_EXYNOS8890:
		data->tmu_initialize = exynos8890_tmu_initialize;
		data->tmu_control = exynos8890_tmu_control;
		data->tmu_read = exynos8890_tmu_read;
		data->tmu_set_emulation = exynos8890_tmu_set_emulation;
		data->tmu_clear_irqs = exynos8890_tmu_clear_irqs;
		break;
	default:
		dev_err(&pdev->dev, "Platform not supported\n");
		return -EINVAL;
	}

	return 0;
}

static const struct thermal_zone_of_device_ops exynos_sensor_ops = {
	.get_temp = exynos_get_temp,
	.set_emul_temp = exynos_tmu_set_emulation,
	.get_trend = exynos_get_trend,
};

static int exynos_cpufreq_cooling_register(struct exynos_tmu_data *data)
{
	struct device_node *np, *child = NULL, *gchild, *ggchild;
	struct device_node *cool_np;
	struct of_phandle_args cooling_spec;
	struct cpumask mask_val;
	int cpu, ret, i;

	np = of_find_node_by_name(NULL, "thermal-zones");
	if (!np)
		return -ENODEV;

	/* Regist cpufreq cooling device */
	for (i = 0; i <= data->id; i++) {
		child = of_get_next_child(np, child);
		if (i == data->id)
			break;
	}
	gchild = of_get_child_by_name(child, "cooling-maps");
	ggchild = of_get_next_child(gchild, NULL);
	ret = of_parse_phandle_with_args(ggchild, "cooling-device", "#cooling-cells",
					 0, &cooling_spec);
	if (ret < 0) {
		pr_err("exynos_tmu do not get cooling spec \n");
	}
	cool_np = cooling_spec.np;

	for_each_possible_cpu(cpu) {
		if (cpu_topology[cpu].cluster_id == data->id) {
			cpumask_copy(&mask_val, topology_core_cpumask(cpu));
		}
	}

	data->cool_dev = of_cpufreq_cooling_register(cool_np, &mask_val);

	return ret;
}

static int exynos_tmu_probe(struct platform_device *pdev)
{
	struct exynos_tmu_data *data;
	int ret;

	if (!cpufreq_frequency_get_table(0))
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(struct exynos_tmu_data),
					GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	ret = exynos_map_dt_data(pdev);
	if (ret)
		goto err_sensor;

	ret = exynos_cpufreq_cooling_register(data);
	if (ret) {
		dev_err(&pdev->dev, "Failed cooling register \n");
		goto err_sensor;
	}

	INIT_WORK(&data->irq_work, exynos_tmu_work);

	/*
	 * data->tzd must be registered before calling exynos_tmu_initialize(),
	 * requesting irq and calling exynos_tmu_control().
	 */
	data->tzd = thermal_zone_of_sensor_register(&pdev->dev, 0, data,
						    &exynos_sensor_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		dev_err(&pdev->dev, "Failed to register sensor: %d\n", ret);
		goto err_sensor;
	}

	ret = exynos_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		goto err_thermal;
	}

	ret = devm_request_irq(&pdev->dev, data->irq, exynos_tmu_irq,
		IRQF_TRIGGER_RISING | IRQF_SHARED, dev_name(&pdev->dev), data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", data->irq);
		goto err_thermal;
	}

	exynos_tmu_control(pdev, true);
	return 0;

err_thermal:
	thermal_zone_of_sensor_unregister(&pdev->dev, data->tzd);
err_sensor:
	return ret;
}

static int exynos_tmu_remove(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tzd = data->tzd;

	thermal_zone_of_sensor_unregister(&pdev->dev, tzd);
	exynos_tmu_control(pdev, false);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_tmu_suspend(struct device *dev)
{
	exynos_tmu_control(to_platform_device(dev), false);

	return 0;
}

static int exynos_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	exynos_tmu_initialize(pdev);
	exynos_tmu_control(pdev, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_tmu_pm,
			 exynos_tmu_suspend, exynos_tmu_resume);
#define EXYNOS_TMU_PM	(&exynos_tmu_pm)
#else
#define EXYNOS_TMU_PM	NULL
#endif

static struct platform_driver exynos_tmu_driver = {
	.driver = {
		.name   = "exynos-tmu",
		.pm     = EXYNOS_TMU_PM,
		.of_match_table = exynos_tmu_match,
	},
	.probe = exynos_tmu_probe,
	.remove	= exynos_tmu_remove,
};

module_platform_driver(exynos_tmu_driver);

MODULE_DESCRIPTION("EXYNOS TMU Driver");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-tmu");
