/*
 * s2mu00x_battery.c - Example battery driver for S2MU00x series
 *
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
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
 */

#include <linux/version.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/power/s2mu00x_battery.h>
#include <linux/power/s2mu106_pmeter.h>
#include <linux/alarmtimer.h>
#include <linux/reboot.h>
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/s2mu004-muic-notifier.h>
#include <linux/muic/muic.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#if defined(CONFIG_IFCONN_NOTIFIER)
#include <linux/ifconn/ifconn_notifier.h>
#include <linux/ifconn/ifconn_manager.h>
#include <linux/muic/muic.h>
#endif

#define FAKE_BAT_LEVEL	50
#define DEFAULT_ALARM_INTERVAL	30
#define SLEEP_ALARM_INTERVAL	60

static char *bat_status_str[] = {
	"Unknown",
	"Charging",
	"Discharging",
	"Not-charging",
	"Full"
};

static char *health_str[] = {
	"Unknown",
	"Good",
	"Overheat",
	"Dead",
	"OverVoltage",
	"UnspecFailure",
	"Cold",
	"WatchdogTimerExpire",
	"SafetyTimerExpire",
	"UnderVoltage",
};

static enum power_supply_property s2mu00x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_TEMP,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_CALIBRATE,
	POWER_SUPPLY_PROP_SOH,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_RATE,
};

static enum power_supply_property s2mu00x_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

//moto
struct pchg_current_map {
	int requested;
	int primary;
	int secondary;
};

enum stepchg_state {
	STEP_MAX,
	STEP_ONE,
	STEP_TAPER,
	STEP_FULL,
	STEP_NONE = 0xFF,
};

enum charging_limit_modes {
	CHARGING_LIMIT_OFF,
	CHARGING_LIMIT_RUN,
	CHARGING_LIMIT_UNKNOWN,
};

static struct s2mu00x_battery_info *the_chip;
static char *smb_health_text[] = {
	"Unknown", "Good", "Overheat", "Dead", "Over voltage",
	"Unspecified failure", "Cold", "Watchdog timer expire",
	"Safety timer expire", "Under voltage", "Warm", "Cool", "Hot", "Slightly Cool"
};

static int smbchg_debug_mask = 0xff;
enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
};

#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

typedef struct s2mu00x_battery_platform_data {
	s2mu00x_charging_current_t *charging_current;
	char *charger_name;
#if defined(CONFIG_SMALL_CHARGER)
	char *smallcharger_name;
#endif
	char *fuelgauge_name;

	int max_input_current;
	int max_charging_current;

#if defined(CONFIG_SMALL_CHARGER)
	int small_input_current;
	int small_charging_current;
#endif

#if defined(CONFIG_USE_CCIC)
	int pdo_max_input_vol;
	int pdo_max_chg_power;
#endif

	int temp_high;
	int temp_high_recovery;
	int temp_low;
	int temp_low_recovery;

	int chg_float_voltage;

	/* full check */
	unsigned int full_check_count;
	unsigned int chg_recharge_vcell;
	unsigned int chg_full_vcell;

	/* Initial maximum raw SOC */
	unsigned int max_rawsoc;
	unsigned int max_rawsoc_offset;

	/* battery */
	char *vendor;
	int technology;
	int battery_type;
	void *battery_data;
} s2mu00x_battery_platform_data_t;

struct s2mu00x_battery_info {
	struct device *dev;
	s2mu00x_battery_platform_data_t *pdata;

	struct power_supply *psy_battery;
	struct power_supply_desc psy_battery_desc;
	struct power_supply *psy_usb;
	struct power_supply_desc psy_usb_desc;
	struct power_supply *psy_ac;
	struct power_supply_desc psy_ac_desc;

	struct mutex iolock;
	struct mutex ifconn_lock;

	struct wake_lock monitor_wake_lock;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
	struct delayed_work soc_control;
	struct wake_lock vbus_wake_lock;

	struct alarm monitor_alarm;
	unsigned int monitor_alarm_interval;

	int input_current;
	int max_input_current;
	int charging_current;
	int max_charging_current;

#if defined(CONFIG_USE_CCIC)
#if defined(CONFIG_USE_PDO_SELECT)
	struct delayed_work select_pdo_work;
#endif
	int pdo_max_input_vol;
	int pdo_max_chg_power;

	int pdo_sel_num;
	int pdo_sel_vol;
	int pdo_sel_cur;

	int pd_input_current;
	bool pd_attach;
	bool rp_attach;
	int rp_input_current;
	int rp_charging_current;
#endif

	int topoff_current;
	int cable_type;
	unsigned int charging_mode;

#if defined(CONFIG_IFCONN_NOTIFIER)
	struct notifier_block ifconn_nb;
#elif defined(CONFIG_MUIC_NOTIFIER)
	struct notifier_block batt_nb;
#endif

	int full_check_cnt;

	/* charging */
	bool is_recharging;

	bool is_factory;	/* factory image support mode */

	bool battery_valid;
	int status;
	int health;

	int voltage_now;
	int voltage_avg;
	int voltage_ocv;

	unsigned int capacity;
	unsigned int max_rawsoc;
	unsigned int max_rawsoc_offset;

	int soh;	/* State of Health (%) */

	int current_now;	/* current (mA) */
	int current_avg;	/* average current (mA) */
	int current_max;	/* input current limit (mA) */
	int current_chg;	/* charge current limit (mA) */

#if defined(CONFIG_SMALL_CHARGER)
	int small_input;	/* input current limit (mA) */
	int small_chg;	/* charge current limit (mA) */
	int small_input_flag;
#endif

#if defined(CONFIG_MUIC_NOTIFIER)
	struct notifier_block cable_check;
#endif

	/* temperature check */
	int temperature;    /* battery temperature(0.1 Celsius)*/
	int temp_high;
	int temp_high_recovery;
	int temp_low;
	int temp_low_recovery;
	int vchg_voltage;
	int vchg_current;
	int charge_temp;

	int thermal_enable;
	int thermal_fast_charge_percentage;

	//moto
	struct wake_lock heartbeat_wake_lock;
	bool				test_mode;
	int				test_mode_soc;
	int				test_mode_temp;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				dc_max_current_ma;
	int				usb_target_current_ma;
	int				target_fastchg_current_ma;
	int				allowed_fastchg_current_ma;
	bool				update_allowed_fastchg_current_ma;
	int				fastchg_current_ma;
	unsigned int			pchg_current_map_len;
	struct pchg_current_map        *pchg_current_map_data;
	int				vfloat_mv;
	int				vfloat_parallel_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				jeita_temp_hard_limit;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				battery_unknown;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				enable_hvdcp_9v;
	u8				original_usbin_allowance;

	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				sw_esr_pulse_en;
	bool				safety_timer_en;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	u32				wa_flags;

	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			chg_thermal_levels;
	unsigned int			chg_therm_lvl_sel;
	unsigned int			*chg_thermal_mitigation;

	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	unsigned int			dc_thermal_levels;
	unsigned int			dc_therm_lvl_sel;
	unsigned int			*dc_thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				safety_timeout_irq;
#ifdef QCOM_BASE
	int				power_ok_irq;
#endif
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				chg_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*charger_psy;
	struct power_supply		*otg_psy;
	bool				psy_registered;

	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	spinlock_t			sec_access_lock;
	struct mutex			current_change_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			usb_set_present_lock;
	struct mutex			dc_set_present_lock;
	struct mutex			battchg_disabled_lock;
	struct mutex			usb_en_lock;
	struct mutex			dc_en_lock;
	struct mutex			fcc_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	bool				factory_mode;
	bool				factory_cable;
	struct delayed_work		heartbeat_work;
	struct mutex			check_temp_lock;
	bool				enable_charging_limit;
	bool				is_factory_image;
	enum charging_limit_modes	charging_limit_modes;
	int				upper_limit_capacity;
	int				lower_limit_capacity;
	int				temp_state;
	int				hotspot_temp;
	int				hotspot_thrs_c;
	int				hot_temp_c;
	int				cold_temp_c;
	int				warm_temp_c;
	int				cool_temp_c;
	int				slightly_cool_temp_c;
	int				ext_high_temp;
	int				ext_temp_volt_mv;
	int				stepchg_voltage_mv;
	int				stepchg_current_ma;
	int				stepchg_taper_ma;
	int				stepchg_iterm_ma;
	int				stepchg_max_voltage_mv;
	int				stepchg_max_current_ma;
	enum stepchg_state		stepchg_state;
	unsigned int			stepchg_state_holdoff;
	struct wakeup_source		smbchg_wake_source;
	struct delayed_work		usb_insertion_work;
	int				charger_rate;
	bool				usbid_disabled;
	bool				usbid_gpio_enabled;
	int				demo_mode;
	bool				batt_therm_wa;
	struct notifier_block		smb_reboot;
	int				aicl_wait_retries;
	bool				hvdcp_det_done;
	int				afvc_mv;
	enum power_supply_type          supply_type;
	bool				enabled_weak_charger_check;
	bool				adapter_vbus_collapse_flag;
	int				weak_charger_valid_cnt;
	bool				is_weak_charger;
	ktime_t			weak_charger_valid_cnt_ktmr;
	ktime_t			adapter_vbus_collapse_ktmr;
	int				temp_good_current_ma;
	int				temp_warm_current_ma;
	int				temp_cool_current_ma;
	int				temp_slightly_cool_current_ma;
	int				temp_allowed_fastchg_current_ma;
	bool				enable_factory_wa;
	int                         max_chrg_temp;
	bool			charging_disabled;
};

static int is_charging_mode = S2MU00X_NOR_MODE;
bool is_cable_present(struct s2mu00x_battery_info *chip);
static void smbchg_stay_awake(struct s2mu00x_battery_info *chip);
static void smbchg_relax(struct s2mu00x_battery_info *chip);
bool is_dc_present(struct s2mu00x_battery_info *chip);
bool is_usb_present(struct s2mu00x_battery_info *chip);
static int smbchg_usb_en(struct s2mu00x_battery_info *chip, bool enable);
void factory_usb_shutdown(struct s2mu00x_battery_info *chip);
static int set_property_on_charger(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int val);
static int set_property_on_otg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int val);
static int get_property_from_otg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int *val);
static int get_property_from_charger(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int *val);
#if 0
static int set_property_on_fg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int val);
#endif

static int get_property_from_fg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int *val);
static int smbchg_charging_en(struct s2mu00x_battery_info *chip, bool en);
static int smbchg_chg_system_temp_level_set(struct s2mu00x_battery_info *chip, int lvl_sel);
static int get_prop_batt_current_now(struct s2mu00x_battery_info *chip);
static void set_max_allowed_current_ma(struct s2mu00x_battery_info *chip,int current_ma);
static void smbchg_set_temp_chgpath(struct s2mu00x_battery_info *chip, int prev_temp);
static char *s2mu00x_supplied_to[] = {
	"s2mu00x-battery",
};
#if defined(CONFIG_CHARGER_S2MU106)
static void get_charging_current(struct s2mu00x_battery_info *battery,
		int *input_current, int *charging_current)
{
	int max_input_current = battery->max_input_current;
	int max_charging_current = battery->max_charging_current;

	if (*input_current > max_input_current) {
		*input_current = max_input_current;
		pr_info("%s: limit input current. (%d)\n", __func__, *input_current);
	}
	if (*charging_current > max_charging_current) {
		*charging_current = max_charging_current;
		pr_info("%s: limit charging current. (%d)\n", __func__, *charging_current);
	}
}

static int set_charging_current(struct s2mu00x_battery_info *battery)
{
	union power_supply_propval value;
	int input_current =
			battery->pdata->charging_current[battery->cable_type].input_current_limit,
		charging_current =
			battery->pdata->charging_current[battery->cable_type].fast_charging_current,
		topoff_current =
			battery->pdata->charging_current[battery->cable_type].full_check_current;
	struct power_supply *psy;
	int ret = 0;

	pr_info("%s: cable_type(%d), current(input:%d, fast:%d,term: %d)\n", __func__,
			battery->cable_type, input_current, charging_current, topoff_current);
	mutex_lock(&battery->iolock);

	/*Limit input & charging current according to the max current*/
	if (battery->cable_type == POWER_SUPPLY_TYPE_PREPARE_TA ||
		battery->cable_type == POWER_SUPPLY_TYPE_USB_PD) {
#if defined(CONFIG_USE_CCIC)
		pr_info("%s, %d, %d\n", __func__, input_current, battery->pd_input_current);
		input_current = battery->pd_input_current;
#endif

		if (input_current >= 1500)
			input_current = input_current - 50;
#if defined(CONFIG_SMALL_CHARGER)
		if (input_current > 2000) {
			battery->small_input_flag = input_current - 2000;
			input_current = 2000;
		}
#endif
	} else {
		if (battery->rp_attach &&
				!(battery->cable_type == POWER_SUPPLY_TYPE_BATTERY ||
					battery->cable_type == POWER_SUPPLY_TYPE_UNKNOWN ||
					battery->cable_type == POWER_SUPPLY_TYPE_OTG)) {
			input_current = battery->rp_input_current > input_current?
				battery->rp_input_current:input_current;
			charging_current = battery->rp_charging_current > charging_current?
				battery->rp_charging_current:charging_current;
			pr_info("%s: Rp attached! use input: %d, chg: %d\n",
					__func__, input_current, charging_current);
		}
		get_charging_current(battery, &input_current, &charging_current);
	}


	if(battery->thermal_enable == 1)
	{
		charging_current = charging_current * battery->thermal_fast_charge_percentage / 100;
		pr_info("%s: cable_type(%d), charging current: %d (by thermal_enable: %d %d \n", __func__,
			battery->cable_type, charging_current, battery->thermal_enable, battery->thermal_fast_charge_percentage);
	}
	/* set input current limit */
	if (battery->input_current != input_current) {
		value.intval = input_current;

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy) {
			ret = -EINVAL;
			goto out;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		battery->input_current = input_current;
	}
	/* set fast charging current */
	if (battery->charging_current != charging_current) {
		value.intval = charging_current;

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy) {
			ret = -EINVAL;
			goto out;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		battery->charging_current = charging_current;
	}
	/* set topoff current */
	if (battery->topoff_current != topoff_current) {
		value.intval = topoff_current;

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy) {
			ret = -EINVAL;
			goto out;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_FULL, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		battery->topoff_current = topoff_current;
	}
#if defined(CONFIG_SMALL_CHARGER)
	if (battery->cable_type == POWER_SUPPLY_TYPE_PREPARE_TA ||
		battery->cable_type == POWER_SUPPLY_TYPE_USB_PD) {

		if (battery->small_input_flag == 0) {
			ret = 0;
			goto out;
		}

		value.intval = battery->small_input_flag;
		psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
		if (!psy) {
			ret = -EINVAL;
			goto out;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		value.intval = battery->pdata->small_charging_current;
		psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
		if (!psy) {
			ret = -EINVAL;
			goto out;
		}
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);
	}
#endif
out:
	mutex_unlock(&battery->iolock);
	return ret;
}

/*
 * set_charger_mode(): charger_mode must have one of following values.
 * 1. S2MU00X_BAT_CHG_MODE_CHARGING
 *	Charger on.
 *	Supply power to system & battery both.
 * 2. S2MU00X_BAT_CHG_MODE_CHARGING_OFF
 *	Buck mode. Stop battery charging.
 *	But charger supplies system power.
 * 3. S2MU00X_BAT_CHG_MODE_BUCK_OFF
 *	All off. Charger is completely off.
 *	Do not supply power to battery & system both.
 */

static int set_charger_mode(
		struct s2mu00x_battery_info *battery,
		int charger_mode)
{
	union power_supply_propval val;
	struct power_supply *psy;
		int ret;

	val.intval = charger_mode;

	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

#if defined(CONFIG_SMALL_CHARGER)
	if (charger_mode == S2MU00X_BAT_CHG_MODE_CHARGING &&
			(battery->cable_type == POWER_SUPPLY_TYPE_PREPARE_TA ||
			 battery->cable_type == POWER_SUPPLY_TYPE_USB_PD)) {

		if (battery->small_input_flag == 0)
			return 0;

		psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);
	} else if (charger_mode != S2MU00X_BAT_CHG_MODE_CHARGING) {
		battery->small_input_flag = 0;
		psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);
	}
#endif

	return 0;
}

#endif
static int set_battery_status(struct s2mu00x_battery_info *battery,
		int status)
{
	union power_supply_propval value;
#if defined(CONFIG_CHARGER_S2MU106)
	struct power_supply *psy;
	int ret;
#endif
	pr_info("%s: current status = %d, new status = %d\n", __func__, battery->status, status);
#if 0
	if (battery->status == status)
		return 0;
#endif

#if defined(CONFIG_CHARGER_S2MU106)
	switch (status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		/* notify charger cable type */
		value.intval = battery->cable_type;

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

#if defined(CONFIG_SMALL_CHARGER)
		if (battery->cable_type == POWER_SUPPLY_TYPE_PREPARE_TA ||
			battery->cable_type == POWER_SUPPLY_TYPE_USB_PD) {
			psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
			if (!psy)
				return -EINVAL;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);
		}
#endif

		/* charger on */
		set_charger_mode(battery, S2MU00X_BAT_CHG_MODE_CHARGING);
		set_charging_current(battery);
		break;

	case POWER_SUPPLY_STATUS_DISCHARGING:
		set_charging_current(battery);

		/* notify charger cable type */
		value.intval = battery->cable_type;

#if defined(CONFIG_SMALL_CHARGER)
		if (battery->cable_type == POWER_SUPPLY_TYPE_PREPARE_TA ||
			battery->cable_type == POWER_SUPPLY_TYPE_USB_PD) {
			psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
			if (!psy)
				return -EINVAL;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);
		}
#endif

		psy = power_supply_get_by_name(battery->pdata->charger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		set_charger_mode(battery, S2MU00X_BAT_CHG_MODE_CHARGING_OFF);
		break;

	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		set_charger_mode(battery, S2MU00X_BAT_CHG_MODE_BUCK_OFF);

		/* to recover charger configuration when heath is recovered */
		battery->input_current = 0;
		battery->charging_current = 0;
		battery->topoff_current = 0;
#if defined(CONFIG_SMALL_CHARGER)
		battery->small_input_flag = 0;
#endif
		break;

	case POWER_SUPPLY_STATUS_FULL:
		set_charger_mode(battery, S2MU00X_BAT_CHG_MODE_CHARGING_OFF);
		break;
	}
#endif
	/* battery status update */
	battery->status = status;
	value.intval = battery->status;
#if defined(CONFIG_CHARGER_S2MU106)
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
#endif
	return 0;
}

static void set_bat_status_by_cable(struct s2mu00x_battery_info *battery)
{
	if (battery->is_factory){
		pr_info("%s: factory image support mode. Skip!\n", __func__);
		return;
	}

	if (battery->charging_disabled) {
		pr_info("%s: charging disabled. Skip!\n", __func__);
		return;
	}

#if defined(CONFIG_CHARGER_S2MU106)
	if (battery->cable_type == POWER_SUPPLY_TYPE_BATTERY ||
		battery->cable_type == POWER_SUPPLY_TYPE_UNKNOWN ||
		battery->cable_type == POWER_SUPPLY_TYPE_OTG || battery->factory_mode) {
		battery->is_recharging = false;
#if defined(CONFIG_USE_CCIC)
		battery->pdo_sel_num = 0;
		battery->pdo_sel_vol = 0;
		battery->pdo_sel_cur = 0;
#endif
		set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
		return;
	}
	if (battery->status != POWER_SUPPLY_STATUS_FULL) {
		set_battery_status(battery, POWER_SUPPLY_STATUS_CHARGING);
		return;
	}

	dev_info(battery->dev, "%s: abnormal cable_type or status", __func__);
#endif
}

#define WEAK_CHRG_THRSH 450
static void get_prop_charge_rate(struct s2mu00x_battery_info *battery)
{
	int prev_chg_rate = battery->charger_rate;
	char *charge_rate[] = {
		"None", "Normal", "Weak", "Turbo"
	};

	if (!is_usb_present(battery)) {
		battery->charger_rate = POWER_SUPPLY_CHARGE_RATE_NONE;
		return;
	}

	if(battery->cable_type == POWER_SUPPLY_TYPE_HV_MAINS ||
		battery->cable_type == POWER_SUPPLY_TYPE_USB_PD ||
		battery->cable_type == POWER_SUPPLY_TYPE_PREPARE_TA )
		battery->charger_rate = POWER_SUPPLY_CHARGE_RATE_TURBO;
	else
		battery->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;

	if(prev_chg_rate != battery->charger_rate)
		printk(KERN_ERR "%s,charge_rate:%s\n",__func__,charge_rate[battery->charger_rate]);
}

static int s2mu00x_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct s2mu00x_battery_info *battery =  power_supply_get_drvdata(psy);
	int ret = 0;
	union power_supply_propval value;

	dev_dbg(battery->dev, "prop: %d\n", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battery->health;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battery->cable_type;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery->battery_valid;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!battery->battery_valid)
			val->intval = FAKE_BAT_LEVEL;
		else
			val->intval = battery->voltage_now * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = battery->voltage_avg * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (battery->test_mode && !(battery->test_mode_temp < -350)
		    && !(battery->test_mode_temp > 1250))
			val->intval = battery->test_mode_temp;
		else
			val->intval = battery->temperature;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TEMP:
		val->intval = battery->charge_temp;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = battery->charging_mode;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!battery->battery_valid)
			val->intval = FAKE_BAT_LEVEL;
		else {
			if (battery->status == POWER_SUPPLY_STATUS_FULL)
				val->intval = 100;
			else
				val->intval = battery->capacity;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = battery->current_avg;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/*Get fuelgauge psy*/
		psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_COUNTER, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		val->intval = value.intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/*Get fuelgauge psy*/
		psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		val->intval = value.intval;
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
		val->intval = battery->is_factory;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		val->intval = battery->vchg_voltage;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		/* Make charge rate and USB input throttling mutually
		   exclusive for now */
		if (battery->chg_thermal_mitigation)
			val->intval = battery->chg_therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		/* Make charge rate and USB input throttling mutually
		   exclusive for now */
		if (battery->chg_thermal_mitigation)
			val->intval = battery->chg_thermal_levels;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		get_property_from_charger(battery, POWER_SUPPLY_PROP_CHARGING_ENABLED, &ret);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		get_property_from_charger(battery, POWER_SUPPLY_PROP_CHARGE_TYPE, &ret);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(battery);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 3500;
		break;
	case POWER_SUPPLY_PROP_SOH:
		val->intval = battery->soh;
		break;
	case POWER_SUPPLY_PROP_CHARGE_RATE:
		get_prop_charge_rate(battery);
		val->intval = battery->charger_rate;
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static int s2mu00x_battery_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct s2mu00x_battery_info *battery = power_supply_get_drvdata(psy);
	int ret = 0;
	union power_supply_propval value;
	struct power_supply *psy_dest;


	dev_dbg(battery->dev, "prop: %d\n", psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		set_battery_status(battery, val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		battery->health = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		battery->cable_type = val->intval;
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
		if (val->intval == S2MU00X_BAT_FAC_MODE_VBAT) {
			battery->is_factory = true;
			pr_info("%s: VBat factory image support mode\n", __func__);

			value.intval = S2MU00X_BAT_FAC_MODE_VBAT;
			psy_dest = power_supply_get_by_name(battery->pdata->charger_name);
			if (!psy_dest)
				return -EINVAL;
			ret = power_supply_set_property(psy_dest, POWER_SUPPLY_PROP_CALIBRATE, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);
		} else if (val->intval == S2MU00X_BAT_FAC_MODE_VBUS) {
			battery->is_factory = true;
			pr_info("%s: VBUS charging factory image support mode\n", __func__);

			value.intval = S2MU00X_BAT_FAC_MODE_VBUS;
			psy_dest = power_supply_get_by_name(battery->pdata->charger_name);
			if (!psy_dest)
				return -EINVAL;
			ret = power_supply_set_property(psy_dest, POWER_SUPPLY_PROP_CALIBRATE, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);
		} else {
			battery->is_factory = false;
			pr_info("%s: disable factory image support mode\n", __func__);

			/* Reset current setting for recovering */
			battery->input_current = 0;
			battery->charging_current = 0;
			battery->topoff_current = 0;

			set_bat_status_by_cable(battery);
		}
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		/* Make charge rate and USB input throttling mutually
		   exclusive for now */
		if (battery->chg_thermal_mitigation)
			smbchg_chg_system_temp_level_set(battery, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		pr_info("%s: POWER_SUPPLY_PROP_CHARGING_ENABLED, val->intval = %d.\n", __func__, val->intval);
		battery->charging_disabled = !val->intval;
		smbchg_usb_en(battery, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (battery->test_mode)
			battery->test_mode_soc = val->intval;
		power_supply_changed(battery->psy_battery);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (battery->test_mode)
			battery->test_mode_temp = val->intval;
		cancel_delayed_work(&battery->heartbeat_work);
		schedule_delayed_work(&battery->heartbeat_work,
					msecs_to_jiffies(0));
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int s2mu00x_battery_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;
	switch (psp) {
		case POWER_SUPPLY_PROP_CALIBRATE:
			ret = 1;
			break;
		default:
		ret = 0;
	}

	return ret;
}

static int s2mu00x_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct s2mu00x_battery_info *battery =  power_supply_get_drvdata(psy);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the USB charger is connected */
	switch (battery->cable_type) {
	case POWER_SUPPLY_TYPE_USB:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_ACA:
		val->intval = 1;
		break;
	default:
		val->intval = 0;
		break;
	}

	return 0;
}

/*
 * AC charger operations
 */
static int s2mu00x_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct s2mu00x_battery_info *battery =  power_supply_get_drvdata(psy);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	switch (battery->cable_type) {
	case POWER_SUPPLY_TYPE_MAINS:
	case POWER_SUPPLY_TYPE_UNKNOWN:
	case POWER_SUPPLY_TYPE_PREPARE_TA:
	case POWER_SUPPLY_TYPE_HV_MAINS:
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = 1;
		break;
	default:
		val->intval = 0;
		break;
	}

	return 0;
}

#if defined(CONFIG_MUIC_NOTIFIER) || defined(CONFIG_IFCONN_NOTIFIER)
static int s2mu00x_bat_cable_check(struct s2mu00x_battery_info *battery,
		muic_attached_dev_t attached_dev)
{
	int current_cable_type = -1;

	pr_info("[%s]ATTACHED(%d)\n", __func__, attached_dev);

	switch (attached_dev) {
	case ATTACHED_DEV_SMARTDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	case ATTACHED_DEV_OTG_MUIC:
	case ATTACHED_DEV_HMT_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_OTG;
		break;
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_SMARTDOCK_USB_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_USB_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_USB;
		break;
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_CARDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_VB_MUIC:
	case ATTACHED_DEV_SMARTDOCK_TA_MUIC:
	case ATTACHED_DEV_AFC_CHARGER_5V_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_TA_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_TA_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_ANY_MUIC:
	case ATTACHED_DEV_QC_CHARGER_5V_MUIC:
	case ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_MAINS;
		break;
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case ATTACHED_DEV_AFC_CHARGER_9V_MUIC:
	case ATTACHED_DEV_QC_CHARGER_9V_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_HV_MAINS;
		break;
	case ATTACHED_DEV_UNDEFINED_CHARGING_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
#if defined(CONFIG_IFCONN_NOTIFIER)
	case ATTACHED_DEV_UNDEFINED_RANGE_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_USB;
		break;
	case ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC:
	case ATTACHED_DEV_QC_CHARGER_PREPARE_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_PREPARE_TA;
		break;
	case ATTACHED_DEV_AFC_CHARGER_9V_DUPLI_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_HV_MAINS;
		break;
	case ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC:
		current_cable_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
#endif
#if defined(CONFIG_HV_MUIC_TURBO_CHARGER)
	case ATTACHED_DEV_TURBO_CHARGER:
		current_cable_type = POWER_SUPPLY_TYPE_HV_MAINS;
		pr_info("[%s]Turbo charger ATTACHED\n", __func__);
		break;
#endif

	default:
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;
		pr_err("%s: invalid type for charger:%d\n",
				__func__, attached_dev);
	}

	if (battery->factory_mode && (current_cable_type == POWER_SUPPLY_TYPE_USB ||
				current_cable_type == POWER_SUPPLY_TYPE_USB_CDP)) {
		pr_err("SMB - Factory Kill Armed\n");
		battery->factory_cable = true;
	}

	return current_cable_type;
}
#endif

#if defined(CONFIG_MUIC_NOTIFIER)
static int s2mu00x_battery_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	const char *cmd;
	int cable_type;
	union power_supply_propval value;
	struct s2mu00x_battery_info *battery =
		container_of(nb, struct s2mu00x_battery_info, batt_nb);
	struct power_supply *psy;
	int ret;

	if (attached_dev == ATTACHED_DEV_MHL_MUIC)
		return 0;

	switch (action) {
	case MUIC_NOTIFY_CMD_DETACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_DETACH:
		cmd = "DETACH";
		cable_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	case MUIC_NOTIFY_CMD_ATTACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_ATTACH:
		cmd = "ATTACH";
		cable_type = s2mu00x_bat_cable_check(battery, attached_dev);
		break;
	default:
		cmd = "ERROR";
		cable_type = -1;
		break;
	}

	pr_info("%s: current_cable(%d) former cable_type(%d) battery_valid(%d)\n",
			__func__, cable_type, battery->cable_type,
			battery->battery_valid);
	if (battery->battery_valid == false)
		pr_info("%s: Battery is disconnected\n", __func__);

	battery->cable_type = cable_type;
	pr_info("%s: CMD=%s, attached_dev=%d battery_cable=%d\n",
			__func__, cmd, attached_dev, battery->cable_type);

#if defined(CONFIG_CHARGER_S2MU106)
	if (attached_dev == ATTACHED_DEV_OTG_MUIC) {
		if (!strcmp(cmd, "ATTACH")) {
			value.intval = true;

			psy = power_supply_get_by_name(battery->pdata->charger_name);
			if (!psy)
				return -EINVAL;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);

			pr_info("%s: OTG cable attached\n", __func__);
		} else {
			value.intval = false;

			psy = power_supply_get_by_name(battery->pdata->charger_name);
			if (!psy)
				return -EINVAL;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);

			pr_info("%s: OTG cable detached\n", __func__);
		}
	}

	if (battery->cable_type == POWER_SUPPLY_TYPE_BATTERY ||
			battery->cable_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		battery->is_recharging = false;
		set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
	} else {
		if (battery->cable_type == POWER_SUPPLY_TYPE_OTG) {
			set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
		} else {
			if (battery->status != POWER_SUPPLY_STATUS_FULL)
				set_battery_status(battery, POWER_SUPPLY_STATUS_CHARGING);
		}
	}
#endif
	pr_info(
			"%s: Status(%s), Health(%s), Cable(%d), Recharging(%d))"
			"\n", __func__,
			bat_status_str[battery->status],
			health_str[battery->health],
			battery->cable_type,
			battery->is_recharging
		  );

	power_supply_changed(battery->psy_battery);
	alarm_cancel(&battery->monitor_alarm);
	wake_lock(&battery->monitor_wake_lock);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
	return 0;
}
#endif

#if defined(CONFIG_IFCONN_NOTIFIER)
#if defined(CONFIG_USE_CCIC)
#if defined(CONFIG_USE_PDO_SELECT)
static void usbpd_select_pdo_work(struct work_struct *work)
{
	struct s2mu00x_battery_info *battery =
		container_of(work, struct s2mu00x_battery_info, select_pdo_work.work);

	int pdo_num = battery->pdo_sel_num;
	int ret = -1;

	ret = ifconn_notifier_notify(IFCONN_NOTIFY_BATTERY,
			IFCONN_NOTIFY_MANAGER,
			IFCONN_NOTIFY_ID_SELECT_PDO,
			pdo_num,
			IFCONN_NOTIFY_PARAM_DATA,
			NULL);
	if (ret < 0)
		pr_err("%s: Fail to send noti\n", __func__);

}
#endif
static int s2mu00x_bat_set_pdo(struct s2mu00x_battery_info *battery,
		ifconn_pd_sink_status_t *pdo_data)
{
	int ret = -1;
	int pdo_num = battery->pdo_sel_num;

	if (pdo_num > pdo_data->available_pdo_num + 1 || pdo_num < 1) {
		dev_info(battery->dev, "%s: wrong pdo number. Stop pdo select.\n",
				__func__);
		return ret;
	}
#if defined(CONFIG_USE_PDO_SELECT)
	ret = POWER_SUPPLY_TYPE_PREPARE_TA;

	schedule_delayed_work(&battery->select_pdo_work, msecs_to_jiffies(50));
#else
	dev_info(battery->dev, "%s: skip select pdo work\n",
			__func__);
	ret = POWER_SUPPLY_TYPE_USB_PD;
#endif
	return ret;
}

static void s2mu00x_bat_set_rp_current(struct s2mu00x_battery_info *battery,
		struct ifconn_notifier_template *pd_info)
{
	ifconn_pd_sink_status_t *pd_data =
		&((struct pdic_notifier_data *)pd_info->data)->sink_status;

	switch (pd_data->rp_currentlvl) {
		case RP_CURRENT_LEVEL3:
			battery->rp_input_current = RP_CURRENT3;
			battery->rp_charging_current = RP_CURRENT3;
			break;
		case RP_CURRENT_LEVEL2:
			battery->rp_input_current = RP_CURRENT2;
			battery->rp_charging_current = RP_CURRENT2;
			break;
		case RP_CURRENT_LEVEL_DEFAULT:
		default:
			battery->rp_input_current = RP_CURRENT1;
			battery->rp_charging_current = RP_CURRENT1;
			break;
	}

	dev_info(battery->dev, "%s: rp_currentlvl(%d), input: %d, chg: %d\n",
			__func__, pd_data->rp_currentlvl,
			battery->rp_input_current, battery->rp_charging_current);
}

static int s2mu00x_bat_pdo_check(struct s2mu00x_battery_info *battery,
		struct ifconn_notifier_template *pdo_info)
{
	int current_cable = -1;
	int i;
	int pd_input_current_limit =
		battery->pdata->charging_current[POWER_SUPPLY_TYPE_USB_PD].input_current_limit;
	ifconn_pd_sink_status_t *pdo_data =
		&((struct pdic_notifier_data *)pdo_info->data)->sink_status;

	dev_info(battery->dev, "%s: available_pdo_num:%d, selected_pdo_num:%d,"
		"current_pdo_num:%d\n",
		__func__, pdo_data->available_pdo_num, pdo_data->selected_pdo_num,
		pdo_data->current_pdo_num);

	dev_info(battery->dev, "%s: pdo_max_input_vol:%d, pdo_max_chg_power:%d, "
			"pdo_sel_num:%d\n",
			__func__, battery->pdo_max_input_vol, battery->pdo_max_chg_power,
			battery->pdo_sel_num);

	if (pdo_data->available_pdo_num < 0)
		return current_cable;

	if (battery->pdo_sel_num == pdo_data->selected_pdo_num) {
		dev_info(battery->dev, "%s: Already done. Finish pdo check.\n",
				__func__);
		current_cable = POWER_SUPPLY_TYPE_USB_PD;
		goto end_pdo_check;
	}

	for (i = 1; i <= pdo_data->available_pdo_num; i++) {
		dev_info(battery->dev, "%s: pdo_num:%d, max_voltage:%d, max_current:%d\n",
				__func__, i, pdo_data->power_list[i].max_voltage,
				pdo_data->power_list[i].max_current);

		if (pdo_data->power_list[i].max_voltage > battery->pdo_max_input_vol)
			continue;

		pd_input_current_limit = (pd_input_current_limit > pdo_data->power_list[i].max_current)?
			pdo_data->power_list[i].max_current:pd_input_current_limit;

		if (((pdo_data->power_list[i].max_voltage/1000) * pd_input_current_limit) <=
				battery->pdo_max_chg_power) {
				battery->pdo_sel_num = i;
				battery->pdo_sel_vol = pdo_data->power_list[i].max_voltage;
				battery->pdo_sel_cur = pdo_data->power_list[i].max_current;
				dev_info(battery->dev, "%s: new pdo_sel_num:%d\n",
						__func__, battery->pdo_sel_num);
		}
	}

	battery->pd_input_current = pd_input_current_limit;

	if (battery->pdo_sel_num == 0) {
		dev_info(battery->dev, "%s: There is no proper pdo. Do normal TA setting\n", __func__);
		current_cable = POWER_SUPPLY_TYPE_MAINS;
	} else
		current_cable = s2mu00x_bat_set_pdo(battery, pdo_data);

end_pdo_check:
	return current_cable;
}
#endif
static int s2mu00x_ifconn_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct s2mu00x_battery_info *battery =
			container_of(nb, struct s2mu00x_battery_info, ifconn_nb);
	struct ifconn_notifier_template *ifconn_info = (struct ifconn_notifier_template *)data;
	muic_attached_dev_t attached_dev = (muic_attached_dev_t)ifconn_info->event;
	const char *cmd;
	int cable_type;
	union power_supply_propval value;
	struct power_supply *psy;
	int ret;
#if defined(CONFIG_USE_CCIC)
	struct pdic_notifier_data *pdic_info;
#endif

	dev_info(battery->dev, "%s: action(%ld) dump(0x%01x, 0x%01x, 0x%02x, 0x%04x, 0x%04x, 0x%04x, 0x%04x)\n",
		__func__, action, ifconn_info->src, ifconn_info->dest, ifconn_info->id,
		ifconn_info->attach, ifconn_info->rprd, ifconn_info->cable_type, ifconn_info->event);

	ifconn_info->cable_type = (muic_attached_dev_t)ifconn_info->event;
#if defined(CONFIG_USE_CCIC)
	dev_info(battery->dev, "%s: pd_attach(%d) rp_attach(%d)\n",
			__func__, battery->pd_attach, battery->rp_attach);
#endif
	action = ifconn_info->id;
	mutex_lock(&battery->ifconn_lock);

	if (attached_dev == ATTACHED_DEV_MHL_MUIC) {
		mutex_unlock(&battery->ifconn_lock);
		return 0;
	}

	switch (action) {
	case IFCONN_NOTIFY_ID_DETACH:
#if defined(CONFIG_USE_CCIC)
		if ((ifconn_info->src == IFCONN_NOTIFY_MANAGER) && battery->pd_attach) {
			pr_info("%s, Skip cable check when PD TA attaching\n", __func__);
			mutex_unlock(&battery->ifconn_lock);
			return 0;
		}

		battery->pd_attach = false;

		battery->rp_attach = false;
		battery->rp_input_current = 0;
		battery->rp_charging_current = 0;
#endif
		cmd = "DETACH";
		cable_type = POWER_SUPPLY_TYPE_BATTERY;
		//moto
		factory_usb_shutdown(battery);
		break;
	case IFCONN_NOTIFY_ID_ATTACH:
#if defined(CONFIG_USE_CCIC)
		if ((ifconn_info->src == IFCONN_NOTIFY_MANAGER) && battery->pd_attach) {
			pr_info("%s: PD TA is attached. Skip cable check\n", __func__);
			cable_type =  POWER_SUPPLY_TYPE_USB_PD;
			cmd = "PD ATTACH";
			break;
		}
#endif
		cmd = "ATTACH";
		cable_type = s2mu00x_bat_cable_check(battery, attached_dev);
		break;
#if defined(CONFIG_USE_CCIC)
	case IFCONN_NOTIFY_ID_POWER_STATUS:
		pdic_info = (struct pdic_notifier_data *)ifconn_info->data;

		if (pdic_info->event == IFCONN_NOTIFY_EVENT_RP_ATTACH) {
			if (battery->pd_attach) {
				pr_info("%s: Skip Rp current setting when PD TA attached\n",
						__func__);
				mutex_unlock(&battery->ifconn_lock);
				return 0;
			}
			/* Do Rp current setting*/
			s2mu00x_bat_set_rp_current(battery, ifconn_info);
			cmd = "Rp ATTACH";
			battery->rp_attach = true;
			cable_type = battery->cable_type;
			attached_dev = ATTACHED_DEV_TYPE3_CHARGER_MUIC;
		} else {
			cable_type = s2mu00x_bat_pdo_check(battery, ifconn_info);
			battery->pd_attach = true;
			if (battery->rp_attach) {
				pr_info("%s: PD TA attached after Rp current setting!"
						"Clear rp_attach flag\n",
						__func__);
				battery->rp_attach = false;
			}

			switch (cable_type) {
				case POWER_SUPPLY_TYPE_USB_PD:
					cmd = "PD ATTACH";
					attached_dev = ATTACHED_DEV_TYPE3_CHARGER_MUIC;
					break;
				case POWER_SUPPLY_TYPE_PREPARE_TA:
					cmd = "PD PREPARE";
					attached_dev = ATTACHED_DEV_TYPE3_CHARGER_MUIC;
					break;
				default:
					cmd = "PD FAIL";
					break;
			}
		}
		break;
#endif
	default:
		cmd = "ERROR";
		cable_type = -1;
		break;
	}

	pr_info("%s: CMD[%s] attached_dev(%d) current_cable(%d) former cable_type(%d) battery_valid(%d)\n",
			__func__, cmd,  attached_dev, cable_type,
			battery->cable_type, battery->battery_valid);

	if (battery->battery_valid == false)
		pr_info("%s: Battery is disconnected\n", __func__);

	battery->cable_type = cable_type;
	//moot
	battery->usb_present =  is_usb_present(battery);
	battery->dc_present =  is_dc_present(battery);

#if 0 // defined(CONFIG_USE_CCIC)
	if (cable_type == POWER_SUPPLY_TYPE_PREPARE_TA)
		goto end_ifconn_handle;
#endif

	if (attached_dev == ATTACHED_DEV_OTG_MUIC) {
		if (!strcmp(cmd, "ATTACH")) {
			value.intval = true;

			psy = power_supply_get_by_name(battery->pdata->charger_name);
			if (!psy) {
				mutex_unlock(&battery->ifconn_lock);
				return -EINVAL;
			}
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);

			pr_info("%s: OTG cable attached\n", __func__);
		} else {
			value.intval = false;

			psy = power_supply_get_by_name(battery->pdata->charger_name);
			if (!psy) {
				mutex_unlock(&battery->ifconn_lock);
				return -EINVAL;
			}
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);

			pr_info("%s: OTG cable detached\n", __func__);
		}
	}
	set_bat_status_by_cable(battery);

#if 0 //defined(CONFIG_USE_CCIC)
end_ifconn_handle:
#endif
	pr_info("%s: Status(%s), Health(%s), Cable(%d), Recharging(%d)\n",
			__func__, bat_status_str[battery->status], health_str[battery->health],
			battery->cable_type, battery->is_recharging);

	power_supply_changed(battery->psy_battery);
	alarm_cancel(&battery->monitor_alarm);
	wake_lock(&battery->monitor_wake_lock);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
	mutex_unlock(&battery->ifconn_lock);

	if (action == IFCONN_NOTIFY_ID_DETACH) {
		smbchg_relax(battery);
		cancel_delayed_work_sync(&battery->heartbeat_work);
		battery->stepchg_state = STEP_NONE;
		battery->charging_limit_modes = CHARGING_LIMIT_OFF;
		set_max_allowed_current_ma(battery,battery->stepchg_current_ma);
		battery->charger_rate = POWER_SUPPLY_CHARGE_RATE_NONE;
	} else {
		battery->charger_rate = POWER_SUPPLY_CHARGE_RATE_NORMAL;
		smbchg_stay_awake(battery);
		cancel_delayed_work(&battery->heartbeat_work);
		schedule_delayed_work(&battery->heartbeat_work, msecs_to_jiffies(0));
	}
	return 0;
}
#endif

static void get_battery_capacity(struct s2mu00x_battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int ret;
	unsigned int raw_soc = 0;
	int new_capacity = 0;

	if (battery->test_mode && !(battery->test_mode_soc < 0)
	    && !(battery->test_mode_soc > 100)) {
		battery->capacity = battery->test_mode_soc;
		return ;
	}

	psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	raw_soc = value.intval;

	if (battery->status == POWER_SUPPLY_STATUS_FULL) {
		battery->max_rawsoc = raw_soc - battery->max_rawsoc_offset;
		if (battery->max_rawsoc <= 0)
			battery->max_rawsoc = 10;
	}

	new_capacity = (raw_soc * 100) / battery->max_rawsoc;

	if ((new_capacity == 0) && (raw_soc != 0)) {
		dev_info(battery->dev, "%s: new_capacity is 0, "
				"but raw_soc is not 0. Maintain SOC 1\n", __func__);
		new_capacity = 1;
	}

	if (new_capacity > 100)
		new_capacity = 100;

	if (new_capacity > battery->capacity)
		new_capacity = battery->capacity + 1;
	else if (new_capacity < battery->capacity)
		new_capacity = battery->capacity - 1;

	if (new_capacity > 100)
		new_capacity = 100;
	else if (new_capacity < 0)
		new_capacity = 0;

	battery->capacity = new_capacity;

	dev_info(battery->dev, "%s: SOC(%u), rawsoc(%d), max_rawsoc(%u).\n",
		__func__, battery->capacity, raw_soc, battery->max_rawsoc);
}

static int get_battery_info(struct s2mu00x_battery_info *battery)
{
	union power_supply_propval value;
	struct power_supply *psy;
	int ret;

	/*Get fuelgauge psy*/
	psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
	if (!psy)
		return -EINVAL;

	/* Get voltage and current value */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->voltage_now = value.intval;

	value.intval = S2MU00X_BATTERY_VOLTAGE_AVERAGE;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_AVG, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->voltage_avg = value.intval;

	value.intval = S2MU00X_BATTERY_CURRENT_MA;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->current_now = value.intval;

	value.intval = S2MU00X_BATTERY_CURRENT_MA;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_AVG, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->current_avg = value.intval;

	/* Get temperature info */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->temperature = value.intval;
	/* Get charge temperature info */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_TEMP, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->charge_temp = value.intval;

	get_battery_capacity(battery);

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_SOH, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->soh = value.intval;

#if defined(CONFIG_CHARGER_S2MU106)
	/*Get charger psy*/
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;

	/* Get input current limit */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->current_max = value.intval;

	/* Get charge current limit */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->current_chg = value.intval;

	/* Get charger status*/
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	if (battery->status != value.intval)
		pr_err("%s: battery status = %d, charger status = %d\n",
				__func__, battery->status, value.intval);
#endif
	psy = power_supply_get_by_name("s2mu106_pmeter");
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VCHGIN, &value);

	/* Get input voltage & current from powermeter */
	battery->vchg_voltage = value.intval;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ICHGIN, &value);
	battery->vchg_current = value.intval;

	dev_info(battery->dev,
			"%s:Vnow(%dmV),Inow(%dmA),Imax(%dmA),Ichg(%dmA),SOC(%d%%),Tbat(%d),SOH(%d%%)"
			",Vbus(%dmV),Ibus(%dmA)"
			"\n", __func__,
			battery->voltage_now, battery->current_now,
			battery->current_max, battery->current_chg, battery->capacity,
			battery->temperature, battery->soh,
			battery->vchg_voltage, battery->vchg_current
			);
	dev_dbg(battery->dev,
			"%s,Vavg(%dmV),Vocv(%dmV),Iavg(%dmA)\n",
			battery->battery_valid ? "Connected" : "Disconnected",
			battery->voltage_avg, battery->voltage_ocv, battery->current_avg);

#if defined(CONFIG_SMALL_CHARGER)
	psy = power_supply_get_by_name(battery->pdata->smallcharger_name);
	if (!psy)
		return -EINVAL;

	/* Get input current limit */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->small_input = value.intval;

	/* Get charge current limit */
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);
	battery->small_chg = value.intval;

	dev_info(battery->dev,
			"%s: small Imax(%dmA), Ichg(%dmA)\n", __func__,
			battery->small_input, battery->small_chg);
#endif
	return 0;
}

static int get_battery_health(struct s2mu00x_battery_info *battery)
{
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;
#if defined(CONFIG_CHARGER_S2MU106)
	struct power_supply *psy;
	union power_supply_propval value;
	int ret;
	/* Get health status from charger */
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_HEALTH, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	health = value.intval;
#else
	health = POWER_SUPPLY_HEALTH_GOOD;
#endif

	return health;
}

static int get_temperature_health(struct s2mu00x_battery_info *battery)
{
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;

	switch (battery->health) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		if (battery->temperature < battery->temp_high_recovery)
			health = POWER_SUPPLY_HEALTH_GOOD;
		else
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		if (battery->temperature > battery->temp_low_recovery)
			health = POWER_SUPPLY_HEALTH_GOOD;
		else
			health = POWER_SUPPLY_HEALTH_COLD;
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
	default:
		if (battery->temperature > battery->temp_high)
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (battery->temperature < battery->temp_low)
			health = POWER_SUPPLY_HEALTH_COLD;
		else
			health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

	/* For test, Temperature health is always good*/
	health = POWER_SUPPLY_HEALTH_GOOD;

	return health;
}

static void check_health(struct s2mu00x_battery_info *battery)
{
	int battery_health = 0;
	int temperature_health = 0;

	battery_health = get_battery_health(battery);
	temperature_health = get_temperature_health(battery);

	pr_info("%s: T = %d, bat_health(%s), T_health(%s), Charging(%s)\n",
		__func__, battery->temperature, health_str[battery_health],
		health_str[temperature_health], bat_status_str[battery->status]);

	/* If battery & temperature both are normal,			 *
	 *	set battery->health GOOD and recover battery->status */
	if (battery_health == POWER_SUPPLY_HEALTH_GOOD &&
		temperature_health == POWER_SUPPLY_HEALTH_GOOD) {
		battery->health = POWER_SUPPLY_HEALTH_GOOD;
		if (battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			set_bat_status_by_cable(battery);
		return;
	}

	switch (battery_health) {
	case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
	case POWER_SUPPLY_HEALTH_UNDERVOLTAGE:
	case POWER_SUPPLY_HEALTH_UNKNOWN:
		battery->health = battery_health;
		goto abnormal_health;
	default:
		break;
	}
	switch (temperature_health) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
	case POWER_SUPPLY_HEALTH_COLD:
	case POWER_SUPPLY_HEALTH_UNKNOWN:
		battery->health = temperature_health;
		goto abnormal_health;
	default:
		break;
	}

	pr_err("%s: Abnormal case of temperature & battery health.\n", __func__);
	return;

abnormal_health:
	if (battery->status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
		battery->is_recharging = false;
		/* Take the wakelock during 10 seconds	*
		 * when not_charging status is detected */
		wake_lock_timeout(&battery->vbus_wake_lock, HZ * 10);
		set_battery_status(battery, POWER_SUPPLY_STATUS_NOT_CHARGING);
	}
}

static void check_charging_full(
		struct s2mu00x_battery_info *battery)
{
	pr_info("%s Start\n", __func__);

	if ((battery->status == POWER_SUPPLY_STATUS_DISCHARGING) ||
			(battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING)) {
		dev_dbg(battery->dev,
				"%s: No Need to Check Full-Charged\n", __func__);
		battery->full_check_cnt = 0;
		return;
	}

	/* 1. Recharging check */
	if (battery->status == POWER_SUPPLY_STATUS_FULL &&
			battery->voltage_now < battery->pdata->chg_recharge_vcell &&
			!battery->is_recharging) {
		pr_info("%s: Recharging start\n", __func__);
		set_battery_status(battery, POWER_SUPPLY_STATUS_CHARGING);
		battery->is_recharging = true;
	}

	/* 2. Full charged check */
	if ((battery->current_now >= 0 && battery->current_now <
				battery->pdata->charging_current[
				battery->cable_type].full_check_current) &&
			(battery->voltage_avg > battery->pdata->chg_full_vcell)) {
		battery->full_check_cnt++;
		pr_info("%s: Full Check Cnt (%d)\n", __func__, battery->full_check_cnt);
	} else if (battery->full_check_cnt != 0) {
	/* Reset full check cnt when it is out of full condition */
		battery->full_check_cnt = 0;
		pr_info("%s: Reset Full Check Cnt\n", __func__);
	}

	/* 3. If full charged, turn off charging. */
	if (battery->full_check_cnt >= battery->pdata->full_check_count) {
		battery->full_check_cnt = 0;
		battery->is_recharging = false;
		set_battery_status(battery, POWER_SUPPLY_STATUS_FULL);
		pr_info("%s: Full charged, charger off\n", __func__);
	}
}

static void bat_monitor_work(struct work_struct *work)
{
	struct s2mu00x_battery_info *battery =
		container_of(work, struct s2mu00x_battery_info, monitor_work.work);
#if defined(CONFIG_CHARGER_S2MU106)
	union power_supply_propval value;
	struct power_supply *psy;
	int ret;
#endif
	pr_info("%s: start monitoring\n", __func__);
#if defined(CONFIG_CHARGER_S2MU106)
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	if (!value.intval) {
		battery->battery_valid = false;
		pr_info("%s: There is no battery, skip monitoring.\n", __func__);
		goto continue_monitor;
	} else
		battery->battery_valid = true;
#else
	battery->battery_valid = true;
#endif
	get_battery_info(battery);

	check_health(battery);

	check_charging_full(battery);

	if (is_charging_mode == S2MU00X_FAC_MODE) {
		pr_info("%s: Factory boot mode and reinsert adapter, stop charging\n", __func__);
		set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
	}

	power_supply_changed(battery->psy_battery);
#if defined(CONFIG_CHARGER_S2MU106)
continue_monitor:
#endif
	pr_err(
		 "%s: Status(%s), Health(%s), Cable(%d), Recharging(%d))"
		 "\n", __func__,
		 bat_status_str[battery->status],
		 health_str[battery->health],
		 battery->cable_type,
		 battery->is_recharging
		 );

	alarm_cancel(&battery->monitor_alarm);
	alarm_start_relative(&battery->monitor_alarm, ktime_set(battery->monitor_alarm_interval, 0));
	wake_unlock(&battery->monitor_wake_lock);

}

#ifdef CONFIG_OF
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	retval = of_property_read_u32(chip,		\
					"moto," dt_property,		\
					&prop);				\
									\
	if (retval)						\
		pr_info("%s : %s  is empty\n", __func__, dt_property); \
	else \
		pr_info("%s : %s  is find\n", __func__, dt_property); \
} while (0)

static int parse_dt_pchg_current_map(const u32 *arr,
				     struct pchg_current_map *current_map,
				     int count)
{
	u32 len = 0;
	u32 requested;
	u32 primary;
	u32 secondary;
	int i;

	if (!arr)
		return 0;

	for (i = 0; i < count*3; i += 3) {
		requested = be32_to_cpu(arr[i]);
		primary = be32_to_cpu(arr[i + 1]);
		secondary = be32_to_cpu(arr[i + 2]);
		current_map->requested = requested;
		current_map->primary = primary;
		current_map->secondary = secondary;
		len++;
		current_map++;
	}
	return len;
}

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
static int mmi_parse_dt(struct device_node *np,
			struct s2mu00x_battery_info *chip, struct device *dev)
{
	int rc = 0;
	struct device_node *node = np;
	const u32 *current_map;
	int ret = 0;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* read optional u32 properties */
	ret = of_property_read_u32(node, "moto,iterm-ma",
			&chip->iterm_ma);
	if (ret)
		pr_info("%s : moto,iterm-ma is empty\n", __func__);

	OF_PROP_READ(node, chip->target_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);

	OF_PROP_READ(node, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(node, chip->fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(node, chip->float_voltage_comp, "float-voltage-comp",
			rc, 1);
	OF_PROP_READ(node, chip->afvc_mv, "auto-voltage-comp-mv",
			rc, 1);
	OF_PROP_READ(node, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(node, chip->jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);

	OF_PROP_READ(node, chip->hot_temp_c,
		     "hot-temp-c", rc, 1);
	if (chip->hot_temp_c == -EINVAL)
		chip->hot_temp_c = 60;

	OF_PROP_READ(node, chip->cold_temp_c,
		     "cold-temp-c", rc, 1);
	if (chip->cold_temp_c == -EINVAL)
		chip->cold_temp_c = -20;

	OF_PROP_READ(node, chip->warm_temp_c,
		     "warm-temp-c", rc, 1);
	if (chip->warm_temp_c == -EINVAL)
		chip->warm_temp_c = 45;

	OF_PROP_READ(node, chip->cool_temp_c,
		     "cool-temp-c", rc, 1);
	if (chip->cool_temp_c == -EINVAL)
		chip->cool_temp_c = 0;
	OF_PROP_READ(node, chip->slightly_cool_temp_c,
		     "slightly-cool-temp-c", rc, 1);
	if (chip->slightly_cool_temp_c == -EINVAL)
		chip->slightly_cool_temp_c = 15;
	OF_PROP_READ(node, chip->ext_temp_volt_mv,
		     "ext-temp-volt-mv", rc, 1);
	if (chip->ext_temp_volt_mv == -EINVAL)
		chip->ext_temp_volt_mv = 4200;

	OF_PROP_READ(node, chip->hotspot_thrs_c,
		     "hotspot-thrs-c", rc, 1);
	if (chip->hotspot_thrs_c == -EINVAL)
		chip->hotspot_thrs_c = 50;

	OF_PROP_READ(node, chip->upper_limit_capacity,
		     "upper-limit-capacity", rc, 1);
	if (chip->upper_limit_capacity == -EINVAL)
		chip->upper_limit_capacity = 75;

	OF_PROP_READ(node, chip->lower_limit_capacity,
		     "lower-limit-capacity", rc, 1);
	if (chip->lower_limit_capacity == -EINVAL)
		chip->lower_limit_capacity = 60;

	OF_PROP_READ(node, chip->stepchg_voltage_mv,
			"stepchg-voltage-mv", rc, 1);

	OF_PROP_READ(node, chip->stepchg_current_ma,
			"stepchg-current-ma", rc, 1);

	OF_PROP_READ(node, chip->stepchg_taper_ma,
			"stepchg-taper-ma", rc, 1);

	OF_PROP_READ(node, chip->stepchg_iterm_ma,
			"stepchg-iterm-ma", rc, 1);
	if ((chip->stepchg_current_ma != -EINVAL) &&
	    (chip->stepchg_voltage_mv != -EINVAL) &&
	    (chip->stepchg_taper_ma != -EINVAL) &&
	    (chip->stepchg_iterm_ma != -EINVAL)) {
		chip->stepchg_max_current_ma = chip->target_fastchg_current_ma;
		chip->allowed_fastchg_current_ma =
			chip->target_fastchg_current_ma;
		chip->stepchg_max_voltage_mv = chip->vfloat_mv;
	}
	OF_PROP_READ(node, chip->temp_warm_current_ma,
			"temp-warm-current-ma", rc, 1);
	if (chip->temp_warm_current_ma == -EINVAL)
		chip->temp_warm_current_ma = chip->target_fastchg_current_ma;
	OF_PROP_READ(node, chip->temp_cool_current_ma,
			"temp-cool-current-ma", rc, 1);
	if (chip->temp_cool_current_ma == -EINVAL)
		chip->temp_cool_current_ma = chip->target_fastchg_current_ma;
	OF_PROP_READ(node, chip->temp_slightly_cool_current_ma,
			"temp-slightly-cool-current-ma", rc, 1);
	if (chip->temp_slightly_cool_current_ma == -EINVAL)
		chip->temp_slightly_cool_current_ma =
			chip->target_fastchg_current_ma;
	chip->temp_good_current_ma = chip->target_fastchg_current_ma;
	chip->temp_allowed_fastchg_current_ma = chip->temp_good_current_ma;

	/* read boolean configuration properties */
	chip->use_vfloat_adjustments = of_property_read_bool(node,
						"moto,autoadjust-vfloat");
	chip->chg_enabled = !(of_property_read_bool(node,
						"moto,charging-disabled"));
	chip->charge_unknown_battery = of_property_read_bool(node,
						"moto,charge-unknown-battery");
	chip->chg_inhibit_en = of_property_read_bool(node,
					"moto,chg-inhibit-en");
	chip->chg_inhibit_source_fg = of_property_read_bool(node,
						"moto,chg-inhibit-fg");
	chip->low_volt_dcin = of_property_read_bool(node,
					"moto,low-volt-dcin");
	chip->usbid_disabled = of_property_read_bool(node,
						"moto,usbid-disabled");
	chip->usbid_gpio_enabled = of_property_read_bool(node,
						"moto,usbid-gpio-enabled");
	chip->enable_hvdcp_9v = of_property_read_bool(node,
					"moto,enable-hvdcp-9v");
	chip->enable_charging_limit = of_property_read_bool(node,
					"moto,enable-charging-limit");
	chip->enabled_weak_charger_check = of_property_read_bool(node,
					"moto,weak-charger-check-enable");

	/* parse the dc power supply configuration */
	current_map = of_get_property(node, "moto,parallel-charge-current-map",
				      &chip->pchg_current_map_len);
	if ((!current_map) || (chip->pchg_current_map_len <= 0))
		dev_err(chip->dev, "No parallel charge current map defined\n");
	else {
		chip->pchg_current_map_len /= 3 * sizeof(u32);
		dev_err(chip->dev, "length=%d\n", chip->pchg_current_map_len);
		if (chip->pchg_current_map_len > 30)
			chip->pchg_current_map_len = 30;

		chip->pchg_current_map_data =
			devm_kzalloc(dev,
				     (sizeof(struct pchg_current_map) *
				      chip->pchg_current_map_len),
				     GFP_KERNEL);
		if (chip->pchg_current_map_data == NULL) {
			dev_err(chip->dev,
			 "Failed to kzalloc memory for parallel charge map.\n");
			return -ENOMEM;
		}

		chip->pchg_current_map_len =
			parse_dt_pchg_current_map(current_map,
						  chip->pchg_current_map_data,
						  chip->pchg_current_map_len);

		if (chip->pchg_current_map_len <= 0) {
			dev_err(chip->dev,
			"Couldn't read parallel charge currents rc = %d\n", rc);
			return rc;
		}
		dev_err(chip->dev, "num parallel charge entries=%d\n",
			chip->pchg_current_map_len);
	}


	if (of_find_property(node, "moto,chg-thermal-mitigation",
					&chip->chg_thermal_levels)) {
		chip->chg_thermal_mitigation = devm_kzalloc(dev,
			chip->chg_thermal_levels,
			GFP_KERNEL);
		if (chip->chg_thermal_mitigation == NULL) {
			pr_info("%s : thermal mitigation kzalloc() failed. \n", __func__);
			dev_err(chip->dev,
			"thermal mitigation kzalloc() failed.\n");
		return -ENOMEM;
		}

		chip->chg_thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
					"moto,chg-thermal-mitigation",
					chip->chg_thermal_mitigation,
					chip->chg_thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read therm limits rc = %d\n", rc);
			return rc;
		}
	} else
		chip->chg_thermal_levels = 0;

	if (of_find_property(node, "moto,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"moto,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read therm limits rc = %d\n", rc);
			return rc;
		}
	} else
		chip->thermal_levels = 0;

	if (of_find_property(node, "moto,dc-thermal-mitigation",
			     &chip->dc_thermal_levels)) {
		chip->dc_thermal_mitigation = devm_kzalloc(dev,
			chip->dc_thermal_levels,
			GFP_KERNEL);

		if (chip->dc_thermal_mitigation == NULL) {
			pr_info("DC thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->dc_thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"moto,dc-thermal-mitigation",
				chip->dc_thermal_mitigation,
				chip->dc_thermal_levels);
		if (rc) {
			pr_info("Couldn't read DC therm limits rc = %d\n", rc);
			return rc;
		}
	} else
		chip->dc_thermal_levels = 0;

	return 0;
}

static int s2mu00x_battery_parse_dt(struct device *dev,
		struct s2mu00x_battery_info *battery)
{
	struct device_node *np = of_find_node_by_name(NULL, "battery");
	s2mu00x_battery_platform_data_t *pdata = battery->pdata;
	int ret = 0, len;
	unsigned int i;
	const u32 *p;
	u32 temp;
	u32 default_input_current, default_charging_current, default_full_check_current;

	if (!np) {
		pr_info("%s np NULL(battery)\n", __func__);
		return -1;
	}
	ret = of_property_read_string(np,
			"battery,vendor", (char const **)&pdata->vendor);
	if (ret)
		pr_info("%s: Vendor is empty\n", __func__);

	ret = of_property_read_string(np,
			"battery,charger_name", (char const **)&pdata->charger_name);
	if (ret)
		pr_info("%s: Charger name is empty\n", __func__);

#if defined(CONFIG_SMALL_CHARGER)
	ret = of_property_read_string(np,
			"battery,smallcharger_name", (char const **)&pdata->smallcharger_name);
	if (ret)
		pr_info("%s: Small charger name is empty\n", __func__);
#endif

	ret = of_property_read_string(np,
			"battery,fuelgauge_name", (char const **)&pdata->fuelgauge_name);
	if (ret)
		pr_info("%s: Fuelgauge name is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,technology",
			&pdata->technology);
	if (ret)
		pr_info("%s : technology is empty\n", __func__);

	p = of_get_property(np, "battery,input_current_limit", &len);
	if (!p)
		return 1;

	len = len / sizeof(u32);

	if (len < POWER_SUPPLY_TYPE_END)
		len = POWER_SUPPLY_TYPE_END;

	pdata->charging_current = kzalloc(sizeof(s2mu00x_charging_current_t) * len,
			GFP_KERNEL);

	ret = of_property_read_u32(np, "battery,default_input_current",
			&default_input_current);
	if (ret)
		pr_info("%s : default_input_current is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,default_charging_current",
			&default_charging_current);
	if (ret)
		pr_info("%s : default_charging_current is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,default_full_check_current",
			&default_full_check_current);
	if (ret)
		pr_info("%s : default_full_check_current is empty\n", __func__);

	for (i = 0; i < len; i++) {
		ret = of_property_read_u32_index(np,
				"battery,input_current_limit", i,
				&pdata->charging_current[i].input_current_limit);
		if (ret) {
			pr_info("%s : Input_current_limit is empty\n",
					__func__);
			pdata->charging_current[i].input_current_limit = default_input_current;
		}

		ret = of_property_read_u32_index(np,
				"battery,fast_charging_current", i,
				&pdata->charging_current[i].fast_charging_current);
		if (ret) {
			pr_info("%s : Fast charging current is empty\n",
					__func__);
			pdata->charging_current[i].fast_charging_current = default_charging_current;
		}

		ret = of_property_read_u32_index(np,
				"battery,full_check_current", i,
				&pdata->charging_current[i].full_check_current);
		if (ret) {
			pr_info("%s : Full check current is empty\n",
					__func__);
			pdata->charging_current[i].full_check_current = default_full_check_current;
		}
	}

	ret = of_property_read_u32(np, "battery,max_input_current",
			&pdata->max_input_current);
	if (ret)
		pr_info("%s : max_input_current is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,max_charging_current",
			&pdata->max_charging_current);
	if (ret)
		pr_info("%s : max_charging_current is empty\n", __func__);

#if defined(CONFIG_SMALL_CHARGER)
	ret = of_property_read_u32(np, "battery,small_input_current",
			&pdata->small_input_current);
	if (ret) {
		pr_info("%s : small_input_current is empty\n", __func__);
		pdata->small_input_current = 500;
	}

	ret = of_property_read_u32(np, "battery,small_charging_current",
			&pdata->small_charging_current);
	if (ret) {
		pr_info("%s : small_charging_current is empty\n", __func__);
		pdata->small_charging_current = 800;
	}
#endif

#if defined(CONFIG_USE_CCIC)
	ret = of_property_read_u32(np, "battery,pdo_max_chg_power",
			&pdata->pdo_max_chg_power);
	if (ret)
		pr_info("%s : pdo_max_chg_power is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,pdo_max_input_vol",
			&pdata->pdo_max_input_vol);
	if (ret)
		pr_info("%s : pdo_max_input_vol is empty\n", __func__);
#endif

	ret = of_property_read_u32(np, "battery,temp_high", &temp);
	if (ret) {
		pr_info("%s : temp_high is empty\n", __func__);
		pdata->temp_high = 500;
	} else
		pdata->temp_high = (int)temp;

	ret = of_property_read_u32(np, "battery,temp_high_recovery", &temp);
	if (ret) {
		pr_info("%s : temp_high_recovery is empty\n", __func__);
		pdata->temp_high_recovery = pdata->temp_high - 50;
	} else
		pdata->temp_high_recovery = (int)temp;

	ret = of_property_read_u32(np, "battery,temp_low", &temp);
	if (ret) {
		pr_info("%s : temp_low is empty\n", __func__);
		pdata->temp_low = 100;
	} else
		pdata->temp_low = (int)temp;

	ret = of_property_read_u32(np, "battery,temp_low_recovery", &temp);
	if (ret) {
		pr_info("%s : temp_low_recovery is empty\n", __func__);
		pdata->temp_low_recovery = pdata->temp_low + 50;
	} else
		pdata->temp_low_recovery = (int)temp;

	pr_info("%s : temp_high(%d), temp_high_recovery(%d), temp_low(%d), temp_low_recovery(%d)\n",
			__func__,
			pdata->temp_high, pdata->temp_high_recovery,
			pdata->temp_low, pdata->temp_low_recovery);

	ret = of_property_read_u32(np, "battery,chg_float_voltage",
			&pdata->chg_float_voltage);
	if (ret) {
		pr_info("%s : chg_float_voltage is empty\n", __func__);
		pdata->chg_float_voltage = 4200;
	}

	ret = of_property_read_u32(np, "battery,full_check_count",
			&pdata->full_check_count);
	if (ret)
		pr_info("%s : full_check_count is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,chg_full_vcell",
			&pdata->chg_full_vcell);
	if (ret)
		pr_info("%s : chg_full_vcell is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,chg_recharge_vcell",
			&pdata->chg_recharge_vcell);
	if (ret)
		pr_info("%s : chg_recharge_vcell is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,max_rawsoc",
			&pdata->max_rawsoc);
	if (ret)
		pr_info("%s : max_rawsoc is empty\n", __func__);

	ret = of_property_read_u32(np, "battery,max_rawsoc_offset",
			&pdata->max_rawsoc_offset);
	if (ret)
		pr_info("%s : max_rawsoc_offset is empty\n", __func__);

	pr_info("%s:DT parsing is done, vendor : %s, technology : %d\n",
			__func__, pdata->vendor, pdata->technology);
	//moto
	mmi_parse_dt(np, battery, dev);

	pr_info("%s:DT parsing is done for mmi, vendor : %s, technology : %d\n",
			__func__, pdata->vendor, pdata->technology);
	return ret;
}
#else
static int s2mu00x_battery_parse_dt(struct device *dev,
		struct s2mu00x_battery_platform_data *pdata)
{
	return pdev->dev.platform_data;
}
#endif

static const struct of_device_id s2mu00x_battery_match_table[] = {
	{ .compatible = "samsung,s2mu00x-battery",},
	{},
};

static enum alarmtimer_restart bat_monitor_alarm(
	struct alarm *alarm, ktime_t now)
{
	struct s2mu00x_battery_info *battery = container_of(alarm,
				struct s2mu00x_battery_info, monitor_alarm);

	wake_lock(&battery->monitor_wake_lock);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);

	return ALARMTIMER_NORESTART;
}

//moto
#define HYSTERISIS_DEGC 2
#define MAX_TEMP_C 60
#define MIN_MAX_TEMP_C 47

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_HEARTBEAT = BIT(3),
	PM_CHARGER = BIT(4),
	PM_WIRELESS = BIT(5),
};

static int smbchg_chg_system_temp_level_set(struct s2mu00x_battery_info *chip,
					    int lvl_sel)
{
	struct s2mu00x_battery_info *battery =
		container_of(work, struct s2mu00x_battery_info, soc_control.work);
	pr_err("%s \n", __func__);
	return 1;
}

static ssize_t charger_set_store(struct device *dev,
			struct device_attribute *devattr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct s2mu00x_battery_info *battery = power_supply_get_drvdata(psy);
	int enable;

	sscanf(buf, "%d", &enable);
	pr_err("%s enable: %d\n", __func__, enable);

	if(enable == 1) {
		battery->cable_type = POWER_SUPPLY_TYPE_MAINS;
		alarm_cancel(&battery->monitor_alarm);
		wake_lock(&battery->monitor_wake_lock);
		queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
	}
	else {
		battery->cable_type = POWER_SUPPLY_TYPE_BATTERY;
		alarm_cancel(&battery->monitor_alarm);
		wake_lock(&battery->monitor_wake_lock);
		queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);

	}
	return count;
}
static ssize_t charger_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct s2mu00x_battery_info *battery = power_supply_get_drvdata(psy);

	if(battery->cable_type == POWER_SUPPLY_TYPE_MAINS)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");

	pr_err("%s \n", __func__);
	return 1;
}
static ssize_t charger_status_store(struct device *dev,
			struct device_attribute *devattr, const char *buf, size_t count)
{
	pr_err("%s \n", __func__);
	return count;
}

static ssize_t charger_current_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct s2mu00x_battery_info *battery = power_supply_get_drvdata(psy);

	return sprintf(buf, "Input current limit : %d , Charging current limit: %d\n", battery->input_current, battery->charging_current);
}

static ssize_t charger_current_store(struct device *dev,
			struct device_attribute *devattr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct s2mu00x_battery_info *battery = power_supply_get_drvdata(psy);
	int thermal_enable, thermal_fast_charge_percentage;

	sscanf(buf, "%d %d", &thermal_enable, &thermal_fast_charge_percentage);
	pr_err("%s thermal_enable: %d thermal_fast_charge_percentage: %d\n", __func__,
		thermal_enable, thermal_fast_charge_percentage);

	battery->thermal_enable = thermal_enable;
	battery->thermal_fast_charge_percentage = thermal_fast_charge_percentage;

	set_charging_current(battery);


	pr_err("%s \n", __func__);
	return count;
}
DEVICE_ATTR(charger_set, 0664, charger_set_show, charger_set_store);
DEVICE_ATTR(charger_status, 0664, charger_status_show, charger_status_store);
DEVICE_ATTR(charger_current, 0664, charger_current_show, charger_current_store);

#if 0
static struct device_attribute s2mu00x_battery_attrs[] = {
	dev_attr_charger_set,
	dev_attr_charger_status,
};
#endif

	if (!chip->chg_thermal_mitigation) {
		dev_err(chip->dev, "Charge thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported charge level selected %d\n",
			lvl_sel);
		return -EINVAL;
	}

	queue_delayed_work(battery->monitor_wqueue, &battery->soc_control, 10*HZ);
	ret = device_create_file(dev, &dev_attr_charger_status);
	if (ret)
		goto create_attrs_failed;

	ret = device_create_file(dev, &dev_attr_charger_current);
	if (ret)
		goto create_attrs_failed;


	goto create_attrs_succeed;

create_attrs_failed:
	device_remove_file(dev, &dev_attr_charger_set);
	device_remove_file(dev, &dev_attr_charger_status);

#endif
create_attrs_succeed:
	return ret;
}

	if (lvl_sel == chip->chg_therm_lvl_sel)
		return 0;

	pr_info("%s,set thermal level:%d", lvl_sel);

//	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->chg_therm_lvl_sel;
	chip->chg_therm_lvl_sel = lvl_sel;

	chip->allowed_fastchg_current_ma =
		chip->chg_thermal_mitigation[lvl_sel];
	chip->update_allowed_fastchg_current_ma = true;

	cancel_delayed_work(&chip->heartbeat_work);
	schedule_delayed_work(&chip->heartbeat_work,
			      msecs_to_jiffies(0));
//	mutex_unlock(&chip->current_change_lock);
	return rc;
}

//may need add some type
bool is_usb_present(struct s2mu00x_battery_info *chip)
{
	int type = chip->cable_type;
	bool present = false;

	switch (type) {
	case POWER_SUPPLY_TYPE_USB:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_MAINS:
	case POWER_SUPPLY_TYPE_HV_MAINS:
	case POWER_SUPPLY_TYPE_PREPARE_TA:
	case POWER_SUPPLY_TYPE_UNKNOWN:
        case POWER_SUPPLY_TYPE_USB_ACA:              /* Accessory Charger Adapters */
        case POWER_SUPPLY_TYPE_USB_TYPE_C:           /* Type C Port */
        case POWER_SUPPLY_TYPE_USB_PD:               /* Power Delivery Port */
        case POWER_SUPPLY_TYPE_USB_PD_DRP:           /* PD Dual Role Port */
        case POWER_SUPPLY_TYPE_APPLE_BRICK_ID:       /* Apple Charging Method */
		present = true;
		break;
	default:
		present = false;
		break;	
	}

	printk(KERN_ERR "%s,usb_present:%d",__func__,present);

	return present;
}

bool is_cable_present(struct s2mu00x_battery_info *chip)
{
        int type = chip->cable_type;
        bool present = false;

        switch (type) {
        case POWER_SUPPLY_TYPE_UNKNOWN:
        case POWER_SUPPLY_TYPE_BATTERY:
        case POWER_SUPPLY_TYPE_OTG:
        case POWER_SUPPLY_TYPE_END:
                present = false;
                break;
        default:
                present = true;
                break;
        }

        printk(KERN_ERR "%s,cable_present:%d",__func__,present);

        return present;
}

bool is_sdp_cdp(struct s2mu00x_battery_info *chip)
{
        int type = chip->cable_type;

	if (type == POWER_SUPPLY_TYPE_USB || type == POWER_SUPPLY_TYPE_USB_CDP)
		return true;
	else
		return false;
}

bool is_dc_present(struct s2mu00x_battery_info *chip)
{
	int type = chip->cable_type;

	if (type == POWER_SUPPLY_TYPE_USB_DCP || type == POWER_SUPPLY_TYPE_MAINS || type == POWER_SUPPLY_TYPE_HV_MAINS)
		return true;
	else
		return false;
}

static void smbchg_stay_awake(struct s2mu00x_battery_info *chip)
{
	wake_lock(&chip->heartbeat_wake_lock);
}

static void smbchg_relax(struct s2mu00x_battery_info *chip)
{
	wake_unlock(&chip->heartbeat_wake_lock);
};

#define CHG_SHOW_MAX_SIZE 50
static ssize_t factory_charge_upper_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = the_chip->upper_limit_capacity;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_charge_upper, 0444,
		factory_charge_upper_show,
		NULL);

static ssize_t force_demo_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid usb suspend mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	the_chip->stepchg_state_holdoff = 0;

	if ((mode >= 35) && (mode <= 80))
		the_chip->demo_mode = mode;
	else
		the_chip->demo_mode = 35;

	return r ? r : count;
}

static ssize_t force_demo_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = the_chip->demo_mode;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static ssize_t factory_image_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid factory image mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	the_chip->is_factory_image = (mode) ? true : false;

	return r ? r : count;
}

static ssize_t factory_image_mode_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = (the_chip->is_factory_image) ? 1 : 0;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static ssize_t force_max_chrg_temp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid max temp value = %lu\n", mode);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if ((mode >= MIN_MAX_TEMP_C) && (mode <= MAX_TEMP_C))
		the_chip->max_chrg_temp = mode;
	else
		the_chip->max_chrg_temp = MAX_TEMP_C;

	return r ? r : count;
}

static ssize_t force_max_chrg_temp_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int state;

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	state = the_chip->max_chrg_temp;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(factory_image_mode, 0644,
		factory_image_mode_show,
		factory_image_mode_store);

static DEVICE_ATTR(force_demo_mode, 0644,
		force_demo_mode_show,
		force_demo_mode_store);

static DEVICE_ATTR(force_max_chrg_temp, 0644,
		force_max_chrg_temp_show,
		force_max_chrg_temp_store);

static ssize_t force_chg_usb_suspend_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid usb suspend mode value = %lu\n", mode);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}
	smbchg_usb_en(the_chip, false);

	return r ? r : count;
}

static ssize_t force_chg_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if (the_chip->status == POWER_SUPPLY_STATUS_NOT_CHARGING)
		state = 1;
	else
		state = 0;

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_usb_suspend, 0664,
		force_chg_usb_suspend_show,
		force_chg_usb_suspend_store);

static ssize_t force_chg_fail_clear_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid chg fail mode value = %lu\n", mode);
		return -EINVAL;
	}

	return r ? r : count;
}

static ssize_t force_chg_fail_clear_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* do nothing for SMBCHG */
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0\n");
}

static DEVICE_ATTR(force_chg_fail_clear, 0664,
		force_chg_fail_clear_show,
		force_chg_fail_clear_store);

static ssize_t force_chg_auto_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid chrg enable value = %lu\n", mode);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if (mode == 0)
		set_battery_status(the_chip, POWER_SUPPLY_STATUS_DISCHARGING);
	else
		set_battery_status(the_chip, POWER_SUPPLY_STATUS_CHARGING);

	return r ? r : count;
}

static ssize_t force_chg_auto_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;

	if (!the_chip) {
		pr_err("chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	if (the_chip->status == POWER_SUPPLY_STATUS_CHARGING)
		state = 1;
	else
		state = 0 ;

end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_auto_enable, 0664,
		force_chg_auto_enable_show,
		force_chg_auto_enable_store);

static ssize_t force_chg_ibatt_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long chg_current;

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		pr_err("Invalid ibatt value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	set_property_on_charger(the_chip, POWER_SUPPLY_PROP_CURRENT_NOW, chg_current);

	return r ? r : count;
}

static ssize_t force_chg_ibatt_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;

	get_property_from_charger(the_chip, POWER_SUPPLY_PROP_CURRENT_NOW, &state);

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_ibatt, 0664,
		force_chg_ibatt_show,
		force_chg_ibatt_store);

static int smbchg_set_high_usb_chg_current_fac(struct s2mu00x_battery_info *chip,
					       int current_ma)
{
	set_property_on_charger(the_chip, POWER_SUPPLY_PROP_CURRENT_MAX, current_ma);

	return 1;
}

static ssize_t force_chg_iusb_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long usb_curr;

	r = kstrtoul(buf, 0, &usb_curr);
	if (r) {
		pr_err("Invalid iusb value = %lu\n", usb_curr);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	r = smbchg_set_high_usb_chg_current_fac(the_chip,
						usb_curr);
	if (r < 0) {
		pr_info("Couldn't set USBIN Current = %d r = %d\n",
			(int)usb_curr, (int)r);
		return r;
	}
	return r ? r : count;
}

static ssize_t force_chg_iusb_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int state;
	int ret;

	if (!the_chip) {
		pr_err("chip not valid\n");
		ret = -ENODEV;
		goto end;
	}

	get_property_from_charger(the_chip, POWER_SUPPLY_PROP_CURRENT_MAX, &state);

end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_iusb, 0664,
		force_chg_iusb_show,
		force_chg_iusb_store);


#define PRECHG_OFFSET 100
#define PRECHG_STEP 50
#define PRECHG_TOP 250
#define PRECHG_REG_SHIFT 5
#define PRECHG_MASK 0x7
#define PRECHG_CFG 0xF1
#define PRECHG_MAX 550
#define PRECHG_MAX_LVL 0x4
static ssize_t force_chg_itrick_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#if 0
	unsigned long r;
	unsigned long chg_current;
	int i;

	r = kstrtoul(buf, 0, &chg_current);
	if (r) {
		pr_err("Invalid pre-charge value = %lu\n", chg_current);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if (chg_current >= PRECHG_MAX) {
		i = PRECHG_MAX_LVL;
		goto prechg_write;
	}

	for (i = PRECHG_TOP; i > PRECHG_OFFSET; i = i - PRECHG_STEP) {
		if (chg_current >= i)
			break;
	}

	i = (i - PRECHG_OFFSET) / PRECHG_STEP;

	i = i & PRECHG_MASK;

prechg_write:
	r = smbchg_sec_masked_write_fac(the_chip,
					the_chip->chgr_base + PRECHG_CFG,
					PRECHG_MASK, i);
	if (r < 0) {
		dev_err(the_chip->dev,
			"Couldn't set Pre-Charge Current = %d r = %d\n",
			(int)chg_current, (int)r);
		return r;
	}


	return 1;
#else
	return count;
#endif
}

static ssize_t force_chg_itrick_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int state;
#if 0
	int ret;
	u8 value;

	if (!the_chip) {
		pr_err("chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	ret = smbchg_read(the_chip,
			  &value, the_chip->chgr_base + PRECHG_CFG, 1);
	if (ret) {
		pr_err("Pre-Charge Current failed ret = %d\n", ret);
		state = -EFAULT;
		goto end;
	}

	state = value & PRECHG_MASK;

	if (state >= PRECHG_MAX_LVL)
		state = PRECHG_MAX;
	else
		state = (state * PRECHG_STEP) + PRECHG_OFFSET;
#else
	state = 50;//fixed value at samsung.
#endif

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_itrick, 0664,
		   force_chg_itrick_show,
		   force_chg_itrick_store);

static ssize_t force_chg_usb_otg_ctl_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	int value;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid otg ctl value = %lu\n", mode);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("chip not valid\n");
		return -ENODEV;
	}

	if (mode)
		value = 1;
	else
		value = 0;

	set_property_on_otg(the_chip, POWER_SUPPLY_PROP_ONLINE, value);

	return r ? r : count;
}

static ssize_t force_chg_usb_otg_ctl_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int state;
	int value;

	if (!the_chip) {
		pr_err("chip not valid\n");
		state = -ENODEV;
		goto end;
	}

	get_property_from_otg(the_chip, POWER_SUPPLY_PROP_ONLINE, &value);

end:
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", state);
}

static DEVICE_ATTR(force_chg_usb_otg_ctl, 0664,
		   force_chg_usb_otg_ctl_show,
		   force_chg_usb_otg_ctl_store);

static bool smbchg_is_max_thermal_level(struct s2mu00x_battery_info *chip)
{
	if ((chip->chg_thermal_levels == 0) ||
	    ((chip->chg_thermal_levels > 0) &&
	     ((chip->usb_present) &&
	      ((chip->chg_therm_lvl_sel >= (chip->chg_thermal_levels - 1)) ||
	       (chip->chg_therm_lvl_sel == -EINVAL)))))
		return true;
	else if ((chip->dc_thermal_levels == 0) ||
		 ((chip->dc_thermal_levels > 0) &&
		  ((chip->dc_present) &&
		   ((chip->dc_therm_lvl_sel >=
		     (chip->dc_thermal_levels - 1)) ||
		    (chip->dc_therm_lvl_sel == -EINVAL)))))
		return true;
	else
		return false;
}

static int smbchg_check_temp_range(struct s2mu00x_battery_info *chip,
				   int batt_volt,
				   int batt_soc,
				   int batt_health,
				   int prev_batt_health)
{
	int ext_high_temp = 0;

	if (((batt_health == POWER_SUPPLY_HEALTH_COOL) ||
	    ((batt_health == POWER_SUPPLY_HEALTH_WARM)
	    && (smbchg_is_max_thermal_level(chip))))
	    && (batt_volt > chip->ext_temp_volt_mv))
		ext_high_temp = 1;

	if ((((prev_batt_health == POWER_SUPPLY_HEALTH_COOL) &&
	    (batt_health == POWER_SUPPLY_HEALTH_COOL)) ||
	    ((prev_batt_health == POWER_SUPPLY_HEALTH_WARM) &&
	    (batt_health == POWER_SUPPLY_HEALTH_WARM))) &&
	    !chip->ext_high_temp)
		ext_high_temp = 0;

	if (chip->ext_high_temp != ext_high_temp) {
		chip->ext_high_temp = ext_high_temp;
		pr_info("Ext High = %s\n",
			chip->ext_high_temp ? "High" : "Low");

		return 1;
	}

	return 0;
}

static void smbchg_check_temp_state(struct s2mu00x_battery_info *chip, int batt_temp)
{
	int hotspot;
	int temp_state = POWER_SUPPLY_HEALTH_GOOD;
	int max_temp = 0;

	if (!chip)
		return;

	if (chip->max_chrg_temp >= MIN_MAX_TEMP_C)
		max_temp = chip->max_chrg_temp;
	else
		max_temp = chip->hot_temp_c;

	mutex_lock(&chip->check_temp_lock);

	/* Convert to Degrees C */
	hotspot = chip->hotspot_temp / 1000;

	/* Override batt_temp if battery hot spot condition
	   is active */
	if ((batt_temp > chip->cool_temp_c) &&
	    (hotspot > batt_temp) &&
	    (hotspot >= chip->hotspot_thrs_c)) {
		batt_temp = hotspot;
	}

	if (chip->temp_state == POWER_SUPPLY_HEALTH_WARM) {
		if (batt_temp >= max_temp)
			/* Warm to Hot */
			temp_state = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (batt_temp <=
			 chip->warm_temp_c - HYSTERISIS_DEGC)
			/* Warm to Normal */
			temp_state = POWER_SUPPLY_HEALTH_GOOD;
		else
			/* Stay Warm */
			temp_state = POWER_SUPPLY_HEALTH_WARM;
	} else if ((chip->temp_state == POWER_SUPPLY_HEALTH_GOOD) ||
		   (chip->temp_state == POWER_SUPPLY_HEALTH_UNKNOWN)) {
		if (batt_temp >= chip->warm_temp_c)
			/* Normal to Warm */
			temp_state = POWER_SUPPLY_HEALTH_WARM;
		else if (batt_temp <= chip->slightly_cool_temp_c)
			/* Normal to slightly Cool */
			temp_state = POWER_SUPPLY_HEALTH_SLIGHTLY_COOL;
		else
			/* Stay Normal */
			temp_state = POWER_SUPPLY_HEALTH_GOOD;
	} else if (chip->temp_state == POWER_SUPPLY_HEALTH_SLIGHTLY_COOL) {
		if (batt_temp >=
		    chip->slightly_cool_temp_c + HYSTERISIS_DEGC)
			/* Slightly Cool to Normal */
			temp_state = POWER_SUPPLY_HEALTH_GOOD;
		else if (batt_temp <= chip->cool_temp_c)
			/*Slightly Cool  to Cool */
			temp_state = POWER_SUPPLY_HEALTH_COOL;
		else
			/* Stay Slightly Cool  */
			temp_state = POWER_SUPPLY_HEALTH_SLIGHTLY_COOL;
	} else if (chip->temp_state == POWER_SUPPLY_HEALTH_COOL) {
		if (batt_temp >=
		    chip->cool_temp_c + HYSTERISIS_DEGC)
			/*Cool to Slightly Cool  */
			temp_state = POWER_SUPPLY_HEALTH_SLIGHTLY_COOL;
		else if (batt_temp <= chip->cold_temp_c)
			/* Cool to Cold */
			temp_state = POWER_SUPPLY_HEALTH_COLD;
		else
			/* Stay Cool */
			temp_state = POWER_SUPPLY_HEALTH_COOL;
	} else if (chip->temp_state == POWER_SUPPLY_HEALTH_COLD) {
		if (batt_temp >=
		    chip->cold_temp_c + HYSTERISIS_DEGC)
			/* Cold to Cool */
			temp_state = POWER_SUPPLY_HEALTH_COOL;
		else
			/* Stay Cold */
			temp_state = POWER_SUPPLY_HEALTH_COLD;
	} else if (chip->temp_state == POWER_SUPPLY_HEALTH_OVERHEAT) {
		if (batt_temp <= max_temp - HYSTERISIS_DEGC)
			/* Hot to Warm */
			temp_state = POWER_SUPPLY_HEALTH_WARM;
		else
			/* Stay Hot */
			temp_state = POWER_SUPPLY_HEALTH_OVERHEAT;
	}

	if (chip->temp_state != temp_state) {
		chip->temp_state = temp_state;
		pr_info("Battery Temp State = %s\n",
			smb_health_text[chip->temp_state]);
	}
	mutex_unlock(&chip->check_temp_lock);

	return;
}


#define DEMO_MODE_VOLTAGE 4000
static void smbchg_set_temp_chgpath(struct s2mu00x_battery_info *chip, int prev_temp)
{
	if (chip->factory_mode)
		return;

	if (chip->demo_mode)
		set_property_on_charger(chip, POWER_SUPPLY_PROP_VOLTAGE_MAX, DEMO_MODE_VOLTAGE);
	else if (((chip->temp_state == POWER_SUPPLY_HEALTH_COOL)
		  || (chip->temp_state == POWER_SUPPLY_HEALTH_WARM))
		 && !chip->ext_high_temp)
		set_property_on_charger(chip, POWER_SUPPLY_PROP_VOLTAGE_MAX, chip->ext_temp_volt_mv);
	else {
		set_property_on_charger(chip, POWER_SUPPLY_PROP_VOLTAGE_MAX, chip->vfloat_mv);
	}

	if ((chip->temp_state == POWER_SUPPLY_HEALTH_COOL) ||
		(chip->temp_state == POWER_SUPPLY_HEALTH_COLD))
		chip->temp_allowed_fastchg_current_ma =
			chip->temp_cool_current_ma;
	else if ((chip->temp_state == POWER_SUPPLY_HEALTH_WARM) ||
		(chip->temp_state == POWER_SUPPLY_HEALTH_OVERHEAT))
		chip->temp_allowed_fastchg_current_ma =
			chip->temp_warm_current_ma;
	else if (chip->temp_state == POWER_SUPPLY_HEALTH_SLIGHTLY_COOL)
		chip->temp_allowed_fastchg_current_ma =
			chip->temp_slightly_cool_current_ma;
	else if (chip->temp_state == POWER_SUPPLY_HEALTH_GOOD)
		chip->temp_allowed_fastchg_current_ma =
			chip->temp_good_current_ma;

	if (chip->ext_high_temp ||
	    (chip->temp_state == POWER_SUPPLY_HEALTH_COLD) ||
	    (chip->temp_state == POWER_SUPPLY_HEALTH_OVERHEAT) ||
	    (chip->stepchg_state == STEP_FULL))
		smbchg_charging_en(chip, 0);
	else {
		if (((prev_temp == POWER_SUPPLY_HEALTH_COOL) ||
		    (prev_temp == POWER_SUPPLY_HEALTH_WARM)) &&
		    (chip->temp_state == POWER_SUPPLY_HEALTH_GOOD ||
		    chip->temp_state == POWER_SUPPLY_HEALTH_SLIGHTLY_COOL)) {
			smbchg_charging_en(chip, 0);
			mdelay(10);
		}
		smbchg_charging_en(chip, 1);
	}
}

static int smbchg_charging_en(struct s2mu00x_battery_info *chip, bool en)
{
	bool is_charging = S2MU00X_BAT_CHG_MODE_CHARGING_OFF;

	if ((chip->charging_limit_modes == CHARGING_LIMIT_RUN)
		&& (chip->enable_charging_limit)
		&& (chip->is_factory_image))
		en = 0;

	if (en == true) {
		if ((!chip->ext_high_temp) &&
		(chip->temp_state != POWER_SUPPLY_HEALTH_COLD) &&
		(chip->temp_state != POWER_SUPPLY_HEALTH_OVERHEAT) &&
		(chip->stepchg_state != STEP_FULL))
			is_charging = S2MU00X_BAT_CHG_MODE_CHARGING;
		else {
			pr_info("Enable conflict! ext_high_temp: %d,temp_state: %d,step_chg_state %d\n",
				chip->ext_high_temp, chip->temp_state,
				chip->stepchg_state);
			return -EINVAL;
		}
	}

		/*
	 * set_charger_mode(): charger_mode must have one of following values.
	 * 1. S2MU00X_BAT_CHG_MODE_CHARGING
	 *	Charger on.
	 *	Supply power to system & battery both.
	 * 2. S2MU00X_BAT_CHG_MODE_CHARGING_OFF
	 *	Buck mode. Stop battery charging.
	 *	But charger supplies system power.
	 * 3. S2MU00X_BAT_CHG_MODE_BUCK_OFF
	 *	All off. Charger is completely off.
	 *	Do not supply power to battery & system both.
	 */
	pr_info("%s, mode:%d", __func__, is_charging);
	set_charger_mode(chip, is_charging);

	return 1;
}

void update_charging_limit_modes(struct s2mu00x_battery_info *chip,
		int batt_soc)
{
	enum charging_limit_modes charging_limit_modes
						= chip->charging_limit_modes;


	if ((charging_limit_modes != CHARGING_LIMIT_RUN)
		&& (batt_soc >= chip->upper_limit_capacity)) {
		charging_limit_modes = CHARGING_LIMIT_RUN;
	} else if ((charging_limit_modes != CHARGING_LIMIT_OFF)
			&& (batt_soc <= chip->lower_limit_capacity)) {
		charging_limit_modes = CHARGING_LIMIT_OFF;
	}

	pr_info("%s: charging_limit_modes:%d\n", __func__, chip->charging_limit_modes);

	if (charging_limit_modes != chip->charging_limit_modes) {
		chip->charging_limit_modes = charging_limit_modes;

		if (charging_limit_modes == CHARGING_LIMIT_RUN)
			smbchg_charging_en(chip, 0);
		else
			smbchg_charging_en(chip, 1);
	}
}

static int set_property_on_otg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->otg_psy)
		chip->otg_psy =
			power_supply_get_by_name("otg");
	if (!chip->otg_psy) {
		pr_smb(PR_STATUS, "no otg psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = power_supply_set_property(chip->otg_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"otg psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_otg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->otg_psy)
		chip->otg_psy =
			power_supply_get_by_name("otg");
	if (!chip->otg_psy) {
		pr_smb(PR_STATUS, "no otg psy found\n");
		return -EINVAL;
	}

	rc = power_supply_get_property(chip->otg_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"otg psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

static int set_property_on_charger(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->charger_psy && chip->pdata->charger_name)
		chip->charger_psy =
			power_supply_get_by_name((char *)chip->pdata->charger_name);
	if (!chip->charger_psy) {
		pr_smb(PR_STATUS, "no charger psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = power_supply_set_property(chip->charger_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"charger psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}
static int get_property_from_charger(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->charger_psy && chip->pdata->charger_name)
		chip->charger_psy =
			power_supply_get_by_name((char *)chip->pdata->charger_name);
	if (!chip->charger_psy) {
		pr_smb(PR_STATUS, "no charger psy found\n");
		return -EINVAL;
	}

	rc = power_supply_get_property(chip->charger_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"charger psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#if 0
static int set_property_on_fg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->pdata->fuelgauge_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->pdata->fuelgauge_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = power_supply_set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}
#endif

static int get_property_from_fg(struct s2mu00x_battery_info *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->pdata->fuelgauge_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->pdata->fuelgauge_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}
	rc = power_supply_get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct s2mu00x_battery_info *chip)
{
	int capacity, rc;

	if (chip->test_mode && !(chip->test_mode_soc < 0)
	    && !(chip->test_mode_soc > 100))
		return chip->test_mode_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}
	return capacity;
}

#define DEFAULT_BATT_TEMP		200
#define GLITCH_BATT_TEMP		600
#define ERROR_BATT_TEMP 		597
static int get_prop_batt_temp(struct s2mu00x_battery_info *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}

	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct s2mu00x_battery_info *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct s2mu00x_battery_info *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}

//true -keep sys on, charging depend on is_charging.
//false-suspend usb.
static int smbchg_usb_en(struct s2mu00x_battery_info *chip, bool enable)
{
	if (enable)
		set_battery_status(chip, POWER_SUPPLY_STATUS_CHARGING);
	else
		set_battery_status(chip, POWER_SUPPLY_STATUS_NOT_CHARGING);

	return 1;
}

static int smbchg_get_pchg_current_map_index(struct s2mu00x_battery_info *chip)
{
	int i;

	for (i = 0; i < chip->pchg_current_map_len; i++) {
		if (chip->target_fastchg_current_ma >=
		    chip->pchg_current_map_data[i].requested) {
			break;
		}
	}

	if (i >= chip->pchg_current_map_len)
		i = (chip->pchg_current_map_len - 1);

	return i;
}
static void set_max_allowed_current_ma(struct s2mu00x_battery_info *chip,
				       int current_ma)
{
	if (!chip->usb_present) {
		pr_smb(PR_STATUS, "NO allowed current, No USB\n");
		chip->target_fastchg_current_ma = current_ma;
		return;
	}

	chip->target_fastchg_current_ma =
		min(current_ma, chip->allowed_fastchg_current_ma);
	chip->target_fastchg_current_ma =
		min(chip->target_fastchg_current_ma,
		chip->temp_allowed_fastchg_current_ma);

	pr_smb(PR_STATUS, "requested=%d: allowed=%d: temp_step=%d: result=%d\n",
	       current_ma, chip->allowed_fastchg_current_ma,
	       chip->temp_allowed_fastchg_current_ma,
	       chip->target_fastchg_current_ma);
}

#define HEARTBEAT_DELAY_MS 30000
#define HEARTBEAT_HOLDOFF_MS 10000
#define STEPCHG_MAX_FV_COMP 60
#define STEPCHG_ONE_FV_COMP 40
#define STEPCHG_FULL_FV_COMP 100
#define STEPCHG_CURR_ADJ 200
#define DEMO_MODE_HYS_SOC 5
#define HYST_STEP_MV 50
static void smbchg_heartbeat_work(struct work_struct *work)
{
	struct s2mu00x_battery_info *chip = container_of(work,
						struct s2mu00x_battery_info,
						heartbeat_work.work);
	int batt_mv;
	int batt_ma;
	int batt_soc;
	int batt_temp;
	int prev_batt_health;
	int prev_ext_lvl;
	int prev_step;
	int index;
	int state;

	if(!is_cable_present(chip)){
		pr_info("%s:cable not inserted\n",__func__);
		return;
	}

	pr_info("%s: start heatbeat\n", __func__);
	//smbchg_stay_awake(chip);
	//dump register of charger
	get_property_from_charger(chip, POWER_SUPPLY_PROP_PRESENT, &state);

	batt_mv = get_prop_batt_voltage_now(chip) ;
	batt_ma = get_prop_batt_current_now(chip) / 1000;
	batt_soc = get_prop_batt_capacity(chip);

	batt_temp = get_prop_batt_temp(chip) / 10;
	pr_info("%s, batt=%d mV, %d mA, %d C ,soc = %d\n",
		__func__, batt_mv, batt_ma, batt_temp, batt_soc);

	if ((chip->enable_charging_limit) && (chip->is_factory_image))
		update_charging_limit_modes(chip, batt_soc);

	prev_step = chip->stepchg_state;

	if (chip->demo_mode) {
		static int demo_full_soc = 100;
		bool voltage_full = false;

		if (batt_ma < 0)
			batt_ma *= -1;
		if (/*(!!!(chip->usb_suspended & REASON_DEMO)) &&*/
		    ((batt_mv + HYST_STEP_MV) >= DEMO_MODE_VOLTAGE) &&
		    (batt_ma <= chip->stepchg_iterm_ma) &&
		    (chip->allowed_fastchg_current_ma >=
		     chip->stepchg_iterm_ma)) {
			if (chip->stepchg_state_holdoff >= 2) {
				voltage_full = true;
				chip->stepchg_state_holdoff = 0;
			} else
				chip->stepchg_state_holdoff++;
		} else {
			chip->stepchg_state_holdoff = 0;
		}

		chip->stepchg_state = STEP_NONE;
		pr_warn("Battery in Demo Mode charging Limited per%d\n",
			 chip->demo_mode);
		if (/*(!!!(chip->usb_suspended & REASON_DEMO)) &&*/
		    ((batt_soc >= chip->demo_mode) ||
		     voltage_full)) {
			demo_full_soc = batt_soc;
			smbchg_usb_en(chip, false);
			pr_warn("Battery in Demo Mode charging false\n");
		} else if (/*!!(chip->usb_suspended & REASON_DEMO) &&*/
			(batt_soc <=
			 (demo_full_soc - DEMO_MODE_HYS_SOC))) {
			smbchg_usb_en(chip, true);
			chip->stepchg_state_holdoff = 0;
			pr_warn("Battery in Demo Mode charging true\n");
		}
		smbchg_set_temp_chgpath(chip, chip->temp_state);
	} else if ((chip->stepchg_state == STEP_NONE) && (chip->usb_present)) {
		if (batt_mv >= chip->stepchg_voltage_mv)
			chip->stepchg_state = STEP_ONE;
		else
			chip->stepchg_state = STEP_MAX;
		chip->stepchg_state_holdoff = 0;
	} else if ((chip->stepchg_state == STEP_MAX) &&
		    (chip->usb_present) &&
		   ((batt_mv + HYST_STEP_MV) >= chip->stepchg_voltage_mv)) {
			if (batt_ma < 0)
				batt_ma *= -1;

		index = smbchg_get_pchg_current_map_index(chip);
		if (chip->pchg_current_map_data[index].primary ==
		    chip->stepchg_current_ma)
			batt_ma -= STEPCHG_CURR_ADJ;

		if (batt_ma <= min(chip->stepchg_current_ma,
		    chip->allowed_fastchg_current_ma))
			if (chip->stepchg_state_holdoff >= 2) {
					chip->stepchg_state = STEP_ONE;
					chip->stepchg_state_holdoff = 0;
			} else
				chip->stepchg_state_holdoff++;
		else
			chip->stepchg_state_holdoff = 0;
	} else if ((chip->stepchg_state == STEP_ONE) &&
		   (batt_ma < 0) && (chip->usb_present) &&
		   ((batt_mv + HYST_STEP_MV) >=
		    chip->stepchg_max_voltage_mv)) {
		batt_ma *= -1;
		if (batt_ma <= min(chip->stepchg_taper_ma,
		    chip->allowed_fastchg_current_ma))
			if (chip->stepchg_state_holdoff >= 2) {
				chip->stepchg_state = STEP_TAPER;
				chip->stepchg_state_holdoff = 0;
			} else
				chip->stepchg_state_holdoff++;
		else
			chip->stepchg_state_holdoff = 0;
	} else if ((chip->stepchg_state == STEP_TAPER) &&
		   (batt_ma < 0) && (chip->usb_present)) {
		batt_ma *= -1;
		if ((batt_soc >= 100) &&
		    (batt_ma <= chip->stepchg_iterm_ma) &&
		    (chip->allowed_fastchg_current_ma >=
		     chip->stepchg_iterm_ma))
			if (chip->stepchg_state_holdoff >= 2) {
				chip->stepchg_state = STEP_FULL;
				chip->stepchg_state_holdoff = 0;
			} else
				chip->stepchg_state_holdoff++;
		else
			chip->stepchg_state_holdoff = 0;
	}  else if ((chip->stepchg_state == STEP_FULL) &&
		    (chip->usb_present) && (batt_soc < 100)) {
		chip->stepchg_state = STEP_TAPER;
	} else if (!chip->usb_present) {
		chip->stepchg_state = STEP_NONE;
		chip->stepchg_state_holdoff = 0;
	} else
		chip->stepchg_state_holdoff = 0;

	switch (chip->stepchg_state) {
	case STEP_FULL:
	case STEP_TAPER:
		//if (smbchg_hvdcp_det_check(chip) &&
		//    (chip->usb_target_current_ma != HVDCP_ICL_TAPER)) {
		//	mutex_lock(&chip->current_change_lock);
		//	chip->usb_target_current_ma = HVDCP_ICL_TAPER;
		//	mutex_unlock(&chip->current_change_lock);
		//}
		chip->vfloat_mv = chip->stepchg_max_voltage_mv;
		chip->vfloat_parallel_mv =
			chip->stepchg_max_voltage_mv - STEPCHG_FULL_FV_COMP;
		set_max_allowed_current_ma(chip, chip->stepchg_current_ma);
		break;
	case STEP_ONE:
	case STEP_NONE:
		chip->vfloat_mv =
			chip->stepchg_max_voltage_mv;
		chip->vfloat_parallel_mv = chip->stepchg_max_voltage_mv;
		set_max_allowed_current_ma(chip, chip->stepchg_current_ma);
		break;
	case STEP_MAX:
		chip->vfloat_mv =
			chip->stepchg_voltage_mv;
		chip->vfloat_parallel_mv =
			chip->stepchg_voltage_mv + STEPCHG_MAX_FV_COMP;
		set_max_allowed_current_ma(chip, chip->stepchg_max_current_ma);
		break;
	default:
		break;
	}

	pr_info("%s, Step State = %d,vfloat:%d,current:%d\n",
		__func__, (int)chip->stepchg_state, chip->vfloat_mv, chip->target_fastchg_current_ma);

	prev_batt_health = chip->temp_state;
	smbchg_check_temp_state(chip, batt_temp);
	prev_ext_lvl = chip->ext_high_temp;
	smbchg_check_temp_range(chip, batt_mv, batt_soc,
				chip->temp_state, prev_batt_health);

	if ((prev_batt_health != chip->temp_state) ||
	    (prev_ext_lvl != chip->ext_high_temp) ||
	    (prev_step != chip->stepchg_state) ||
	    (chip->update_allowed_fastchg_current_ma)) {
		pr_info("%s, temp state: %d\n",
			__func__, chip->temp_state);
		smbchg_set_temp_chgpath(chip, prev_batt_health);
		if (chip->stepchg_state == STEP_MAX)
			set_max_allowed_current_ma(chip,
				      chip->stepchg_max_current_ma);
		else
			set_max_allowed_current_ma(chip,
				      chip->stepchg_current_ma);

		pr_info("%s, target_fastchg_current_ma:%d\n", __func__, chip->target_fastchg_current_ma);
		if (is_sdp_cdp(chip)) {
			pr_info("%s, usb\n",__func__);
			//need test usb charging.
			chip->update_allowed_fastchg_current_ma = false;
		} else if (is_dc_present(chip)) {
			pr_info("%s, dc,current:%d\n", __func__, chip->target_fastchg_current_ma);
			set_property_on_charger(chip, POWER_SUPPLY_PROP_CURRENT_NOW, chip->target_fastchg_current_ma);
		}
	}

	pr_info("%s, end heartbeat\n", __func__);
	//dump register of charger
	get_property_from_charger(chip, POWER_SUPPLY_PROP_PRESENT, &state);

//end_hb:
	power_supply_changed(chip->psy_battery);

	if (!chip->stepchg_state_holdoff)
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(HEARTBEAT_DELAY_MS));
	else
		schedule_delayed_work(&chip->heartbeat_work,
				      msecs_to_jiffies(HEARTBEAT_HOLDOFF_MS));

//	smbchg_relax(chip);
}

static bool smbchg_charger_mmi_factory(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

static bool qpnp_smbcharger_test_mode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	const char *mode;
	int rc;
	bool test = false;

	if (!np)
		return test;

	rc = of_property_read_string(np, "mmi,battery", &mode);
	if ((rc >= 0) && mode) {
		if (strcmp(mode, "test") == 0)
			test = true;
	}
	of_node_put(np);

	return test;
}
#if 0
static int smbchg_reboot(struct notifier_block *nb,
			 unsigned long event, void *unused)
{
	struct s2mu00x_battery_info *chip =
			container_of(nb, struct s2mu00x_battery_info, smb_reboot);

	dev_dbg(chip->dev, "SMB Reboot\n");
	if (!chip) {
		dev_warn(chip->dev, "called before chip valid!\n");
		return NOTIFY_DONE;
	}

	switch (event) {
	case SYS_POWER_OFF:
		/* Disable Charging */
		smbchg_charging_en(chip, 0);

		/* Suspend USB and DC */
//		smbchg_usb_suspend(chip, true);
//		smbchg_dc_suspend(chip, true);
		smbchg_usb_en(chip, 0);
//		s2mu106_set_buck(chip, 0);

		power_supply_set_present(chip->psy_chg, 0);
		power_supply_set_chg_present(chip->psy_chg, 0);
		power_supply_set_online(chip->psy_chg, 0);

		if (!chip->factory_mode)
			break;

		while (is_usb_present(chip))
			msleep(100);

		dev_warn(chip->dev, "VBUS UV wait 1 sec!\n");
		/* Delay 1 sec to allow more VBUS decay */
		msleep(1000);
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}
#endif

#define DEFAULT_TEST_MODE_SOC  52
#define DEFAULT_TEST_MODE_TEMP  225
static int factory_kill_disable;
module_param(factory_kill_disable, int, 0644);
void factory_usb_shutdown(struct s2mu00x_battery_info *chip)
{

	if (chip->factory_cable) {
		if (!factory_kill_disable) {
			printk(KERN_ERR "SMB - Factory Cable removed, power-off\n");
			kernel_power_off();
		} else
			pr_err("SMB - Factory cable removed - kill disabled\n");
		chip->factory_cable = false;
	}
}
#if 0
static void soc_control_worker(struct work_struct *work)
{
	struct s2mu00x_battery_info *battery =
		container_of(work, struct s2mu00x_battery_info, soc_control.work);

	pr_info("%s: S2MU00x battery capacity = %d, status = %d\n",
		__func__, battery->capacity, battery->status);

	if ((battery->capacity >= 75) && (battery->status == POWER_SUPPLY_STATUS_CHARGING)) {
		pr_info("%s: Capacity is more than 75, stop charging\n", __func__);
		set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
	}

	queue_delayed_work(battery->monitor_wqueue, &battery->soc_control, 10*HZ);
}
#endif
static int s2mu00x_battery_probe(struct platform_device *pdev)
{
	struct s2mu00x_battery_info *battery;
	struct power_supply_config psy_cfg = {};
	union power_supply_propval value;
	int ret = 0, temp = 0;
	struct power_supply *psy;
#ifndef CONFIG_OF
	int i;
#endif
	int rc;

	pr_info("%s: S2MU00x battery driver loading\n", __func__);

	/* Allocate necessary device data structures */
	battery = kzalloc(sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -ENOMEM;

	pr_info("%s: battery is allocated\n", __func__);

	battery->pdata = devm_kzalloc(&pdev->dev, sizeof(*(battery->pdata)),
			GFP_KERNEL);
	if (!battery->pdata) {
		ret = -ENOMEM;
		goto err_bat_free;
	}

	pr_info("%s: pdata is allocated\n", __func__);

	/* Get device/board dependent configuration data from DT */
	temp = s2mu00x_battery_parse_dt(&pdev->dev, battery);
	if (temp) {
		pr_info("%s: s2mu00x_battery_parse_dt(&pdev->dev, battery) == %d\n", __func__, temp);
		pr_info("%s: Failed to get battery dt\n", __func__);
		ret = -EINVAL;
		goto err_parse_dt_nomem;
	}

	pr_info("%s: DT parsing is done\n", __func__);

	/* Set driver data */
	platform_set_drvdata(pdev, battery);
	battery->dev = &pdev->dev;

	mutex_init(&battery->iolock);
	mutex_init(&battery->ifconn_lock);

	wake_lock_init(&battery->monitor_wake_lock, WAKE_LOCK_SUSPEND,
			"sec-battery-monitor");
	wake_lock_init(&battery->vbus_wake_lock, WAKE_LOCK_SUSPEND,
			"sec-battery-vbus");

	/* Inintialization of battery information */
	battery->status = POWER_SUPPLY_STATUS_DISCHARGING;
	battery->health = POWER_SUPPLY_HEALTH_GOOD;

	battery->input_current = 0;
	battery->charging_current = 0;
	battery->topoff_current = 0;
	battery->small_input_flag = 0;

	battery->max_input_current = battery->pdata->max_input_current;
	battery->max_charging_current = battery->pdata->max_charging_current;
#if defined(CONFIG_USE_CCIC)
	battery->pdo_max_input_vol = battery->pdata->pdo_max_input_vol;
	battery->pdo_max_chg_power = battery->pdata->pdo_max_chg_power;
	battery->pd_input_current= 2000;
	battery->pd_attach = false;
#endif
	battery->temp_high = battery->pdata->temp_high;
	battery->temp_high_recovery = battery->pdata->temp_high_recovery;
	battery->temp_low = battery->pdata->temp_low;
	battery->temp_low_recovery = battery->pdata->temp_low_recovery;

	battery->max_rawsoc = battery->pdata->max_rawsoc;

	battery->charging_disabled = false;
	battery->is_recharging = false;
	battery->cable_type = POWER_SUPPLY_TYPE_BATTERY;
	battery->pd_attach = false;

#if defined(CHARGER_S2MU106)
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	if (!value.intval)
		battery->battery_valid = false;
	else
		battery->battery_valid = true;
#else
	battery->battery_valid = true;
#endif
	/* Register battery as "POWER_SUPPLY_TYPE_BATTERY" */
	battery->psy_battery_desc.name = "battery";
	battery->psy_battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	battery->psy_battery_desc.get_property =  s2mu00x_battery_get_property;
	battery->psy_battery_desc.set_property =  s2mu00x_battery_set_property;
	battery->psy_battery_desc.property_is_writeable =  s2mu00x_battery_property_is_writeable;
	battery->psy_battery_desc.properties = s2mu00x_battery_props;
	battery->psy_battery_desc.num_properties =  ARRAY_SIZE(s2mu00x_battery_props);

	battery->psy_usb_desc.name = "usb";
	battery->psy_usb_desc.type = POWER_SUPPLY_TYPE_USB;
	battery->psy_usb_desc.get_property = s2mu00x_usb_get_property;
	battery->psy_usb_desc.properties = s2mu00x_power_props;
	battery->psy_usb_desc.num_properties = ARRAY_SIZE(s2mu00x_power_props);

	battery->psy_ac_desc.name = "ac";
	battery->psy_ac_desc.type = POWER_SUPPLY_TYPE_MAINS;
	battery->psy_ac_desc.properties = s2mu00x_power_props;
	battery->psy_ac_desc.num_properties = ARRAY_SIZE(s2mu00x_power_props);
	battery->psy_ac_desc.get_property = s2mu00x_ac_get_property;

	/* Initialize work queue for periodic polling thread */
	battery->monitor_wqueue =
		create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!battery->monitor_wqueue) {
		dev_err(battery->dev,
				"%s: Fail to Create Workqueue\n", __func__);
		goto err_irr;
	}

	/* Init work & alarm for monitoring */
	INIT_DELAYED_WORK(&battery->monitor_work, bat_monitor_work);
	alarm_init(&battery->monitor_alarm, ALARM_BOOTTIME, bat_monitor_alarm);
	battery->monitor_alarm_interval = DEFAULT_ALARM_INTERVAL;

#if defined(CONFIG_USE_CCIC)
	INIT_DELAYED_WORK(&battery->select_pdo_work, usbpd_select_pdo_work);
#endif
	/* Register power supply to framework */
	psy_cfg.drv_data = battery;
	psy_cfg.supplied_to = s2mu00x_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(s2mu00x_supplied_to);

	battery->psy_battery = power_supply_register(&pdev->dev, &battery->psy_battery_desc, &psy_cfg);
	if (IS_ERR(battery->psy_battery)) {
		pr_err("%s: Failed to Register psy_battery\n", __func__);
		ret = PTR_ERR(battery->psy_battery);
		goto err_workqueue;
	}
	pr_info("%s: Registered battery as power supply\n", __func__);

	battery->psy_usb = power_supply_register(&pdev->dev, &battery->psy_usb_desc, &psy_cfg);
	if (IS_ERR(battery->psy_usb)) {
		pr_err("%s: Failed to Register psy_usb\n", __func__);
		ret = PTR_ERR(battery->psy_usb);
		goto err_unreg_battery;
	}
	pr_info("%s: Registered USB as power supply\n", __func__);

	battery->psy_ac = power_supply_register(&pdev->dev, &battery->psy_ac_desc, &psy_cfg);
	if (IS_ERR(battery->psy_ac)) {
		pr_err("%s: Failed to Register psy_ac\n", __func__);
		ret = PTR_ERR(battery->psy_ac);
		goto err_unreg_usb;
	}
	pr_info("%s: Registered AC as power supply\n", __func__);

	/* Initialize battery level*/
	value.intval = 0;

	psy = power_supply_get_by_name(battery->pdata->fuelgauge_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	battery->capacity = value.intval / 10;

	/* Set float voltage for charger */
	psy = power_supply_get_by_name(battery->pdata->charger_name);
	if (!psy)
		return -EINVAL;
	value.intval = battery->pdata->chg_float_voltage;
	ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

#if 0
#if defined(CONFIG_IFCONN_NOTIFIER)
	ifconn_notifier_register(&battery->ifconn_nb,
			s2mu00x_ifconn_handle_notification,
			IFCONN_NOTIFY_BATTERY,
			IFCONN_NOTIFY_MANAGER);
	ifconn_notifier_register(&battery->ifconn_nb,
			s2mu00x_ifconn_handle_notification,
			IFCONN_NOTIFY_BATTERY,
			IFCONN_NOTIFY_CCIC);
#elif defined(CONFIG_MUIC_NOTIFIER)
	pr_info("%s: Register MUIC notifier\n", __func__);
	muic_notifier_register(&battery->batt_nb, s2mu00x_battery_handle_notification,
			MUIC_NOTIFY_DEV_CHARGER);
#endif
	/* Kick off monitoring thread */
	pr_info("%s: start battery monitoring work\n", __func__);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 5*HZ);
#endif

	if ((is_charging_mode == S2MU00X_POWEROFF_CHG_MODE) ||
		(is_charging_mode == S2MU00X_NOR_MODE)) {
		pr_info("%s: Poweroff charger mode, enable charging\n", __func__);
	//	INIT_DELAYED_WORK(&battery->soc_control, soc_control_worker);
	//	queue_delayed_work(battery->monitor_wqueue, &battery->soc_control, 5*HZ);
	} else if (is_charging_mode == S2MU00X_FAC_MODE) {
		pr_info("%s: Factory boot mode, stop charging\n", __func__);
		set_battery_status(battery, POWER_SUPPLY_STATUS_DISCHARGING);
	} else {

	}

	dev_info(battery->dev, "%s: Battery driver is loaded\n", __func__);

	the_chip = battery;
	wake_lock_init(&battery->heartbeat_wake_lock, WAKE_LOCK_SUSPEND,
			"heartbeat suspend");
	INIT_DELAYED_WORK(&battery->heartbeat_work,
			  smbchg_heartbeat_work);

	battery->factory_mode = smbchg_charger_mmi_factory();
	if (battery->factory_mode) {
		pr_err("Entering Factory Mode SMB Writes Disabled\n");
	}
	battery->demo_mode = 0;
//	battery->hvdcp_det_done = false;
	battery->is_factory_image = false;
	battery->charging_limit_modes = CHARGING_LIMIT_UNKNOWN;
	battery->test_mode_soc = DEFAULT_TEST_MODE_SOC;
	battery->test_mode_temp = DEFAULT_TEST_MODE_TEMP;
	battery->test_mode = qpnp_smbcharger_test_mode();
	battery->max_chrg_temp = 0;
	if (battery->test_mode)
		pr_err("Test Mode Enabled\n");
	battery->is_weak_charger = false;
	battery->usb_online = -EINVAL;
	battery->stepchg_state = STEP_NONE;
	battery->charger_rate = POWER_SUPPLY_CHARGE_RATE_NONE;
	rc = device_create_file(battery->dev,
				&dev_attr_force_demo_mode);
	if (rc) {
		pr_err("couldn't create force_demo_mode\n");
		goto unregister_dc_psy;
	}

	rc = device_create_file(battery->dev,
				&dev_attr_factory_image_mode);
	if (rc) {
		pr_err("couldn't create factory_image_mode\n");
		goto unregister_dc_psy;
	}

	rc = device_create_file(battery->dev,
				&dev_attr_factory_charge_upper);
	if (rc) {
		pr_err("couldn't create factory_charge_upper\n");
		goto unregister_dc_psy;
	}

	rc = device_create_file(battery->dev,
				&dev_attr_force_max_chrg_temp);
	if (rc) {
		pr_err("couldn't create force_max_chrg_temp\n");
		goto unregister_dc_psy;
	}

	if (battery->factory_mode) {
		rc = device_create_file(battery->dev,
					&dev_attr_force_chg_usb_suspend);
		if (rc) {
			pr_err("couldn't create force_chg_usb_suspend\n");
			goto unregister_dc_psy;
		}

		rc = device_create_file(battery->dev,
					&dev_attr_force_chg_fail_clear);
		if (rc) {
			pr_err("couldn't create force_chg_fail_clear\n");
			goto unregister_dc_psy;
		}

		rc = device_create_file(battery->dev,
					&dev_attr_force_chg_auto_enable);
		if (rc) {
			pr_err("couldn't create force_chg_auto_enable\n");
			goto unregister_dc_psy;
		}

		rc = device_create_file(battery->dev,
				&dev_attr_force_chg_ibatt);
		if (rc) {
			pr_err("couldn't create force_chg_ibatt\n");
			goto unregister_dc_psy;
		}

		rc = device_create_file(battery->dev,
					&dev_attr_force_chg_iusb);
		if (rc) {
			pr_err("couldn't create force_chg_iusb\n");
			goto unregister_dc_psy;
		}

		rc = device_create_file(battery->dev,
					&dev_attr_force_chg_itrick);
		if (rc) {
			pr_err("couldn't create force_chg_itrick\n");
			goto unregister_dc_psy;
		}

		rc = device_create_file(battery->dev,
				&dev_attr_force_chg_usb_otg_ctl);
		if (rc) {
			pr_err("couldn't create force_chg_usb_otg_ctl\n");
			goto unregister_dc_psy;
		}
	}

	pr_info("%s: moto Battery driver is loaded\n", __func__);

	#if defined(CONFIG_IFCONN_NOTIFIER)
	ifconn_notifier_register(&battery->ifconn_nb,
			s2mu00x_ifconn_handle_notification,
			IFCONN_NOTIFY_BATTERY,
			IFCONN_NOTIFY_MANAGER);
#elif defined(CONFIG_MUIC_NOTIFIER)
	pr_info("%s: Register MUIC notifier\n", __func__);
	muic_notifier_register(&battery->batt_nb, s2mu00x_battery_handle_notification,
			MUIC_NOTIFY_DEV_CHARGER);
#endif
       ifconn_notifier_register(&battery->ifconn_nb,
                        s2mu00x_ifconn_handle_notification,
                        IFCONN_NOTIFY_BATTERY,
                        IFCONN_NOTIFY_CCIC);


	/* Kick off monitoring thread */
	pr_info("%s: start battery monitoring work\n", __func__);
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 5*HZ);

	schedule_delayed_work(&battery->heartbeat_work,
			      msecs_to_jiffies(0));

	return 0;

unregister_dc_psy:
	wake_lock_destroy(&battery->heartbeat_wake_lock);
err_unreg_usb:
	power_supply_unregister(battery->psy_usb);
err_unreg_battery:
	power_supply_unregister(battery->psy_battery);
err_workqueue:
	destroy_workqueue(battery->monitor_wqueue);
err_irr:
	wake_lock_destroy(&battery->monitor_wake_lock);
	wake_lock_destroy(&battery->vbus_wake_lock);
	mutex_destroy(&battery->iolock);
	mutex_destroy(&battery->ifconn_lock);
err_parse_dt_nomem:
	kfree(battery->pdata);
err_bat_free:
	kfree(battery);

	return ret;
}

static int s2mu00x_battery_remove(struct platform_device *pdev)
{
//	struct s2mu00x_battery_info *battery = dev_get_drvdata(pdev);
	if (the_chip == NULL)
		printk(KERN_ERR "%s, the chip null !!", __func__);

	device_remove_file(the_chip->dev,
			   &dev_attr_force_demo_mode);
	device_remove_file(the_chip->dev,
			   &dev_attr_factory_image_mode);
	device_remove_file(the_chip->dev,
			   &dev_attr_factory_charge_upper);
	device_remove_file(the_chip->dev,
			   &dev_attr_force_max_chrg_temp);
	if (the_chip->factory_mode) {
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_usb_suspend);
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_fail_clear);
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_auto_enable);
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_ibatt);
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_iusb);
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_itrick);
		device_remove_file(the_chip->dev,
				   &dev_attr_force_chg_usb_otg_ctl);
	}

	power_supply_unregister(the_chip->psy_battery);
	power_supply_unregister(the_chip->psy_usb);
	power_supply_unregister(the_chip->psy_ac);
//	smbchg_regulator_deinit(chip);
//	wakeup_source_trash(&chip->smbchg_wake_source);

	return 0;
}

#if defined CONFIG_PM
static int s2mu00x_battery_prepare(struct device *dev)
{
	struct s2mu00x_battery_info *battery = dev_get_drvdata(dev);

	alarm_cancel(&battery->monitor_alarm);
	cancel_delayed_work_sync(&battery->monitor_work);
	wake_unlock(&battery->monitor_wake_lock);
	/* If charger is connected, monitoring is required*/
	if (battery->cable_type != POWER_SUPPLY_TYPE_BATTERY) {
		battery->monitor_alarm_interval = SLEEP_ALARM_INTERVAL;
		pr_info("%s: Increase battery monitoring interval -> %d\n",
				__func__, battery->monitor_alarm_interval);
		alarm_start_relative(&battery->monitor_alarm,
				ktime_set(battery->monitor_alarm_interval, 0));
	}
	return 0;
}

static int s2mu00x_battery_suspend(struct device *dev)
{
	return 0;
}

static int s2mu00x_battery_resume(struct device *dev)
{
	return 0;
}

static void s2mu00x_battery_complete(struct device *dev)
{
	struct s2mu00x_battery_info *battery = dev_get_drvdata(dev);
	if (battery->monitor_alarm_interval != DEFAULT_ALARM_INTERVAL) {
		battery->monitor_alarm_interval = DEFAULT_ALARM_INTERVAL;
		pr_info("%s: Recover battery monitoring interval -> %d\n",
			__func__, battery->monitor_alarm_interval);
	}
	alarm_cancel(&battery->monitor_alarm);
	wake_lock(&battery->monitor_wake_lock);
//	wake_lock(&battery->heartbeat_wake_lock);	
	queue_delayed_work(battery->monitor_wqueue, &battery->monitor_work, 0);
}

#else
#define s2mu00x_battery_prepare NULL
#define s2mu00x_battery_suspend NULL
#define s2mu00x_battery_resume NULL
#define s2mu00x_battery_complete NULL
#endif

static const struct dev_pm_ops s2mu00x_battery_pm_ops = {
	.prepare = s2mu00x_battery_prepare,
	.suspend = s2mu00x_battery_suspend,
	.resume = s2mu00x_battery_resume,
	.complete = s2mu00x_battery_complete,
};

static struct platform_driver s2mu00x_battery_driver = {
	.driver         = {
		.name   = "s2mu00x-battery",
		.owner  = THIS_MODULE,
		.pm     = &s2mu00x_battery_pm_ops,
		.of_match_table = s2mu00x_battery_match_table,
	},
	.probe          = s2mu00x_battery_probe,
	.remove     = s2mu00x_battery_remove,
};

static int __init s2mu00x_battery_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&s2mu00x_battery_driver);
	return ret;
}
late_initcall(s2mu00x_battery_init);

static void __exit s2mu00x_battery_exit(void)
{
	platform_driver_unregister(&s2mu00x_battery_driver);
}
module_exit(s2mu00x_battery_exit);

static int __init is_poweroff_charging_mode(char *str)
{
	if (strncmp("charger", str, 7) == 0)
		is_charging_mode = S2MU00X_POWEROFF_CHG_MODE;
	else if (strncmp("factory", str, 7) == 0)
		is_charging_mode = S2MU00X_FAC_MODE;
	else {
		is_charging_mode = S2MU00X_NOR_MODE;
	}

	return 0;
} early_param("androidboot.mode", is_poweroff_charging_mode);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Battery driver for S2MU00x");
