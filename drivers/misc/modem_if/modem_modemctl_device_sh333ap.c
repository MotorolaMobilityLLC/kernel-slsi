/*
 * Copyright (C) 2010 Samsung Electronics.
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

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mcu_ipc.h>
#include <linux/of_gpio.h>
#include <linux/smc.h>
#include <linux/modem_notifier.h>
#ifdef CONFIG_PCI_EXYNOS
#include <linux/exynos-pci-ctrl.h>
#endif
#include <soc/samsung/cal-if.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_link_device_shmem.h"
#include "uart_switch.h"

#define MIF_INIT_TIMEOUT	(15 * HZ)
#define MBREG_MAX_NUM 64

static struct modem_ctl *g_mc;

static ssize_t modem_ctrl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct modem_ctl *mc = dev_get_drvdata(dev);
	u32 reg;

	ret = snprintf(buf, PAGE_SIZE, "Check Mailbox Registers\n");

	reg = mbox_extract_value(MCU_CP, mc->mbx_ap_status,
		mc->sbi_ap_status_mask, mc->sbi_ap_status_pos);
	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "AP2CP_STATUS : %d\n", reg);

	reg = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
		mc->sbi_cp_status_mask, mc->sbi_cp_status_pos);
	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "CP2AP_STATUS : %d\n", reg);

	reg = mbox_extract_value(MCU_CP, mc->mbx_ap_status,
		mc->sbi_pda_active_mask, mc->sbi_pda_active_pos);
	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "PDA_ACTIVE : %d\n", reg);

	reg = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
		mc->sbi_lte_active_mask, mc->sbi_lte_active_pos);
	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "PHONE_ACTIVE : %d\n", reg);

	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "CP2AP_DVFS_REQ : %d\n",
				mbox_get_value(MCU_CP, mc->mbx_perf_req));

	return ret;
}

static ssize_t modem_ctrl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	long ops_num;
	int ret;

	ret = kstrtol(buf, 10, &ops_num);
	if (ret == 0)
		return count;

	switch (ops_num) {
	case 1:
		mif_info("Reset CP (Stop)!!!\n");
		if (cal_cp_status() > 0) {
			mif_err("CP aleady Power on, try reset\n");
			mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
			cal_cp_enable_dump_pc_no_pg();
			cal_cp_reset_assert();
			mbox_sw_reset(MCU_CP);
		}
		break;

	case 2:
		mif_info("Reset CP - Release (Start)!!!\n");
		cal_cp_reset_release();
		break;

	case 3:
		mif_info("force_crash_exit!!!\n");
		mc->ops.modem_force_crash_exit(mc);
		break;
	case 4:
		mif_info("Modem Power OFF!!! - TBD\n");
		break;

	default:
		mif_info("Wrong operation number\n");
		mif_info("1. Modem Reset - (Stop)\n");
		mif_info("2. Modem Reset - (Start)\n");
		mif_info("3. Modem force crash\n");
		mif_info("4. Modem Power OFF\n");
	}

	return count;
}

static DEVICE_ATTR(modem_ctrl, 0644, modem_ctrl_show, modem_ctrl_store);

static irqreturn_t cp_wdt_handler(int irq, void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	struct link_device *ld = get_current_link(mc->iod);
	int new_state;

	mif_disable_irq(&mc->irq_cp_wdt);
	mif_err("%s: ERR! CP_WDOG occurred\n", mc->name);

	if (cp_online(mc))
		modem_notify_event(MODEM_EVENT_WATCHDOG);

	cal_cp_reset_req_clear();

	new_state = STATE_CRASH_EXIT;
	ld->mode = LINK_MODE_ULOAD;

	mif_err("new_state = %s\n", get_cp_state_str(new_state));
	mc->bootd->modem_state_changed(mc->bootd, new_state);
	mc->iod->modem_state_changed(mc->iod, new_state);

	return IRQ_HANDLED;
}

static void cp_active_handler(void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	struct link_device *ld = get_current_link(mc->iod);
	int cp_on = cal_cp_status();
	int cp_active = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
					mc->sbi_lte_active_mask, mc->sbi_lte_active_pos);
	int old_state = mc->phone_state;
	int new_state = mc->phone_state;

	mif_err("old_state:%s cp_on:%d cp_active:%d\n",
		get_cp_state_str(old_state), cp_on, cp_active);

	if (!cp_active) {
		if (cp_on) {
			new_state = STATE_OFFLINE;
			ld->mode = LINK_MODE_OFFLINE;
			complete_all(&mc->off_cmpl);
		} else {
			mif_err("don't care!!!\n");
		}
	}

	if (old_state != new_state) {
		/* corner case */
		if (old_state == STATE_ONLINE)
			modem_notify_event(MODEM_EVENT_EXIT);
		mif_err("new_state = %s\n", get_cp_state_str(new_state));
		mc->bootd->modem_state_changed(mc->bootd, new_state);
		mc->iod->modem_state_changed(mc->iod, new_state);
	}
}

static void dvfs_req_handler(void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	mif_info("CP reqest to change DVFS level!!!!\n");

	schedule_work(&mc->pm_qos_work);
}

static void dvfs_req_handler_cpu(void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	mif_info("CP reqest to change DVFS level!!!!\n");

	schedule_work(&mc->pm_qos_work_cpu);
}

static void dvfs_req_handler_mif(void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	mif_info("CP reqest to change DVFS level!!!!\n");

	schedule_work(&mc->pm_qos_work_mif);
}

static void dvfs_req_handler_int(void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	mif_info("CP reqest to change DVFS level!!!!\n");

	schedule_work(&mc->pm_qos_work_int);
}

#ifdef CONFIG_PCI_EXYNOS
static void pcie_l1ss_disable_handler(void *arg)
{
	struct modem_mbox *mbx = (struct modem_mbox *)arg;
	unsigned int req;

	req = mbox_get_value(MCU_CP, mbx->mbx_cp2ap_pcie_l1ss_disable);

	if (req == 1) {
		exynos_pcie_l1ss_ctrl(0, PCIE_L1SS_CTRL_MODEM_IF);
		mif_err("cp requests pcie l1ss disable\n");
	} else if (req == 0) {
		exynos_pcie_l1ss_ctrl(1, PCIE_L1SS_CTRL_MODEM_IF);
		mif_err("cp requests pcie l1ss enable\n");
	} else {
		mif_err("unsupported request: pcie_l1ss_disable\n");
	}
}
#else
static void pcie_l1ss_disable_handler(void *arg)
{
	mif_err("CONFIG_PCI_EXYNOS is not set!\n");
}
#endif

static int get_hw_rev(struct device_node *np)
{
	int value, cnt, gpio_cnt;
	unsigned int gpio_hw_rev, hw_rev = 0;

	gpio_cnt = of_gpio_count(np);
	if (gpio_cnt < 0) {
		mif_err("failed to get gpio_count from DT(%d)\n", gpio_cnt);
		return gpio_cnt;
	}

	for (cnt = 0; cnt < gpio_cnt; cnt++) {
		gpio_hw_rev = of_get_gpio(np, cnt);
		if (!gpio_is_valid(gpio_hw_rev)) {
			mif_err("gpio_hw_rev%d: Invalied gpio\n", cnt);
			return -EINVAL;
		}

		value = gpio_get_value(gpio_hw_rev);
		hw_rev |= (value & 0x1) << cnt;
	}

	return hw_rev;
}

#ifdef CONFIG_SIM_DETECT
static unsigned int get_sim_socket_detection(struct device_node *np)
{
	unsigned int gpio_ds_det;

	gpio_ds_det = of_get_named_gpio(np, "mif,gpio_ds_det", 0);
	if (!gpio_is_valid(gpio_ds_det)) {
		mif_err("gpio_ds_det: Invalid gpio\n");
		return 0;
	}

	return gpio_get_value(gpio_ds_det);
}
#else
static unsigned int get_sim_socket_detection(struct device_node *np)
{
	unsigned int sim_socket_num = 1;
	mif_dt_read_u32(np, "mif,sim_socket_num", sim_socket_num);
	mif_err("mif,sim_socket_num : %d", sim_socket_num);

	return sim_socket_num - 1;
}
#endif

static int sh333ap_on(struct modem_ctl *mc)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	struct link_device *ld = get_current_link(mc->iod);
	int cp_active = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
			mc->sbi_lte_active_mask, mc->sbi_lte_active_pos);
	int cp_status = mbox_extract_value(MCU_CP, mc->mbx_cp_status,
			mc->sbi_cp_status_mask, mc->sbi_cp_status_pos);
	int ret;
	int i;
	unsigned long int flags;
	int sys_rev, ds_det;
	unsigned int mbx_ap_status;
	unsigned int sbi_ds_det_mask, sbi_ds_det_pos;
	unsigned int sbi_sys_rev_mask, sbi_sys_rev_pos;

	mif_info("+++\n");
	mif_info("cp_active:%d cp_status:%d\n", cp_active, cp_status);

	if (!wake_lock_active(&mc->mc_wake_lock))
		wake_lock(&mc->mc_wake_lock);

	mc->phone_state = STATE_OFFLINE;
	ld->mode = LINK_MODE_OFFLINE;

	for (i = 0; i < MBREG_MAX_NUM; i++)
		mbox_set_value(MCU_CP, i, 0);

	mif_dt_read_u32(np, "mbx_ap2cp_united_status", mbx_ap_status);
	mif_dt_read_u32(np, "sbi_sys_rev_mask", sbi_sys_rev_mask);
	mif_dt_read_u32(np, "sbi_sys_rev_pos", sbi_sys_rev_pos);
	mif_dt_read_u32(np, "sbi_ds_det_mask", sbi_ds_det_mask);
	mif_dt_read_u32(np, "sbi_ds_det_pos", sbi_ds_det_pos);

	spin_lock_irqsave(&mc->ap_status_lock, flags);

	sys_rev = get_hw_rev(np);
	if (sys_rev >= 0) {
		mbox_update_value(MCU_CP, mbx_ap_status, sys_rev,
			sbi_sys_rev_mask, sbi_sys_rev_pos);
	} else {
		mif_err("get_hw_rev() ERROR\n");
	}
	mif_err("System Revision %d\n", sys_rev);

	if (mc->sim_slot_cnt > 0) {
		mif_err("System sim config: %d\n", mc->sim_slot_cnt);
		mbox_update_value(MCU_CP, mbx_ap_status, (mc->sim_slot_cnt)-1,
				sbi_ds_det_mask, sbi_ds_det_pos);
	} else {
		ds_det = get_sim_socket_detection(np);

		if (ds_det >= 0) {
			mbox_update_value(MCU_CP, mbx_ap_status, ds_det,
				sbi_ds_det_mask, sbi_ds_det_pos);
			mif_err("SIM Socket Detection %d\n", ds_det);
		} else {
			mif_err("get_sim_socket_detection() ERROR\n");
		}
	}


	spin_unlock_irqrestore(&mc->ap_status_lock, flags);

	spin_lock_irqsave(&mc->ap_status_lock, flags);
	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
		mc->sbi_pda_active_mask, mc->sbi_pda_active_pos);
	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
		mc->sbi_ap_status_mask, mc->sbi_ap_status_pos);
	spin_unlock_irqrestore(&mc->ap_status_lock, flags);

	if (mc->ap2cp_cfg_ioaddr) {
		mif_info("Before setting AP2CP_CFG:0x%08x\n",
					__raw_readl(mc->ap2cp_cfg_ioaddr));
		__raw_writel(1, mc->ap2cp_cfg_ioaddr);
		ret = __raw_readl(mc->ap2cp_cfg_ioaddr);
		if (ret != 1) {
			mif_err("AP2CP_CFG setting is not correct:%d\n", ret);
			return -1;
		}
		mif_info("AP2CP_CFG is ok:0x%08x\n", ret);
	} else {
		ret = cal_cp_status();
		if (ret) {
			mif_err("CP aleady Init, Just reset release!\n");
			cal_cp_reset_release();
		} else {
			mif_err("CP first Init!\n");
			cal_cp_init();
		}
	}

	mif_info("---\n");
	return 0;
}

static int sh333ap_off(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	int cp_on = cal_cp_status();
	unsigned long timeout = msecs_to_jiffies(3000);
	unsigned long remain;
	mif_info("+++\n");

	if (mc->phone_state == STATE_OFFLINE || cp_on == 0)
		goto exit;

	reinit_completion(&mc->off_cmpl);
	remain = wait_for_completion_timeout(&mc->off_cmpl, timeout);
	if (remain == 0) {
		mif_err("T-I-M-E-O-U-T\n");
		mc->phone_state = STATE_OFFLINE;
		ld->mode = LINK_MODE_OFFLINE;
		mc->bootd->modem_state_changed(mc->iod, STATE_OFFLINE);
		mc->iod->modem_state_changed(mc->iod, STATE_OFFLINE);
	}

exit:
	if (cal_cp_status() > 0) {
		mif_err("CP aleady Power on, try reset\n");
		mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
		cal_cp_enable_dump_pc_no_pg();
		cal_cp_reset_assert();
		cal_cp_reset_release();
		mbox_sw_reset(MCU_CP);
	}

	mif_info("---\n");
	return 0;
}

static int sh333ap_reset(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);

	if (*(unsigned int *)(mc->mdm_data->ipc_base + SHM_CPINFO_DEBUG)
			== 0xDEB)
		return 0;

	mif_err("+++\n");

	if (cp_online(mc))
		modem_notify_event(MODEM_EVENT_RESET);

	if (cal_cp_status() > 0) {
		mif_err("CP aleady Power on, try reset\n");
		mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
		cal_cp_enable_dump_pc_no_pg();
		cal_cp_reset_assert();
		cal_cp_reset_release();
		mbox_sw_reset(MCU_CP);
	}

	mc->phone_state = STATE_OFFLINE;
	ld->mode = LINK_MODE_OFFLINE;

	usleep_range(10000, 11000);

	mif_err("---\n");
	return 0;
}

static int sh333ap_boot_on(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	int cnt = 100;
	unsigned long int flags;
	mif_info("+++\n");

	ld->mode = LINK_MODE_BOOT;

	mc->bootd->modem_state_changed(mc->bootd, STATE_BOOTING);
	mc->iod->modem_state_changed(mc->iod, STATE_BOOTING);

	while (mbox_extract_value(MCU_CP, mc->mbx_cp_status,
		mc->sbi_cp_status_mask, mc->sbi_cp_status_pos) == 0) {
		if (--cnt > 0)
			usleep_range(10000, 20000);
		else {
			mif_err("mbx_cp_status == 0, return -EACCES !!!!!!\n");
			return -EACCES;
		}
	}

	mif_err("mbx_cp_status == 1 ==>  CP booting in Progress...\n");

	mif_disable_irq(&mc->irq_cp_wdt);

	spin_lock_irqsave(&mc->ap_status_lock, flags);
	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
		mc->sbi_ap_status_mask, mc->sbi_ap_status_pos);
	spin_unlock_irqrestore(&mc->ap_status_lock, flags);

	mif_info("---\n");
	return 0;
}

static int sh333ap_boot_off(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	unsigned long remain;
	int err = 0;
	mif_info("+++\n");

	ld->mode = LINK_MODE_IPC;

	cal_cp_disable_dump_pc_no_pg();

	reinit_completion(&ld->init_cmpl);
	remain = wait_for_completion_timeout(&ld->init_cmpl, MIF_INIT_TIMEOUT);
	if (remain == 0) {
		mif_err("T-I-M-E-O-U-T\n");
		err = -EAGAIN;
		goto exit;
	}

	mif_err("Got INT_CMD_PHONE_START from CP. CP booting completion!!!\n");

	mif_enable_irq(&mc->irq_cp_wdt);

	mif_info("---\n");

exit:
	return err;
}

static int sh333ap_boot_done(struct modem_ctl *mc)
{
	mif_info("+++\n");

	if (wake_lock_active(&mc->mc_wake_lock))
		wake_unlock(&mc->mc_wake_lock);

#ifdef CONFIG_UART_SWITCH
	mif_err("Recheck UART direction.\n");
	cp_recheck_uart_dir();
#endif

	mif_info("---\n");
	return 0;
}

static int sh333ap_force_crash_exit(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	mif_err("+++\n");

	/* Make DUMP start */
	ld->force_dump(ld, mc->bootd);

	mif_err("---\n");
	return 0;
}

int modem_force_crash_exit_ext(void)
{
	if (!g_mc) {
		mif_err("g_mc is null\n");
		return -1;
	}

	mif_info("Make forced crash exit\n");
	sh333ap_force_crash_exit(g_mc);

	return 0;
}
EXPORT_SYMBOL(modem_force_crash_exit_ext);

static int sh333ap_dump_reset(struct modem_ctl *mc)
{
	mif_err("+++\n");

	if (!wake_lock_active(&mc->mc_wake_lock))
		wake_lock(&mc->mc_wake_lock);

	if (cal_cp_status() > 0) {
		mif_err("CP aleady Power on, try reset\n");
		mbox_set_interrupt(MCU_CP, mc->int_cp_wakeup);
		cal_cp_enable_dump_pc_no_pg();
		cal_cp_reset_assert();
		cal_cp_reset_release();
		mbox_sw_reset(MCU_CP);
	}

	mif_err("---\n");
	return 0;
}

static int sh333ap_dump_start(struct modem_ctl *mc)
{
	int err, ret;
	struct link_device *ld = get_current_link(mc->bootd);
	int cnt = 100;
	unsigned long int flags;
	mif_err("+++\n");

	if (!ld->dump_start) {
		mif_err("ERR! %s->dump_start not exist\n", ld->name);
		return -EFAULT;
	}

	err = ld->dump_start(ld, mc->bootd);
	if (err)
		return err;

	if (mc->ap2cp_cfg_ioaddr) {
		mif_info("Before setting AP2CP_CFG:0x%08x\n",
					__raw_readl(mc->ap2cp_cfg_ioaddr));
		__raw_writel(1, mc->ap2cp_cfg_ioaddr);
		ret = __raw_readl(mc->ap2cp_cfg_ioaddr);
		if (ret != 1) {
			mif_err("AP2CP_CFG setting is not correct:%d\n", ret);
			return -1;
		}
		mif_info("AP2CP_CFG is ok:0x%08x\n", ret);
	} else {
		cal_cp_reset_release();
	}

	while (mbox_extract_value(MCU_CP, mc->mbx_cp_status,
		mc->sbi_cp_status_mask, mc->sbi_cp_status_pos) == 0) {
		if (--cnt > 0)
			usleep_range(10000, 20000);
		else {
			mif_err("mbx_cp_status == 0, return -EACCES !!!!!!\n");
			return -EFAULT;
		}
	}

	spin_lock_irqsave(&mc->ap_status_lock, flags);
	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
		mc->sbi_ap_status_mask, mc->sbi_ap_status_pos);
	spin_unlock_irqrestore(&mc->ap_status_lock, flags);

	mif_err("---\n");
	return err;
}

static int sh333ap_get_meminfo(struct modem_ctl *mc, unsigned long arg)
{
	struct meminfo mem_info;
	struct modem_data *modem = mc->mdm_data;

	mem_info.base_addr = modem->shmem_base;
	mem_info.size = modem->ipcmem_offset;

	if (copy_to_user((void __user *)arg, &mem_info, sizeof(struct meminfo)))
		return -EFAULT;

	return 0;
}

static int sh333ap_suspend_modemctl(struct modem_ctl *mc)
{
	unsigned long int flags;

	spin_lock_irqsave(&mc->ap_status_lock, flags);
	mbox_update_value(MCU_CP, mc->mbx_ap_status, 0,
		mc->sbi_pda_active_mask, mc->sbi_pda_active_pos);
	spin_unlock_irqrestore(&mc->ap_status_lock, flags);

	mbox_set_interrupt(MCU_CP, mc->int_pda_active);

	return 0;
}

static int sh333ap_resume_modemctl(struct modem_ctl *mc)
{
	unsigned long int flags;

	spin_lock_irqsave(&mc->ap_status_lock, flags);
	mbox_update_value(MCU_CP, mc->mbx_ap_status, 1,
		mc->sbi_pda_active_mask, mc->sbi_pda_active_pos);
	spin_unlock_irqrestore(&mc->ap_status_lock, flags);

	mbox_set_interrupt(MCU_CP, mc->int_pda_active);

	return 0;
}

static void sh333ap_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on = sh333ap_on;
	mc->ops.modem_off = sh333ap_off;
	mc->ops.modem_reset = sh333ap_reset;
	mc->ops.modem_boot_on = sh333ap_boot_on;
	mc->ops.modem_boot_off = sh333ap_boot_off;
	mc->ops.modem_boot_done = sh333ap_boot_done;
	mc->ops.modem_force_crash_exit = sh333ap_force_crash_exit;
	mc->ops.modem_dump_reset = sh333ap_dump_reset;
	mc->ops.modem_dump_start = sh333ap_dump_start;
	mc->ops.modem_get_meminfo = sh333ap_get_meminfo;
	mc->ops.suspend_modem_ctrl = sh333ap_suspend_modemctl;
	mc->ops.resume_modem_ctrl = sh333ap_resume_modemctl;
}

static void sh333ap_get_pdata(struct modem_ctl *mc, struct modem_data *modem)
{
	struct modem_mbox *mbx = modem->mbx;

	mc->int_pda_active = mbx->int_ap2cp_active;
	mc->int_cp_wakeup = mbx->int_ap2cp_wakeup;

	mc->irq_phone_active = mbx->irq_cp2ap_active;

	mc->mbx_ap_status = mbx->mbx_ap2cp_status;
	mc->mbx_cp_status = mbx->mbx_cp2ap_status;

	mc->mbx_perf_req = mbx->mbx_cp2ap_perf_req;
	mc->irq_perf_req = mbx->irq_cp2ap_perf_req;
	mc->mbx_perf_req_cpu = mbx->mbx_cp2ap_perf_req_cpu;
	mc->irq_perf_req_cpu = mbx->irq_cp2ap_perf_req_cpu;
	mc->mbx_perf_req_mif = mbx->mbx_cp2ap_perf_req_mif;
	mc->irq_perf_req_mif = mbx->irq_cp2ap_perf_req_mif;
	mc->mbx_perf_req_int = mbx->mbx_cp2ap_perf_req_int;
	mc->irq_perf_req_int = mbx->irq_cp2ap_perf_req_int;
	mc->irq_cp_wakelock = mbx->irq_cp2ap_wake_lock;

	mc->mbx_sys_rev = mbx->mbx_ap2cp_sys_rev;
	mc->mbx_pmic_rev = mbx->mbx_ap2cp_pmic_rev;
	mc->mbx_pkg_id = mbx->mbx_ap2cp_pkg_id;

	mc->hw_revision = modem->hw_revision;
	mc->package_id = modem->package_id;
	mc->lock_value = modem->lock_value;

	mc->int_uart_noti = mbx->int_ap2cp_uart_noti;

	mc->sbi_wake_lock_mask = mbx->sbi_wake_lock_mask;
	mc->sbi_wake_lock_pos = mbx->sbi_wake_lock_pos;
	mc->sbi_lte_active_mask = mbx->sbi_lte_active_mask;
	mc->sbi_lte_active_pos = mbx->sbi_lte_active_pos;
	mc->sbi_cp_status_mask = mbx->sbi_cp_status_mask;
	mc->sbi_cp_status_pos = mbx->sbi_cp_status_pos;

	mc->sbi_pda_active_mask = mbx->sbi_pda_active_mask;
	mc->sbi_pda_active_pos = mbx->sbi_pda_active_pos;
	mc->sbi_ap_status_mask = mbx->sbi_ap_status_mask;
	mc->sbi_ap_status_pos = mbx->sbi_ap_status_pos;

	mc->sbi_uart_noti_mask = mbx->sbi_uart_noti_mask;
	mc->sbi_uart_noti_pos = mbx->sbi_uart_noti_pos;

}

int init_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	int ret = 0;
	int irq_num;
	struct device_node *np = pdev->dev.of_node;
	unsigned long flags = IRQF_NO_SUSPEND | IRQF_NO_THREAD;
	struct modem_mbox *mbx;

	mif_err("+++\n");

	g_mc = mc;

	sh333ap_get_ops(mc);
	sh333ap_get_pdata(mc, pdata);
	dev_set_drvdata(mc->dev, mc);
	mbx = pdata->mbx;

	/* Init spin_lock for ap2cp status mbx */
	spin_lock_init(&mc->ap_status_lock);

	wake_lock_init(&mc->mc_wake_lock, WAKE_LOCK_SUSPEND, "umts_wake_lock");

	/* Register CP_WDT interrupt handler */
	irq_num = platform_get_irq(pdev, 0);
	mif_init_irq(&mc->irq_cp_wdt, irq_num, "cp_wdt", flags);

	ret = mif_request_irq(&mc->irq_cp_wdt, cp_wdt_handler, mc);
	if (ret)
		return ret;

	irq_set_irq_wake(irq_num, 1);

	/* CP_WDT interrupt must be enabled only after CP booting */
	mc->irq_cp_wdt.active = true;
	mif_disable_irq(&mc->irq_cp_wdt);

	/* Register DVFS_REQ MBOX interrupt handler */
	mbox_request_irq(MCU_CP, mc->irq_perf_req, dvfs_req_handler, mc);
	mif_err("dvfs_req_handler registered\n");

	mbox_request_irq(MCU_CP, mc->irq_perf_req_cpu,
			dvfs_req_handler_cpu, mc);
	mif_err("dvfs_req_handler_cpu registered\n");

	mbox_request_irq(MCU_CP, mc->irq_perf_req_mif,
			dvfs_req_handler_mif, mc);
	mif_err("dvfs_req_handler_mif registered\n");

	mbox_request_irq(MCU_CP, mc->irq_perf_req_int,
			dvfs_req_handler_int, mc);
	mif_err("dvfs_req_handler_int registered\n");

	/* Register PCIE_CTRL MBOX interrupt handler */
	mbox_request_irq(MCU_CP, mbx->irq_cp2ap_pcie_l1ss_disable,
			pcie_l1ss_disable_handler, mbx);
	mif_err("pcie_l1ss_disable_handler registered\n");

	/* Register LTE_ACTIVE MBOX interrupt handler */
	mbox_request_irq(MCU_CP, mc->irq_phone_active, cp_active_handler, mc);
	mif_err("cp_active_handler registered\n");

	/* Register global value for mif_freq */
	g_mc = mc;

	init_completion(&mc->off_cmpl);

	ret = device_create_file(mc->dev, &dev_attr_modem_ctrl);
	if (ret)
		mif_err("can't create modem_ctrl!!!\n");

	/* AP2CP_CFG */
	mif_dt_read_u32_noerr(np, "ap2cp_cfg_addr", mc->ap2cp_cfg_addr);
	if (mc->ap2cp_cfg_addr) {
		mif_info("AP2CP_CFG:0x%08x\n", mc->ap2cp_cfg_addr);
		mc->ap2cp_cfg_ioaddr = devm_ioremap(mc->dev, mc->ap2cp_cfg_addr, SZ_64);
		if (mc->ap2cp_cfg_ioaddr == NULL) {
			mif_err("%s: AP2CP_CFG ioremap failed.\n", __func__);
			return -EACCES;
		}
	}

	mif_err("---\n");
	return 0;
}
