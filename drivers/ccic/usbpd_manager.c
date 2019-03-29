/*
*	USB PD Driver - Device Policy Manager
*/

#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ccic/usbpd.h>
#include <linux/ccic/usbpd-s2mu106.h>
#include <linux/of_gpio.h>

#include <linux/muic/muic.h>
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#if defined(CONFIG_CCIC_NOTIFIER)
#include <linux/ccic/ccic_notifier.h>
#endif

#if (defined CONFIG_CCIC_NOTIFIER || defined CONFIG_DUAL_ROLE_USB_INTF\
	|| CONFIG_IFCONN_NOTIFIER)
#include <linux/ccic/usbpd_ext.h>
#endif

/* switch device header */
#if defined(CONFIG_SWITCH)
#include <linux/switch.h>
#endif /* CONFIG_SWITCH */

#ifdef CONFIG_USB_HOST_NOTIFY
#include <linux/usb_notify.h>
#endif

#include <linux/completion.h>
#if defined(CONFIG_SWITCH)
static struct switch_dev switch_dock = {
	.name = "ccic_dock",
};
#endif

static char DP_Pin_Assignment_Print[7][40] = {
    {"DP_Pin_Assignment_None"},
    {"DP_Pin_Assignment_A"},
    {"DP_Pin_Assignment_B"},
    {"DP_Pin_Assignment_C"},
    {"DP_Pin_Assignment_D"},
    {"DP_Pin_Assignment_E"},
    {"DP_Pin_Assignment_F"},

};

#ifdef CONFIG_IFCONN_NOTIFIER
void select_pdo(int num);
void s2mu106_select_pdo(int num);
void (*fp_select_pdo)(int num);

void s2mu106_select_pdo(int num)
{
	struct usbpd_data *pd_data = pd_noti.pd_data;
	struct usbpd_manager_data *manager = &pd_data->manager;

	if (pd_noti.sink_status.selected_pdo_num == num)
		return;
	else if (num > pd_noti.sink_status.available_pdo_num)
		pd_noti.sink_status.selected_pdo_num = pd_noti.sink_status.available_pdo_num;
	else if (num < 1)
		pd_noti.sink_status.selected_pdo_num = 1;
	else
		pd_noti.sink_status.selected_pdo_num = num;
	pr_info(" %s : PDO(%d) is selected to change\n", __func__, pd_noti.sink_status.selected_pdo_num);

	schedule_delayed_work(&manager->select_pdo_handler, msecs_to_jiffies(50));
}

void select_pdo(int num)
{
	if (fp_select_pdo)
		fp_select_pdo(num);
}
#endif

#ifdef CONFIG_CHECK_CTYPE_SIDE
int usbpd_manager_get_side_check(void)
{
	struct usbpd_data *pd_data = pd_noti.pd_data;
	int ret = 0;

	ret = pd_data->phy_ops.get_side_check(pd_data);

	return ret;
}
#endif
void usbpd_manager_select_pdo_handler(struct work_struct *work)
{
	pr_info("%s: call select pdo handler\n", __func__);

	usbpd_manager_inform_event(pd_noti.pd_data,
							MANAGER_NEW_POWER_SRC);

}

void usbpd_manager_select_pdo_cancel(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct usbpd_manager_data *manager = &pd_data->manager;

	cancel_delayed_work_sync(&manager->select_pdo_handler);
}

void usbpd_manager_start_discover_msg_handler(struct work_struct *work)
{
	struct usbpd_manager_data *manager =
		container_of(work, struct usbpd_manager_data,
										start_discover_msg_handler.work);
	pr_info("%s: call handler\n", __func__);

	mutex_lock(&manager->vdm_mutex);
	if (manager->alt_sended == 0 && manager->vdm_en == 1) {
		usbpd_manager_inform_event(pd_noti.pd_data,
						MANAGER_START_DISCOVER_IDENTITY);
		manager->alt_sended = 1;
	}
	mutex_unlock(&manager->vdm_mutex);
}

void usbpd_manager_start_discover_msg_cancel(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct usbpd_manager_data *manager = &pd_data->manager;

	cancel_delayed_work_sync(&manager->start_discover_msg_handler);
}

void usbpd_manager_send_pr_swap(struct device *dev)
{
	pr_info("%s: call send pr swap msg\n", __func__);

	usbpd_manager_inform_event(pd_noti.pd_data, MANAGER_SEND_PR_SWAP);
}

static void init_source_cap_data(struct usbpd_manager_data *_data)
{
/*	struct usbpd_data *pd_data = manager_to_usbpd(_data);
	int val;						*/
	msg_header_type *msg_header = &_data->pd_data->source_msg_header;
	data_obj_type *data_obj = &_data->pd_data->source_data_obj;

	msg_header->msg_type = USBPD_Source_Capabilities;
/*	pd_data->phy_ops.get_power_role(pd_data, &val);		*/
	msg_header->port_data_role = USBPD_DFP;
	msg_header->spec_revision = 1;
	msg_header->port_power_role = USBPD_SOURCE;
	msg_header->num_data_objs = 1;

	data_obj->power_data_obj.max_current = 500 / 10;
	data_obj->power_data_obj.voltage = 5000 / 50;
	data_obj->power_data_obj.supply = POWER_TYPE_FIXED;
	data_obj->power_data_obj.data_role_swap = 1;
	data_obj->power_data_obj.dual_role_power = 1;
	data_obj->power_data_obj.usb_suspend_support = 1;
	data_obj->power_data_obj.usb_comm_capable = 0;

}

static void init_sink_cap_data(struct usbpd_manager_data *_data)
{
/*	struct usbpd_data *pd_data = manager_to_usbpd(_data);
	int val;						*/
	msg_header_type *msg_header = &_data->pd_data->sink_msg_header;
	data_obj_type *data_obj = _data->pd_data->sink_data_obj;

	msg_header->msg_type = USBPD_Sink_Capabilities;
/*	pd_data->phy_ops.get_power_role(pd_data, &val);		*/
	msg_header->port_data_role = USBPD_UFP;
	msg_header->spec_revision = 1;
	msg_header->port_power_role = USBPD_SINK;
	msg_header->num_data_objs = 1;

	data_obj->power_data_obj_sink.supply_type = POWER_TYPE_FIXED;
	data_obj->power_data_obj_sink.dual_role_power = 1;
	data_obj->power_data_obj_sink.higher_capability = 1;
	data_obj->power_data_obj_sink.externally_powered = 0;
	data_obj->power_data_obj_sink.usb_comm_capable = 0;
	data_obj->power_data_obj_sink.data_role_swap = 1;
	data_obj->power_data_obj_sink.voltage = 5000/50;
	data_obj->power_data_obj_sink.op_current = 3000/10;
#if 0
	(data_obj + 1)->power_data_obj_variable.supply_type = POWER_TYPE_VARIABLE;
	(data_obj + 1)->power_data_obj_variable.max_voltage = _data->sink_cap_max_volt / 50;
	(data_obj + 1)->power_data_obj_variable.min_voltage = 5000 / 50;
	(data_obj + 1)->power_data_obj_variable.max_current = 3000 / 10;
#endif
}

void usbpd_manager_receive_samsung_uvdm_message(struct usbpd_data *pd_data)
{
}

void usbpd_manager_plug_attach(struct device *dev)
{
#if defined(CONFIG_IFCONN_NOTIFIER)
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;
	struct policy_data *policy = &pd_data->policy;
	struct usbpd_manager_data *manager = &pd_data->manager;

	if (policy->send_sink_cap) {
		pd_noti.event = IFCONN_NOTIFY_EVENT_PD_SINK_CAP;
		policy->send_sink_cap = 0;
	} else
		pd_noti.event = IFCONN_NOTIFY_EVENT_PD_SINK;
	manager->template.data = &pd_noti.sink_status;
	ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
		IFCONN_NOTIFY_ID_POWER_STATUS, IFCONN_NOTIFY_EVENT_ATTACH, &pd_noti);
	manager->pd_attached = 1;
#endif
	pr_info("%s: usbpd plug atached\n", __func__);
}

void usbpd_manager_plug_detach(struct device *dev, bool notify)
{
#if defined(CONFIG_IFCONN_NOTIFIER)
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;
	struct usbpd_manager_data *manager = &pd_data->manager;

	if (manager->pd_attached && pdic_data->power_role == PDIC_SINK) {
		ifconn_event_work(pdic_data, IFCONN_NOTIFY_BATTERY,
								IFCONN_NOTIFY_ID_DETACH, 0, 0);
		manager->pd_attached = 0;
	}
#endif

	pr_info("%s: usbpd plug detached\n", __func__);

	usbpd_policy_reset(pd_data, PLUG_DETACHED);
}

void usbpd_manager_acc_detach(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct usbpd_manager_data *manager = &pd_data->manager;

	pr_info("%s\n", __func__);
	if (manager->acc_type != CCIC_DOCK_DETACHED) {
		pr_info("%s: schedule_delayed_work \n", __func__);
		if (manager->acc_type == CCIC_DOCK_HMT)
			schedule_delayed_work(&manager->acc_detach_handler, msecs_to_jiffies(1000));
		else
			schedule_delayed_work(&manager->acc_detach_handler, msecs_to_jiffies(0));
	}
}

int usbpd_manager_command_to_policy(struct device *dev,
		usbpd_manager_command_type command)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct usbpd_manager_data *manager = &pd_data->manager;

	manager->cmd |= command;

	usbpd_kick_policy_work(dev);

	/* TODO: check result
	if (manager->event) {
	 ...
	}
	*/
	return 0;
}

void usbpd_manager_inform_event(struct usbpd_data *pd_data,
		usbpd_manager_event_type event)
{
	struct usbpd_manager_data *manager = &pd_data->manager;
	int ret = 0;

	manager->event = event;

	switch (event) {
	case MANAGER_DISCOVER_IDENTITY_ACKED:
		usbpd_manager_get_identity(pd_data);
		usbpd_manager_command_to_policy(pd_data->dev,
				MANAGER_REQ_VDM_DISCOVER_SVID);
		break;
	case MANAGER_DISCOVER_SVID_ACKED:
		usbpd_manager_get_svids(pd_data);
		usbpd_manager_command_to_policy(pd_data->dev,
				MANAGER_REQ_VDM_DISCOVER_MODE);
		break;
	case MANAGER_DISCOVER_MODE_ACKED:
		ret = usbpd_manager_get_modes(pd_data);
		if (ret == USBPD_DP_SUPPORT)
			usbpd_manager_command_to_policy(pd_data->dev,
								MANAGER_REQ_VDM_ENTER_MODE);
		break;
	case MANAGER_ENTER_MODE_ACKED:
		usbpd_manager_enter_mode(pd_data);
		usbpd_manager_command_to_policy(pd_data->dev,
			MANAGER_REQ_VDM_STATUS_UPDATE);
		break;
	case MANAGER_STATUS_UPDATE_ACKED:
		usbpd_manager_get_status(pd_data);
		usbpd_manager_command_to_policy(pd_data->dev,
			MANAGER_REQ_VDM_DisplayPort_Configure);
		break;
	case MANAGER_DisplayPort_Configure_ACKED:
		usbpd_manager_get_configure(pd_data);
		break;
	case MANAGER_ATTENTION_REQUEST:
		usbpd_manager_get_attention(pd_data);
		break;
	case MANAGER_NEW_POWER_SRC:
		usbpd_manager_command_to_policy(pd_data->dev,
				MANAGER_REQ_NEW_POWER_SRC);
		break;
	case MANAGER_UVDM_SEND_MESSAGE:
		usbpd_manager_command_to_policy(pd_data->dev,
				MANAGER_REQ_UVDM_SEND_MESSAGE);
		break;
	case MANAGER_UVDM_RECEIVE_MESSAGE:
		usbpd_manager_receive_samsung_uvdm_message(pd_data);
		break;
	case MANAGER_START_DISCOVER_IDENTITY:
		usbpd_manager_command_to_policy(pd_data->dev,
					MANAGER_REQ_VDM_DISCOVER_IDENTITY);
		break;
	case MANAGER_SEND_PR_SWAP:
		usbpd_manager_command_to_policy(pd_data->dev,
					MANAGER_REQ_PR_SWAP);
		break;
	default:
		pr_info("%s: not matched event(%d)\n", __func__, event);
	}
}

bool usbpd_manager_vdm_request_enabled(struct usbpd_data *pd_data)
{
	struct usbpd_manager_data *manager = &pd_data->manager;
	/* TODO : checking cable discovering
	   if (pd_data->counter.discover_identity_counter
		   < USBPD_nDiscoverIdentityCount)

	   struct usbpd_manager_data *manager = &pd_data->manager;
	   if (manager->event != MANAGER_DISCOVER_IDENTITY_ACKED
	      || manager->event != MANAGER_DISCOVER_IDENTITY_NAKED)

	   return(1);
	*/

	manager->vdm_en = 1;

	schedule_delayed_work(&manager->start_discover_msg_handler,
											msecs_to_jiffies(50));
	return true;
}

bool usbpd_manager_power_role_swap(struct usbpd_data *pd_data)
{
	struct usbpd_manager_data *manager = &pd_data->manager;

	return manager->power_role_swap;
}

bool usbpd_manager_vconn_source_swap(struct usbpd_data *pd_data)
{
	struct usbpd_manager_data *manager = &pd_data->manager;

	return manager->vconn_source_swap;
}

void usbpd_manager_turn_off_vconn(struct usbpd_data *pd_data)
{
	/* TODO : Turn off vconn */
}

void usbpd_manager_turn_on_source(struct usbpd_data *pd_data)
{
	pr_info("%s: usbpd plug attached\n", __func__);

	/* TODO : Turn on source */
}

void usbpd_manager_turn_off_power_supply(struct usbpd_data *pd_data)
{
	pr_info("%s: usbpd plug detached\n", __func__);

	/* TODO : Turn off power supply */
}

void usbpd_manager_turn_off_power_sink(struct usbpd_data *pd_data)
{
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;

	pr_info("%s: usbpd sink turn off\n", __func__);

	/* TODO : Turn off power sink */
	pd_noti.event = IFCONN_NOTIFY_EVENT_DETACH;
	ifconn_event_work(pdic_data, IFCONN_NOTIFY_BATTERY,
					IFCONN_NOTIFY_ID_ATTACH, IFCONN_NOTIFY_EVENT_DETACH, NULL);
}

bool usbpd_manager_data_role_swap(struct usbpd_data *pd_data)
{
	struct usbpd_manager_data *manager = &pd_data->manager;

	return manager->data_role_swap;
}

int usbpd_manager_register_switch_device(int mode)
{
#ifdef CONFIG_SWITCH
	int ret = 0;
	if (mode) {
		ret = switch_dev_register(&switch_dock);
		if (ret < 0) {
			pr_err("%s: Failed to register dock switch(%d)\n",
			       __func__, ret);
			return -ENODEV;
		}
	} else {
		switch_dev_unregister(&switch_dock);
	}
#endif /* CONFIG_SWITCH */
	return 0;
}

static void usbpd_manager_send_dock_intent(int type)
{
	pr_info("%s: CCIC dock type(%d)\n", __func__, type);
#ifdef CONFIG_SWITCH
	switch_set_state(&switch_dock, type);
#endif /* CONFIG_SWITCH */
}

void usbpd_manager_send_dock_uevent(u32 vid, u32 pid, int state)
{
	char switch_string[32];
	char pd_ids_string[32];

	pr_info("%s: CCIC dock : USBPD_IPS=%04x:%04x SWITCH_STATE=%d\n",
			__func__,
			le16_to_cpu(vid),
			le16_to_cpu(pid),
			state);


	snprintf(switch_string, 32, "SWITCH_STATE=%d", state);
	snprintf(pd_ids_string, 32, "USBPD_IDS=%04x:%04x",
			le16_to_cpu(vid),
			le16_to_cpu(pid));
}

void usbpd_manager_acc_detach_handler(struct work_struct *wk)
{
	struct usbpd_manager_data *manager =
		container_of(wk, struct usbpd_manager_data, acc_detach_handler.work);

	pr_info("%s: ccic dock type %d\n", __func__,
												manager->acc_type);
	if (manager->acc_type != CCIC_DOCK_DETACHED) {
		if (manager->acc_type != CCIC_DOCK_NEW)
			usbpd_manager_send_dock_intent(CCIC_DOCK_DETACHED);
		usbpd_manager_send_dock_uevent(manager->Vendor_ID, manager->Product_ID,
				CCIC_DOCK_DETACHED);
		manager->acc_type = CCIC_DOCK_DETACHED;
		manager->Vendor_ID = 0;
		manager->Product_ID = 0;
		manager->is_samsung_accessory_enter_mode = false;
	}
}

void usbpd_manager_acc_handler_cancel(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct usbpd_manager_data *manager = &pd_data->manager;

	if (manager->acc_type != CCIC_DOCK_DETACHED) {
		pr_info("%s: cancel_delayed_work_sync \n", __func__);
		cancel_delayed_work_sync(&manager->acc_detach_handler);
	}
}

static int usbpd_manager_check_accessory(struct usbpd_manager_data *manager)
{
#if defined(CONFIG_USB_HW_PARAM)
	struct otg_notify *o_notify = get_otg_notify();
#endif
	uint16_t vid = manager->Vendor_ID;
	uint16_t pid = manager->Product_ID;
	uint16_t acc_type = CCIC_DOCK_DETACHED;

	/* detect Gear VR */
	if (manager->acc_type == CCIC_DOCK_DETACHED) {
		if (vid == SAMSUNG_VENDOR_ID) {
			switch (pid) {
			/* GearVR: Reserved GearVR PID+6 */
			case GEARVR_PRODUCT_ID:
			case GEARVR_PRODUCT_ID_1:
			case GEARVR_PRODUCT_ID_2:
			case GEARVR_PRODUCT_ID_3:
			case GEARVR_PRODUCT_ID_4:
			case GEARVR_PRODUCT_ID_5:
				acc_type = CCIC_DOCK_HMT;
				pr_info("%s : Samsung Gear VR connected.\n", __func__);
#if defined(CONFIG_USB_HW_PARAM)
				if (o_notify)
					inc_hw_param(o_notify, USB_CCIC_VR_USE_COUNT);
#endif
				break;
			case DEXDOCK_PRODUCT_ID:
				acc_type = CCIC_DOCK_DEX;
				pr_info("%s : Samsung DEX connected.\n", __func__);
#if defined(CONFIG_USB_HW_PARAM)
				if (o_notify)
					inc_hw_param(o_notify, USB_CCIC_DEX_USE_COUNT);
#endif
				break;
			case HDMI_PRODUCT_ID:
				acc_type = CCIC_DOCK_HDMI;
				pr_info("%s : Samsung HDMI connected.\n", __func__);
				break;
			default:
				acc_type = CCIC_DOCK_NEW;
				pr_info("%s : default device connected.\n", __func__);
				break;
			}
		} else if (vid == SAMSUNG_MPA_VENDOR_ID) {
			switch (pid) {
			case MPA_PRODUCT_ID:
				acc_type = CCIC_DOCK_MPA;
				pr_info("%s : Samsung MPA connected.\n", __func__);
				break;
			default:
				acc_type = CCIC_DOCK_NEW;
				pr_info("%s : default device connected.\n", __func__);
				break;
			}
		}
		manager->acc_type = acc_type;
	} else
		acc_type = manager->acc_type;

	if (acc_type != CCIC_DOCK_NEW)
		usbpd_manager_send_dock_intent(acc_type);

	usbpd_manager_send_dock_uevent(vid, pid, acc_type);
	return 1;
}

/* Ok : 0, NAK: -1 */
int usbpd_manager_get_identity(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	struct usbpd_manager_data *manager = &pd_data->manager;

	manager->Vendor_ID = policy->rx_data_obj[1].id_header_vdo.USB_Vendor_ID;
	manager->Product_ID = policy->rx_data_obj[3].product_vdo.USB_Product_ID;
	manager->Device_Version = policy->rx_data_obj[3].product_vdo.Device_Version;

	pr_info("%s, Vendor_ID : 0x%x, Product_ID : 0x%x, Device Version : 0x%x\n",
			__func__, manager->Vendor_ID, manager->Product_ID, manager->Device_Version);

	if (usbpd_manager_check_accessory(manager))
		pr_info("%s, Samsung Accessory Connected.\n", __func__);

	return 0;
}

/* Ok : 0, NAK: -1 */
int usbpd_manager_get_svids(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	struct usbpd_manager_data *manager = &pd_data->manager;
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;

	manager->SVID_0 = policy->rx_data_obj[1].vdm_svid.svid_0;
	manager->SVID_1 = policy->rx_data_obj[1].vdm_svid.svid_1;

	pr_info("%s, SVID_0 : 0x%x, SVID_1 : 0x%x\n", __func__,
				manager->SVID_0, manager->SVID_1);

	if (manager->SVID_0 == TypeC_DP_SUPPORT) {
#if defined(CONFIG_IFCONN_NOTIFIER)
		if (pdic_data->is_client == CLIENT_ON) {
			ifconn_event_work(pdic_data, IFCONN_NOTIFY_MUIC,
				IFCONN_NOTIFY_ID_ATTACH, IFCONN_NOTIFY_EVENT_DETACH, NULL);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role = DUAL_ROLE_PROP_PR_NONE;
#endif
			ifconn_event_work(pdic_data, IFCONN_NOTIFY_USB,
				IFCONN_NOTIFY_ID_USB, IFCONN_NOTIFY_EVENT_DETACH, NULL);
			pdic_data->is_client = CLIENT_OFF;
		}

		if (pdic_data->is_host == HOST_OFF) {
			/* muic */
			ifconn_event_work(pdic_data, IFCONN_NOTIFY_MUIC,
				IFCONN_NOTIFY_ID_ATTACH, IFCONN_NOTIFY_EVENT_ATTACH, NULL);
			/* otg */
			pdic_data->is_host = HOST_ON;

			ifconn_event_work(pdic_data, IFCONN_NOTIFY_USB,
					IFCONN_NOTIFY_ID_USB,
					IFCONN_NOTIFY_EVENT_USB_ATTACH_DFP, NULL);
		}
#endif
		manager->dp_is_connect = 1;
		/* If you want to support USB SuperSpeed when you connect
		 * Display port dongle, You should change dp_hs_connect depend
		 * on Pin assignment.If DP use 4lane(Pin Assignment C,E,A),
		 * dp_hs_connect is 1. USB can support HS.If DP use 2lane(Pin Assigment B,D,F), dp_hs_connect is 0. USB
		 * can support SS */
		manager->dp_hs_connect = 1;

		/* sub is only used here to pass the Product_ID */
		/* template->sub1 = pd_info->Product_ID; */
		/* USBPD_SEND_DATA_NOTI_DP(DP_CONNECT,
				pd_info->Vendor_ID, &pd_info->Product_ID); */
		ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
				IFCONN_NOTIFY_ID_DP_CONNECT,
				IFCONN_NOTIFY_EVENT_ATTACH, manager);

		ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
				IFCONN_NOTIFY_ID_USB_DP, manager->dp_hs_connect, manager);
	}

	return 0;
}

/* Ok : 0, NAK: -1 */
int usbpd_manager_get_modes(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	struct usbpd_manager_data *manager = &pd_data->manager;
	data_obj_type *pd_obj = &policy->rx_data_obj[1];

	manager->Standard_Vendor_ID = policy->rx_data_obj[0].structured_vdm.svid;

	pr_info("%s, Standard_Vendor_ID = 0x%x\n", __func__,
				manager->Standard_Vendor_ID);

	if (manager->Standard_Vendor_ID == TypeC_DP_SUPPORT &&
			manager->SVID_0 == TypeC_DP_SUPPORT) {
		if (policy->rx_msg_header.num_data_objs > 1) {
			if (((pd_obj->displayport_capabilities.port_capability == num_UFP_D_Capable)
				&& (pd_obj->displayport_capabilities.receptacle_indication == num_USB_TYPE_C_Receptacle))
				|| ((pd_obj->displayport_capabilities.port_capability == num_DFP_D_Capable)
				&& (pd_obj->displayport_capabilities.receptacle_indication == num_USB_TYPE_C_PLUG))) {

				manager->pin_assignment = pd_obj->displayport_capabilities.ufp_d_pin_assignments;
				pr_info("%s, support UFP_D %d\n", __func__, manager->pin_assignment);
			} else if (((pd_obj->displayport_capabilities.port_capability == num_DFP_D_Capable)
				&& (pd_obj->displayport_capabilities.receptacle_indication == num_USB_TYPE_C_Receptacle))
				|| ((pd_obj->displayport_capabilities.port_capability == num_UFP_D_Capable)
				&& (pd_obj->displayport_capabilities.receptacle_indication == num_USB_TYPE_C_PLUG))) {

				manager->pin_assignment = pd_obj->displayport_capabilities.dfp_d_pin_assignments;
				pr_info("%s, support DFP_D %d\n", __func__, manager->pin_assignment);
			} else if (pd_obj->displayport_capabilities.port_capability == num_DFP_D_and_UFP_D_Capable) {
				if (pd_obj->displayport_capabilities.receptacle_indication == num_USB_TYPE_C_PLUG) {

					manager->pin_assignment = pd_obj->displayport_capabilities.dfp_d_pin_assignments;
					pr_info("%s, support DFP_D %d\n", __func__, manager->pin_assignment);
				} else {
					manager->pin_assignment = pd_obj->displayport_capabilities.ufp_d_pin_assignments;
					pr_info("%s, support UFP_D %d\n", __func__, manager->pin_assignment);
				}
			} else {
				manager->pin_assignment = DP_PIN_ASSIGNMENT_NODE;
				pr_info("%s, there is not valid object %d\n", __func__, manager->pin_assignment);
			}
		}

		return USBPD_DP_SUPPORT;
	}

	return USBPD_NOT_DP;
}

int usbpd_manager_enter_mode(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	struct usbpd_manager_data *manager = &pd_data->manager;
	manager->Standard_Vendor_ID = policy->rx_data_obj[0].structured_vdm.svid;
	manager->is_samsung_accessory_enter_mode = true;
	return 0;
}

int usbpd_manager_exit_mode(struct usbpd_data *pd_data, unsigned mode)
{
	return 0;
}

int usbpd_manager_get_status(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;
	struct usbpd_manager_data *manager = &pd_data->manager;
	bool multi_func_preference = 0;
	int pin_assignment = 0;
	data_obj_type *pd_obj = &policy->rx_data_obj[1];

	if (manager->SVID_0 != TypeC_DP_SUPPORT)
		return 0;

	if (pd_obj->displayport_status.port_connected == 0) {
		pr_info("%s, port disconnected!\n", __func__);
	}

	if (manager->is_sent_pin_configuration) {
		pr_info("%s, already sent pin configuration\n", __func__);
	}

	if (pd_obj->displayport_status.port_connected &&
			!manager->is_sent_pin_configuration) {
		multi_func_preference = pd_obj->displayport_status.multi_function_preferred;

		if (multi_func_preference) {
			if (manager->pin_assignment & DP_PIN_ASSIGNMENT_D) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_D;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_B) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_B;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_F) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_F;
			} else {
				pr_info("wrong pin assignment value\n");
			}
		} else {
			if (manager->pin_assignment & DP_PIN_ASSIGNMENT_C) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_C;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_E) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_E;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_A) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_A;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_D) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_D;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_B) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_B;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_F) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_F;
			} else {
				pr_info("wrong pin assignment value\n");
			}
		}
		manager->dp_selected_pin = pin_assignment;

		manager->is_sent_pin_configuration = 1;

		pr_info("%s multi_func_preference %d  %s\n", __func__,
			multi_func_preference, DP_Pin_Assignment_Print[pin_assignment]);
	}

	if (pd_obj->displayport_status.hpd_state)
		manager->hpd = IFCONN_NOTIFY_HIGH;
	else
		manager->hpd = IFCONN_NOTIFY_LOW;

	if (pd_obj->displayport_status.irq_hpd)
		manager->hpdirq = IFCONN_NOTIFY_IRQ;

	ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
					IFCONN_NOTIFY_ID_DP_HPD, manager->hpdirq, manager);

	return 0;
}

int usbpd_manager_get_configure(struct usbpd_data *pd_data)
{
	struct usbpd_manager_data *manager = &pd_data->manager;
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;

	if (manager->SVID_0 == TypeC_DP_SUPPORT)
		ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
						IFCONN_NOTIFY_ID_DP_LINK_CONF,
						IFCONN_NOTIFY_EVENT_ATTACH, manager);

	return 0;
}

int usbpd_manager_get_attention(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;
	struct usbpd_manager_data *manager = &pd_data->manager;
	bool multi_func_preference = 0;
	int pin_assignment = 0;
	data_obj_type *pd_obj = &policy->rx_data_obj[1];

	if (manager->SVID_0 != TypeC_DP_SUPPORT)
		return 0;

	if (pd_obj->displayport_status.port_connected == 0) {
		pr_info("%s, port disconnected!\n", __func__);
	}

	if (manager->is_sent_pin_configuration) {
		pr_info("%s, already sent pin configuration\n", __func__);
	}

	if (pd_obj->displayport_status.port_connected &&
			!manager->is_sent_pin_configuration) {
		multi_func_preference = pd_obj->displayport_status.multi_function_preferred;

		if (multi_func_preference) {
			if (manager->pin_assignment & DP_PIN_ASSIGNMENT_D) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_D;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_B) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_B;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_F) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_F;
			} else {
				pr_info("wrong pin assignment value\n");
			}
		} else {
			if (manager->pin_assignment & DP_PIN_ASSIGNMENT_C) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_C;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_E) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_E;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_A) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_A;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_D) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_D;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_B) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_B;
			} else if (manager->pin_assignment & DP_PIN_ASSIGNMENT_F) {
				pin_assignment = IFCONN_NOTIFY_DP_PIN_F;
			} else {
				pr_info("wrong pin assignment value\n");
			}
		}
		manager->dp_selected_pin = pin_assignment;

		manager->is_sent_pin_configuration = 1;

		pr_info("%s multi_func_preference %d  %s\n", __func__,
			multi_func_preference, DP_Pin_Assignment_Print[pin_assignment]);
	}

	if (pd_obj->displayport_status.hpd_state)
		manager->hpd = IFCONN_NOTIFY_HIGH;
	else
		manager->hpd = IFCONN_NOTIFY_LOW;

	if (pd_obj->displayport_status.irq_hpd)
		manager->hpdirq = IFCONN_NOTIFY_IRQ;

	ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
					IFCONN_NOTIFY_ID_DP_HPD, manager->hpdirq, manager);

	return 0;
}

void usbpd_dp_detach(struct usbpd_data *pd_data)
{
	struct s2mu106_usbpd_data *pdic_data = pd_data->phy_driver_data;
	struct usbpd_manager_data *manager = &pd_data->manager;

	dev_info(pd_data->dev, "%s: dp_is_connect %d\n", __func__, manager->dp_is_connect);

	ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
				IFCONN_NOTIFY_ID_USB_DP, manager->dp_hs_connect, NULL);
	ifconn_event_work(pdic_data, IFCONN_NOTIFY_MANAGER,
				IFCONN_NOTIFY_ID_DP_CONNECT, IFCONN_NOTIFY_EVENT_DETACH, NULL);

	manager->dp_is_connect = 0;
	manager->dp_hs_connect = 0;
	manager->is_sent_pin_configuration = 0;

	return;
}

data_obj_type usbpd_manager_select_capability(struct usbpd_data *pd_data)
{
	/* TODO: Request from present capabilities
		indicate if other capabilities would be required */
	data_obj_type obj;
#ifdef CONFIG_IFCONN_NOTIFIER
	int pdo_num = pd_noti.sink_status.selected_pdo_num;
#endif
	obj.request_data_object.no_usb_suspend = 1;
	obj.request_data_object.usb_comm_capable = 0;
	obj.request_data_object.capability_mismatch = 0;
	obj.request_data_object.give_back = 0;
#ifdef CONFIG_IFCONN_NOTIFIER
	obj.request_data_object.min_current = pd_noti.sink_status.power_list[pdo_num].max_current / USBPD_CURRENT_UNIT;
	obj.request_data_object.op_current = pd_noti.sink_status.power_list[pdo_num].max_current / USBPD_CURRENT_UNIT;
	obj.request_data_object.object_position = pd_noti.sink_status.selected_pdo_num;
#endif

	return obj;
}

/*
   usbpd_manager_evaluate_capability
   : Policy engine ask Device Policy Manager to evaluate option
     based on supplied capabilities
	return	>0	: request object number
		0	: no selected option
*/
int usbpd_manager_evaluate_capability(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
	int i = 0;
	int power_type = 0;
	int pd_volt = 0, pd_current;
#ifdef CONFIG_IFCONN_NOTIFIER
	int available_pdo_num = 0;
	ifconn_pd_sink_status_t *pdic_sink_status = &pd_noti.sink_status;
#endif
	data_obj_type *pd_obj;

	for (i = 0; i < policy->rx_msg_header.num_data_objs; i++) {
		pd_obj = &policy->rx_data_obj[i];
		power_type = pd_obj->power_data_obj_supply_type.supply_type;
		switch (power_type) {
		case POWER_TYPE_FIXED:
			pd_volt = pd_obj->power_data_obj.voltage;
			pd_current = pd_obj->power_data_obj.max_current;
			dev_info(pd_data->dev, "[%d] FIXED volt(%d)mV, max current(%d)\n",
					i+1, pd_volt * USBPD_VOLT_UNIT, pd_current * USBPD_CURRENT_UNIT);
#ifdef CONFIG_IFCONN_NOTIFIER
			if (pd_volt * USBPD_VOLT_UNIT <= MAX_CHARGING_VOLT)
				available_pdo_num = i + 1;
			pdic_sink_status->power_list[i + 1].max_voltage = pd_volt * USBPD_VOLT_UNIT;
			pdic_sink_status->power_list[i + 1].max_current = pd_current * USBPD_CURRENT_UNIT;
#endif
			break;
		case POWER_TYPE_BATTERY:
			pd_volt = pd_obj->power_data_obj_battery.max_voltage;
			dev_info(pd_data->dev, "[%d] BATTERY volt(%d)mV\n",
					i+1, pd_volt * USBPD_VOLT_UNIT);
			break;
		case POWER_TYPE_VARIABLE:
			pd_volt = pd_obj->power_data_obj_variable.max_voltage;
			dev_info(pd_data->dev, "[%d] VARIABLE volt(%d)mV\n",
					i+1, pd_volt * USBPD_VOLT_UNIT);
			break;
		default:
			dev_err(pd_data->dev, "[%d] Power Type Error\n", i+1);
			break;
		}
	}
#ifdef CONFIG_IFCONN_NOTIFIER
	pdic_sink_status->available_pdo_num = available_pdo_num;
	return available_pdo_num;
#else
	return 1; /* select default first obj */
#endif
}

/* return: 0: cab be met, -1: cannot be met, -2: could be met later */
int usbpd_manager_match_request(struct usbpd_data *pd_data)
{
	/* TODO: Evaluation of sink request */

	unsigned supply_type
	= pd_data->source_request_obj.power_data_obj_supply_type.supply_type;
	unsigned src_max_current,  mismatch, max_min, op, pos;

	if (supply_type == POWER_TYPE_FIXED)
		pr_info("REQUEST: FIXED\n");
	else if (supply_type == POWER_TYPE_VARIABLE)
		pr_info("REQUEST: VARIABLE\n");
	else if (supply_type == POWER_TYPE_BATTERY) {
		pr_info("REQUEST: BATTERY\n");
		goto log_battery;
	} else {
		pr_info("REQUEST: UNKNOWN Supply type.\n");
		return -1;
	}

    /* Tx Source PDO */
    src_max_current = pd_data->source_data_obj.power_data_obj.max_current;

    /* Rx Request RDO */
	mismatch = pd_data->source_request_obj.request_data_object.capability_mismatch;
	max_min = pd_data->source_request_obj.request_data_object.min_current;
	op = pd_data->source_request_obj.request_data_object.op_current;
	pos = pd_data->source_request_obj.request_data_object.object_position;

#if 0
	pr_info("%s %x\n", __func__, pd_data->source_request_obj.object);
#endif
    /*src_max_current is already *10 value ex) src_max_current 500mA */
	//pr_info("Tx SourceCap Current : %dmA\n", src_max_current*10);
	//pr_info("Rx Request Current : %dmA\n", max_min*10);

    /* Compare Pdo and Rdo */
    if ((src_max_current >= op) && (pos == 1))
		return 0;
    else
		return -1;

log_battery:
	mismatch = pd_data->source_request_obj.request_data_object_battery.capability_mismatch;
	return 0;
}

#ifdef CONFIG_OF
static int of_usbpd_manager_dt(struct usbpd_manager_data *_data)
{
	int ret = 0;
	struct device_node *np =
		of_find_node_by_name(NULL, "pdic-manager");

	if (np == NULL) {
		pr_err("%s np NULL\n", __func__);
		return -EINVAL;
	} else {
		ret = of_property_read_u32(np, "pdic,max_power",
				&_data->max_power);
		if (ret < 0)
			pr_err("%s error reading max_power %d\n",
					__func__, _data->max_power);

		ret = of_property_read_u32(np, "pdic,op_power",
				&_data->op_power);
		if (ret < 0)
			pr_err("%s error reading op_power %d\n",
					__func__, _data->max_power);

		ret = of_property_read_u32(np, "pdic,max_current",
				&_data->max_current);
		if (ret < 0)
			pr_err("%s error reading max_current %d\n",
					__func__, _data->max_current);

		ret = of_property_read_u32(np, "pdic,min_current",
				&_data->min_current);
		if (ret < 0)
			pr_err("%s error reading min_current %d\n",
					__func__, _data->min_current);

		_data->giveback = of_property_read_bool(np,
						     "pdic,giveback");
		_data->usb_com_capable = of_property_read_bool(np,
						     "pdic,usb_com_capable");
		_data->no_usb_suspend = of_property_read_bool(np,
						     "pdic,no_usb_suspend");

		/* source capability */
		ret = of_property_read_u32(np, "source,max_voltage",
				&_data->source_max_volt);
		ret = of_property_read_u32(np, "source,min_voltage",
				&_data->source_min_volt);
		ret = of_property_read_u32(np, "source,max_power",
				&_data->source_max_power);

		/* sink capability */
		ret = of_property_read_u32(np, "sink,capable_max_voltage",
				&_data->sink_cap_max_volt);
		if (ret < 0) {
			_data->sink_cap_max_volt = 5000;
			pr_err("%s error reading sink_cap_max_volt %d\n",
					__func__, _data->sink_cap_max_volt);
		}
	}

	return ret;
}
#endif

void usbpd_init_manager_val(struct usbpd_data *pd_data)
{
	struct usbpd_manager_data *manager = &pd_data->manager;

	pr_info("%s\n", __func__);
	manager->alt_sended = 0;
	manager->cmd = 0;
	manager->vdm_en = 0;
	manager->Vendor_ID = 0;
	manager->Product_ID = 0;
	manager->Device_Version = 0;
	manager->SVID_0 = 0;
	manager->SVID_1 = 0;
	manager->Standard_Vendor_ID = 0;
	manager->dp_is_connect = 0;
	manager->dp_hs_connect = 0;
	manager->is_sent_pin_configuration = 0;
	manager->pin_assignment = 0;
	manager->dp_selected_pin = 0;
	manager->hpd = 0;
	manager->hpdirq = 0;
	init_completion(&manager->uvdm_out_wait);
	init_completion(&manager->uvdm_in_wait);
	usbpd_manager_select_pdo_cancel(pd_data->dev);
	usbpd_manager_start_discover_msg_cancel(pd_data->dev);
}

int usbpd_init_manager(struct usbpd_data *pd_data)
{
	int ret = 0;
	struct usbpd_manager_data *manager = &pd_data->manager;

	pr_info("%s\n", __func__);
	if (manager == NULL) {
		pr_err("%s, usbpd manager data is error!!\n", __func__);
		return -ENOMEM;
	} else
		ret = of_usbpd_manager_dt(manager);
#ifdef CONFIG_BATTERY_SAMSUNG
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
	fp_select_pdo = s2mu106_select_pdo;
#endif
#endif
	mutex_init(&manager->vdm_mutex);
	manager->pd_data = pd_data;
	manager->pd_attached = 0;
	manager->power_role_swap = true;
	manager->data_role_swap = true;
	manager->vconn_source_swap = false;
	manager->alt_sended = 0;
	manager->vdm_en = 0;
	manager->acc_type = 0;
	manager->Vendor_ID = 0;
	manager->Product_ID = 0;
	manager->Device_Version = 0;
	manager->SVID_0 = 0;
	manager->SVID_1 = 0;
	manager->Standard_Vendor_ID = 0;
	manager->dp_is_connect = 0;
	manager->dp_hs_connect = 0;
	manager->is_sent_pin_configuration = 0;
	manager->pin_assignment = 0;
	manager->dp_selected_pin = 0;
	manager->hpd = 0;
	manager->hpdirq = 0;
	init_completion(&manager->uvdm_out_wait);
	init_completion(&manager->uvdm_in_wait);

	usbpd_manager_register_switch_device(1);
	init_source_cap_data(manager);
	init_sink_cap_data(manager);
	INIT_DELAYED_WORK(&manager->acc_detach_handler, usbpd_manager_acc_detach_handler);
	INIT_DELAYED_WORK(&manager->select_pdo_handler, usbpd_manager_select_pdo_handler);
	INIT_DELAYED_WORK(&manager->start_discover_msg_handler,
									usbpd_manager_start_discover_msg_handler);

	pr_info("%s done\n", __func__);
	return ret;
}
