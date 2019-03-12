/*
*	USB PD Driver - Policy Engine
*/

#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ccic/usbpd.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include <linux/muic/muic.h>
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#include <linux/usb_notify.h>

#if (defined CONFIG_IFCONN_NOTIFIER || defined CONFIG_DUAL_ROLE_USB_INTF)
#include <linux/ccic/usbpd_ext.h>
#endif

#if defined(CONFIG_IFCONN_NOTIFIER)
#include <linux/ifconn/ifconn_notifier.h>
#endif

#define CHECK_MSG(pd, msg, ret) do {\
	if (pd->phy_ops.get_status(pd, msg))\
		return ret;\
	} while (0);

#define CHECK_CMD(pd, event, ret) do {\
		if (pd->manager.cmd & event) {\
			pd->manager.cmd &= ~event; \
			return ret;\
		} \
	} while (0);

policy_state usbpd_policy_src_startup(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Send_Capabilities;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Reset CapsCounter
	Reset Protocol Layer
	Start SwapSourceStartTimer (only after Swap)
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) CapsCounter Reset */
	pd_data->counter.caps_counter = 0;

	/* 3) PD Protocol Initialization */
	usbpd_init_protocol(pd_data);

	/* 4) Fro tSrcrecover after PE_SRC_Transition_to_default */
	if (policy->txhardresetflag == 1) {
		policy->txhardresetflag = 0;

		usbpd_timer1_start(pd_data);
		while (1) {
			if (policy->plug_valid == 0) {
				ret = PE_SRC_Startup;
				break;
			}
			ms = usbpd_check_time1(pd_data);
			if (ms >= 200)
				break;
		}
	}

	/* 5) Configuration Channel On */
	pd_data->phy_ops.set_cc_control(pd_data, USBPD_CC_ON);

	return ret;
}

policy_state usbpd_policy_src_discovery(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Initialize and run SourceCapabilityTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Delay*/
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Discovery;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= tTypeCSendSourceCap)
			break;
	}

	if (ret == PE_SRC_Discovery)
		return ret;
	/* 3) Caps Counter Check */
	if (pd_data->counter.caps_counter <= USBPD_nCapsCount)
		return PE_SRC_Send_Capabilities;
	else
		return PE_SRC_Disabled;
}

policy_state usbpd_policy_src_send_capabilities(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	bool received_goodcrc = 0;
	int ret = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Request present source capabilities from Device Policy Manager
	Send PD Capabilities message
	Increment CapsCounter (optional)
	If GoodCRC received:
	- stop NoResponseTimer
	- reset HardResetCounter and CapsCounter
	- initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Source Capabilities PDO Read & Write */
	policy->tx_msg_header.word = pd_data->source_msg_header.word;
	policy->tx_data_obj[0].object = pd_data->source_data_obj.object;

	/* 3) Interrupt Status Bit Clear  */
	pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC | MSG_REQUEST |
															MSG_GET_SNK_CAP);
	/* 4) Add Caps Counter */
	pd_data->counter.caps_counter++;

	/* 5) Send Message */
	usbpd_send_msg(pd_data, &policy->tx_msg_header, policy->tx_data_obj);

	/* 6) Start Timer */
	usbpd_timer1_start(pd_data);	// Setting 25ms is actual 25 ~ 29ms
	/* 7) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Send_Capabilities;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_REQUEST)) {
			pd_data->counter.hard_reset_counter = 0;
			pd_data->counter.caps_counter = 0;
			pd_data->source_request_obj.object
				= policy->rx_data_obj[0].object;
			dev_info(pd_data->dev, "got Request.\n");
			ret = PE_SRC_Negotiate_Capability;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC))
			received_goodcrc = 1;

		if (policy->rx_hardreset) {
			ret = 0;
			break;
		}

		/* TD.PD.SRC.E14 Atomic Message Sequence */
		if (pd_data->phy_ops.get_status(pd_data, MSG_GET_SNK_CAP)) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Discovery;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_PING)) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}

		if (ms >= tSenderResponse) {
			if (received_goodcrc) {
				ret = PE_SRC_Hard_Reset;

				if (pd_data->counter.hard_reset_counter > USBPD_nHardResetCount)
					ret = Error_Recovery;

				break;
			}
		}

		if (ms >= 100) {
			ret = PE_SRC_Discovery;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_src_negotiate_capability(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Negotiate_Capability;
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Get Device Policy Manager evaluation of sink request:
	- Can be met
	- Can??t be met
	- Could be met later from Power Reserve
	If the sink request for Operating Current or Operating Power can be met,
	but the sink still requires more power Capability Mismatch this
	information will be passed to Device Policy Manager
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	mutex_lock(&pd_data->accept_mutex);
	/* 2) Analysis Received Request Message */
	if (usbpd_manager_match_request(pd_data) == 0) {
		usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Accept,
													data_role, USBPD_SOURCE);
		dev_info(pd_data->dev, "%s sended accept\n", __func__);
		ret = PE_SRC_Transition_Supply; /* Accept */
	} else {
		ret = PE_SRC_Capability_Response; /* Reject */
	}
	mutex_unlock(&pd_data->accept_mutex);

	return ret;
}

policy_state usbpd_policy_src_transition_supply(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Ready;
	int ms1 = 0, ms2 = 0;

	/**********************************************
	Actions on entry:
	Initialize and run SourceActivityTimer (see Section 8.3.3.5)
	If GotoMin send GotoMin message
	Else send Accept message (within tReceiverResponse)
	Wait tSrcTransition and request Device Policy Manager to transition Power Supply

	Actions on exit:
	Send PS_RDY message
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Send Message */
	// Move to PE_SRC_Nego
	//usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Accept, USBPD_DFP, USBPD_SOURCE);

	/* 3) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 4) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Transition_Supply;
			break;
		}
		ms1 = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {

			usbpd_timer2_start(pd_data);
			while (1) {
				if (policy->plug_valid == 0) {
					ret = PE_SRC_Transition_Supply;
					break;
				}
				ms2 = usbpd_check_time2(pd_data);
				if (ms2 > tSrcTransition)
					break;
			}

			if (ret == PE_SRC_Transition_Supply)
				return ret;

			usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_PS_RDY, USBPD_DFP, USBPD_SOURCE);
			ret = PE_SRC_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}

		if (ms1 >= 8) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_src_ready(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Initialize and run SourceActivityTimer (see Section 8.3.3.5)
	Initialize and run DiscoverIdentityTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	policy->pd_support = 1;

	/* 2) Wait Message or State */
	CHECK_MSG(pd_data, MSG_GET_SRC_CAP, PE_SRC_Give_Source_Cap);
	CHECK_MSG(pd_data, MSG_REQUEST, PE_SRC_Negotiate_Capability);
	CHECK_MSG(pd_data, MSG_PR_SWAP, PE_PRS_SRC_SNK_Evaluate_Swap);
	CHECK_MSG(pd_data, MSG_DR_SWAP, PE_DRS_Evaluate_Port);
	CHECK_MSG(pd_data, MSG_VCONN_SWAP, PE_VCS_Evaluate_Swap);
	CHECK_MSG(pd_data, MSG_GET_SNK_CAP, PE_DR_SRC_Give_Sink_Cap);
	CHECK_MSG(pd_data, MSG_SOFTRESET, PE_SRC_Soft_Reset);
	CHECK_MSG(pd_data, MSG_ERROR, PE_SRC_Send_Soft_Reset);
	CHECK_MSG(pd_data, MSG_BIST, PE_BIST_Receive_Mode);
#if 0
	CHECK_MSG(pd_data, VDM_DISCOVER_IDENTITY, PE_UFP_VDM_Get_Identity);
	CHECK_MSG(pd_data, VDM_DISCOVER_SVID, PE_UFP_VDM_Get_SVIDs);
	CHECK_MSG(pd_data, VDM_DISCOVER_MODE, PE_UFP_VDM_Get_Modes);
	CHECK_MSG(pd_data, VDM_ENTER_MODE, PE_UFP_VDM_Evaluate_Mode_Entry);
	CHECK_MSG(pd_data, VDM_ATTENTION, PE_DFP_VDM_Attention_Request);
	CHECK_MSG(pd_data, VDM_DP_STATUS_UPDATE, PE_UFP_VDM_Evaluate_Status);
	CHECK_MSG(pd_data, VDM_DP_CONFIGURE, PE_UFP_VDM_Evaluate_Configure);
	CHECK_MSG(pd_data, UVDM_MSG, PE_DFP_UVDM_Receive_Message);
#endif
    /* 3) Command Check from AP */
	CHECK_CMD(pd_data, MANAGER_REQ_GET_SNKCAP, PE_SRC_Get_Sink_Cap);
	CHECK_CMD(pd_data, MANAGER_REQ_GOTOMIN, PE_SRC_Transition_Supply);
	CHECK_CMD(pd_data, MANAGER_REQ_SRCCAP_CHANGE, PE_SRC_Send_Capabilities);
	CHECK_CMD(pd_data, MANAGER_REQ_PR_SWAP, PE_PRS_SRC_SNK_Send_Swap);
	CHECK_CMD(pd_data, MANAGER_REQ_DR_SWAP, PE_DRS_Evaluate_Send_Port);
	CHECK_CMD(pd_data, MANAGER_REQ_VCONN_SWAP, PE_VCS_Send_Swap);
	CHECK_CMD(pd_data, MANAGER_REQ_UVDM_SEND_MESSAGE,
										PE_DFP_UVDM_Send_Message);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DISCOVER_IDENTITY, PE_DFP_VDM_Identity_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DISCOVER_SVID, PE_DFP_VDM_SVIDs_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DISCOVER_MODE, PE_DFP_VDM_Modes_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_ENTER_MODE, PE_DFP_VDM_Mode_Entry_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_EXIT_MODE, PE_DFP_VDM_Mode_Exit_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_STATUS_UPDATE, PE_DFP_VDM_Status_Update);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DisplayPort_Configure, PE_DFP_VDM_DisplayPort_Configure);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_ATTENTION, PE_UFP_VDM_Attention_Request);


/*	for data role swap test
	if (usbpd_manager_vdm_request_enabled(pd_data)) {
		msleep(tDiscoverIdentity);
		return PE_DRS_Evaluate_Send_Port;
	}
*/
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

#if 0
	if (data_role == USBPD_DFP)
		usbpd_manager_vdm_request_enabled(pd_data);
#endif
	return PE_SRC_Ready;
}

policy_state usbpd_policy_src_disabled(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Disable Power Delivery
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	return PE_SRC_Disabled;
}

policy_state usbpd_policy_src_capability_response(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = 0;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send Reject message if request can't be met
	Send Wait message if request could be met later from the Power
	Reserve and present Contract is still valid
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 2) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Reject,
												data_role, USBPD_SOURCE);

	/* 3) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 4) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Capability_Response;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			ret = PE_SRC_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}
		if (ms >= 5) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_src_hard_reset(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Transition_to_default;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Generate Hard Reset signalling
	Start PSHardResetTimer
	Increment HardResetCounter
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Send Hardreset */
	pd_data->phy_ops.hard_reset(pd_data);

	/* 3) Configuration Channel On */
	pd_data->phy_ops.set_cc_control(pd_data, USBPD_CC_OFF);

	/* 4) Set Tx HardReset Flag After SRC_HADRESET */
	policy->txhardresetflag = 1;

	/* 5) Delay : Setting 25 is actual 57.3ms */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= tPSHardReset)
			break;
	}

	if (ret == PE_SRC_Hard_Reset)
		return ret;

	/* 6) Add Hardreset Counter */
	pd_data->counter.hard_reset_counter++;

	return PE_SRC_Transition_to_default;
}

policy_state usbpd_policy_src_hard_reset_received(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Transition_to_default;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Start PSHardResetTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Delay */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Hard_Reset_Received;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= tPSHardReset)
			break;
	}

	if (ret == PE_SRC_Hard_Reset_Received)
		return ret;

	/* 3) Set Tx HardReset Flag After SRC_HADRESET */
	policy->txhardresetflag = 1;

	return PE_SRC_Transition_to_default;
}

policy_state usbpd_policy_src_transition_to_default(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Startup;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Request Device Policy Manager to request power
	supply Hard Resets to vSafe5V via vSafe0V
	Reset local HW
	If Type-C request Device Policy Manager to set
	Port Data Role to DFP and turn off VCONN
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) VBUS Turn off */
	pd_data->phy_ops.set_otg_control(pd_data, 0);

	/* 3) Delay */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Transition_to_default;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= tSrcRecover)
			break;
	}

	if (ret == PE_SRC_Transition_to_default)
		return ret;

	/* 4) initial reset */
	pd_data->phy_ops.driver_reset(pd_data);

	pd_data->phy_ops.set_otg_control(pd_data, 1);
	/*
	Request Device Policy Manager to request power
	supply Hard Resets to vSafe5V via vSafe0V

	If(Type-C request Device Policy Manager to set Port Data Role to DFP)
		turn off VCONN
	*/

	/*
	Request Device Policy Manager to turn on VCONN
	Initialize and start NoResponseTimer
	Inform Protocol Layer Hard Reset complete
	*/

	/* confirm VBUS ON : done by set_otg_control */
	return PE_SRC_Startup;
}

policy_state usbpd_policy_src_give_source_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Give_Source_Cap;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	Action on entry :
	Request source capabilities from Device Policy Manager
	Send Capabilities message
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 3) Message Setting */
	policy->tx_msg_header.msg_type = USBPD_Source_Capabilities;
	policy->tx_msg_header.port_data_role = data_role;
	policy->tx_msg_header.port_power_role = USBPD_SOURCE;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].power_data_obj.max_current = 1500 / 10;
	policy->tx_data_obj[0].power_data_obj.voltage = 5000 / 50;
	policy->tx_data_obj[0].power_data_obj.peak_current = 0;
	policy->tx_data_obj[0].power_data_obj.data_role_swap = 1;
	policy->tx_data_obj[0].power_data_obj.usb_comm_capable = 1;
	policy->tx_data_obj[0].power_data_obj.externally_powered = 0;
	policy->tx_data_obj[0].power_data_obj.usb_suspend_support = 1;
	policy->tx_data_obj[0].power_data_obj.dual_role_power = 1;
	policy->tx_data_obj[0].power_data_obj.supply = 0;

	/* 4) Send Message */
	usbpd_send_msg(pd_data, &policy->tx_msg_header, policy->tx_data_obj);

	/* 5) Start Timer */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Give_Source_Cap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_REQUEST)) {
			ret = PE_SRC_Negotiate_Capability;
			break;
		}
		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}
		if (ms >= tSenderResponse) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_src_get_sink_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int ret = PE_SRC_Get_Sink_Cap;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send Get_Sink_Cap message
	Initialize and run
	SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 2) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Get_Sink_Cap, data_role, USBPD_SOURCE);

	/* 3) Wait Message */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0)
			break;
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_SNK_CAP)) {
			dev_info(pd_data->dev, "got SinkCap.\n");
			ret = PE_SRC_Ready;
			break;
		}
		if (ms >= tSenderResponse) {
			ret = PE_SRC_Ready;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_src_wait_new_capabilities(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Wait for new Source Capabilities
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	return PE_SRC_Send_Capabilities;
}

policy_state usbpd_policy_src_send_soft_reset(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Send_Soft_Reset;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Reset Protocol Layer
	Send Soft Reset message
	Initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) USB PD Protocol Initialization */
	usbpd_init_protocol(pd_data);

	/* 3) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 4) Self SoftReset */
	pd_data->phy_ops.soft_reset(pd_data);

	/* 5) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Soft_Reset,
													data_role, USBPD_SOURCE);

	/* 6) Start Timer */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
			ret = PE_SRC_Send_Capabilities;
			break;
		}
		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
		if (ms >= tSenderResponse) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_src_soft_reset(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SRC_Soft_Reset;
	int data_role = 0;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Reset Protocol Layer
	Send Accept message
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) USB PD Protocol Initialization */
	usbpd_init_protocol(pd_data);

	/* 3) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 4) Self SoftReset */
	pd_data->phy_ops.soft_reset(pd_data);

	/* 5) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Accept, data_role, USBPD_SOURCE);

	/* 6) Start Timer */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SRC_Soft_Reset;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			ret = PE_SRC_Send_Capabilities;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Hard_Reset;
			break;
		}
		if (ms >= 5) {
			ret = PE_SRC_Ready;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_snk_startup(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Reset Protocol Layer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) PD Protocol Initialization */
	usbpd_init_protocol(pd_data);

	/* 3) Configuration Channel On */
	//pd_data->phy_ops.set_cc_control(pd_data, USBPD_CC_ON);
	//Move to PE_SNK_Wait_for_Capabilities

	return PE_SNK_Discovery;
}

policy_state usbpd_policy_snk_discovery(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = 0;
	int vbus_check = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Wait for VBUS
	**********************************************/

	/* TODO: wait vbus */
	/* if coming from HardReset
	   && NoResponseTimer timeout
	   && HardResetCounter <= nHardResetCount,
	   return(PE_SNK_Hard_Reset) */

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 3) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Discovery;
			break;
		}
		ms = usbpd_check_time1(pd_data);

		vbus_check = pd_data->phy_ops.vbus_on_check(pd_data);
		if (vbus_check < 0 || vbus_check > 0) {
			ret = PE_SNK_Wait_for_Capabilities;
			break;
		}

		/* TimeOver Check */
		if (ms >= tNoResponse) {
			/* HardReset Count Check */
			if (pd_data->counter.hard_reset_counter <= USBPD_nHardResetCount) {
				ret = PE_SNK_Hard_Reset;
				break;
			} else {
				ret = Error_Recovery;
				break;
			}
		}
	}

	return ret;
}

policy_state usbpd_policy_snk_wait_for_capabilities(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SNK_Wait_for_Capabilities;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Initialize and run SinkWaitCapTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* Configuration Channel On */
	pd_data->phy_ops.set_cc_control(pd_data, USBPD_CC_ON);

	/* Start Timer */
	usbpd_timer1_start(pd_data);

	/* 4) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Wait_for_Capabilities;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		/* Rx Source Capabilities */
		if (pd_data->phy_ops.get_status(pd_data, MSG_SRC_CAP)) {
			ret = PE_SNK_Evaluate_Capability;
			break;
		}

#if !defined(CONFIG_SEC_FACTORY)
		/* TimeOver Check */
		if (ms >= tTypeCSinkWaitCap) {
			/* HardReset Count Check */
			if (pd_data->counter.hard_reset_counter <= USBPD_nHardResetCount) {
				ret = PE_SNK_Hard_Reset;
				break;
			} else {
				ret = Error_Recovery;
				break;
			}
		}
#endif
	}

	return ret;
}

policy_state usbpd_policy_snk_evaluate_capability(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int sink_request_obj_num = 0;
	int ret = PE_SNK_Evaluate_Capability;

	/**********************************************
	Actions on entry:
	Reset HardResetCounter to zero.
	Ask Device Policy Manager to evaluate the options based on supplied
	capabilities, any Power Reserve that it needs, and respond indicating
	the selected capability and, optionally, a Capability Mismatch
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 4) Select PDO */
#if defined(CONFIG_IFCONN_NOTIFIER)
	if (pd_noti.sink_status.selected_pdo_num == 0) {
		pd_noti.sink_status.selected_pdo_num = 1;
		if (policy->sink_cap_received) {
			policy->send_sink_cap = 1;
			policy->sink_cap_received = 0;
		}
	}
#endif
	/* 5) Select Object Position */
	sink_request_obj_num = usbpd_manager_evaluate_capability(pd_data);

	/* 6) Branch */
	if (sink_request_obj_num > 0)
		ret = PE_SNK_Select_Capability;
	else
		ret = PE_SNK_Hard_Reset;

	return ret;
}

policy_state usbpd_policy_snk_select_capability(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SNK_Select_Capability;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send Request based on Device Policy Manager response:
	- Request from present capabilities
	- Optionally Indicate that other capabilities would be preferred (Capability Mismatch)
	Initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform to AP */
	//dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 2) Message Header Setting */
	policy->tx_msg_header.msg_type = USBPD_Request;
	policy->tx_msg_header.port_data_role = data_role;
	policy->tx_msg_header.port_power_role = USBPD_SINK;
	policy->tx_msg_header.num_data_objs = 1; /* Initial Select PDO = 1 */

	/* 3) Select PDO */
	policy->tx_data_obj[0] = usbpd_manager_select_capability(pd_data);

	/* 4) Send Message*/
	usbpd_send_msg(pd_data, &policy->tx_msg_header, policy->tx_data_obj);

	/* 5) Start Timer*/
	usbpd_timer1_start(pd_data);

	/* 6) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Select_Capability;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GET_SNK_CAP)) {
			ret = PE_SNK_Send_Soft_Reset;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
			ret = PE_SNK_Transition_Sink;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_PSRDY)) {
			ret = PE_SNK_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT | MSG_WAIT)) {
			/* 1st Power Negotiation Check */
			if (pd_noti.sink_status.selected_pdo_num == 0)
				ret = PE_SNK_Wait_for_Capabilities;
			else
				ret = PE_SNK_Ready;

			break;
		}

		/* TimeOver Check */
		if (ms >= tSenderResponse) {
			ret = PE_SNK_Hard_Reset;
			break;
		}
	}

	return ret;
}

uint32_t RX_PS_READY_Message_ID;

policy_state usbpd_policy_snk_transition_sink(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret  = PE_SNK_Transition_Sink;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Initialize and run PSTransitionTimer
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Policy Engine State Setting */
	policy->state = PE_SNK_Transition_Sink;

	/* 3) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 4) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Transition_Sink;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		/* PD Certification(Ellisys) : TD.PD.SNK.E10 GetSinkCap in place of PS_RDY */
		if (pd_data->phy_ops.get_status(pd_data, MSG_GET_SNK_CAP)) {
			ret = PE_SNK_Hard_Reset;
			break;
		}
		if (pd_data->phy_ops.get_status(pd_data, MSG_PSRDY)) {
			/* Device Information */
			dev_info(pd_data->dev, "got PS_READY.\n");

#if defined(CONFIG_IFCONN_NOTIFIER)
			pd_noti.sink_status.current_pdo_num = pd_noti.sink_status.selected_pdo_num;
#endif
			ret = PE_SNK_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= tPSTransition) {
			ret = PE_SNK_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_snk_ready(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Initialize and run SinkActivityTimer2
	Initialize and run SinkRequestTimer3 (on receiving Wait)
	Initialize and run DiscoverIdentityTimer5
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* enable pd support for typec role swap */
	policy->pd_support = 1;

	/* 2) Notify Plug Attach */
	usbpd_manager_plug_attach(pd_data->dev);

	/* 3) Message Check */
	CHECK_MSG(pd_data, MSG_GET_SNK_CAP, PE_SNK_Give_Sink_Cap);
	CHECK_MSG(pd_data, MSG_PR_SWAP, PE_PRS_SNK_SRC_Evaluate_Swap);
	CHECK_MSG(pd_data, MSG_DR_SWAP, PE_DRS_Evaluate_Port);
	CHECK_MSG(pd_data, MSG_VCONN_SWAP, PE_VCS_Evaluate_Swap);
	CHECK_MSG(pd_data, MSG_GET_SRC_CAP, PE_DR_SNK_Give_Source_Cap);
	CHECK_MSG(pd_data, MSG_BIST, PE_BIST_Receive_Mode);

#if 0
	CHECK_MSG(pd_data, VDM_DISCOVER_IDENTITY, PE_UFP_VDM_Get_Identity);
	CHECK_MSG(pd_data, VDM_DISCOVER_SVID, PE_UFP_VDM_Get_SVIDs);
	CHECK_MSG(pd_data, VDM_DISCOVER_MODE, PE_UFP_VDM_Get_Modes);
	CHECK_MSG(pd_data, VDM_ENTER_MODE, PE_UFP_VDM_Evaluate_Mode_Entry);
	CHECK_MSG(pd_data, VDM_ATTENTION, PE_DFP_VDM_Attention_Request);
	CHECK_MSG(pd_data, VDM_DP_STATUS_UPDATE, PE_UFP_VDM_Evaluate_Status);
	CHECK_MSG(pd_data, VDM_DP_CONFIGURE, PE_UFP_VDM_Evaluate_Configure);
	CHECK_MSG(pd_data, UVDM_MSG, PE_DFP_UVDM_Receive_Message);
#endif

	/* 5) Command Check from AP */
	CHECK_CMD(pd_data, MANAGER_REQ_NEW_POWER_SRC, PE_SNK_Select_Capability);
	CHECK_CMD(pd_data, MANAGER_REQ_PR_SWAP, PE_PRS_SNK_SRC_Send_Swap);
	CHECK_CMD(pd_data, MANAGER_REQ_DR_SWAP, PE_DRS_Evaluate_Send_Port);
	CHECK_CMD(pd_data, MANAGER_REQ_VCONN_SWAP, PE_VCS_Send_Swap);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DISCOVER_IDENTITY, PE_DFP_VDM_Identity_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DISCOVER_SVID, PE_DFP_VDM_SVIDs_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DISCOVER_MODE, PE_DFP_VDM_Modes_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_ATTENTION, PE_UFP_VDM_Attention_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_ENTER_MODE, PE_DFP_VDM_Mode_Entry_Request);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_STATUS_UPDATE, PE_DFP_VDM_Status_Update);
	CHECK_CMD(pd_data, MANAGER_REQ_VDM_DisplayPort_Configure, PE_DFP_VDM_DisplayPort_Configure);
	CHECK_CMD(pd_data, MANAGER_REQ_UVDM_SEND_MESSAGE, PE_DFP_UVDM_Send_Message);

	/* 6) Data Role Check */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

#if 0
	if (data_role == USBPD_DFP)
		usbpd_manager_vdm_request_enabled(pd_data);
#endif

	return PE_SNK_Ready;
}

policy_state usbpd_policy_snk_hard_reset(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Generate Hard Reset signalling.
	Increment HardResetCounter.
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.hard_reset(pd_data);
	pd_data->phy_ops.set_cc_control(pd_data, USBPD_CC_OFF);
	/* increase hard reset counter */
	pd_data->counter.hard_reset_counter++;

	return PE_SNK_Transition_to_default;
}

policy_state usbpd_policy_snk_transition_to_default(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SNK_Startup;
	int ms = 0;

	/**********************************************
	Hard reset signalling received

	Actions on entry:
	Request Device Policy Manager to request power sink transition to default
	Reset local HW
	If Type-C set Port Data Role to UFP and turn off VCONN
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Driver Reset */
	pd_data->phy_ops.driver_reset(pd_data);

	/* 3) Vconn Off */
	usbpd_manager_turn_off_vconn(pd_data);

	/* 4) Wait 200ms */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Transition_to_default;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= 200)
			break;
	}

	return ret;
}

policy_state usbpd_policy_snk_give_sink_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SNK_Give_Sink_Cap;
	int data_role = 0;
	int ms = 0;
	/**********************************************
	Get Sink Cap Message received

	Actions on entry:
	Get present sink capabilities from Device Policy Manager
	Send Capabilities message (based on Device Policy Manager response)
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 2) TODO JETSEO ????*/
#if defined(CONFIG_IFCONN_NOTIFIER)
	pd_noti.sink_status.selected_pdo_num = 0;
#endif

	/* 3) Sink Cap Message Setting */
	policy->tx_msg_header.word = pd_data->sink_msg_header.word;
	policy->tx_msg_header.port_data_role = data_role;
	policy->tx_data_obj[0].object = pd_data->sink_data_obj[0].object;
#if 0
	policy->tx_data_obj[1].object = pd_data->sink_data_obj[1].object;
#endif
	policy->sink_cap_received = 1;

	/* 4) Send Message */
	usbpd_send_msg(pd_data, &policy->tx_msg_header, policy->tx_data_obj);

	/* 5) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 6) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Give_Sink_Cap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			dev_info(pd_data->dev, "got snk_give_sink_cap MSG_GOODCRC.\n");
			ret = PE_SNK_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= 5) {
			dev_info(pd_data->dev, "got snk_give_sink_cap Timer1_overflag.\n");
			ret = PE_SNK_Send_Soft_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_snk_get_source_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SNK_Get_Source_Cap;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send Get_Source_Cap message
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 2) Send Message*/
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
						USBPD_Get_Source_Cap, data_role, USBPD_SINK);

	/* 3) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 4) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Get_Source_Cap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			ret = PE_SNK_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= 5) {
			ret = PE_SNK_Get_Source_Cap;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_snk_send_soft_reset(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_SNK_Send_Soft_Reset;
	int data_role = 0;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Reset Protocol Layer
	Send Soft Reset message
	Initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) USB PD Protocol Initialization */
	usbpd_init_protocol(pd_data);

	/* 3) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 4) Self SoftReset */
	pd_data->phy_ops.soft_reset(pd_data);

	/* 5) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Soft_Reset,
														data_role, USBPD_SINK);

	/* 6) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 7) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Send_Soft_Reset;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
			ret = PE_SNK_Wait_for_Capabilities;
			break;
		}

		/* TimeOver Check */
		if (ms >= tSenderResponse) {
			ret = PE_SNK_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_snk_soft_reset(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int ret = PE_SNK_Soft_Reset;
	int ms = 0;

	/**********************************************
	Soft Reset message received

	Actions on entry:
	Reset Protocol Layer
	Send Accept message
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) USB PD Protocol Initialization */
	usbpd_init_protocol(pd_data);

	/* 3) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 4) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Accept,
													data_role, USBPD_SINK);

	/* 5) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 6) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_SNK_Soft_Reset;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			ret = PE_SNK_Wait_for_Capabilities;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SNK_Hard_Reset;
			break;
		}

		if (ms >= 5) {
			ret = PE_SNK_Ready;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_drs_evaluate_port(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int power_role = 0;

	/**********************************************
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	if (policy->modal_operation) {
		pd_data->phy_ops.get_power_role(pd_data, &power_role);

		if (power_role == USBPD_SOURCE)
			return PE_SRC_Hard_Reset;
		else
			return PE_SNK_Hard_Reset;
	}

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	if (data_role == USBPD_DFP)
		return PE_DRS_DFP_UFP_Evaluate_DR_Swap;
	else
		return PE_DRS_UFP_DFP_Evaluate_DR_Swap;
}

policy_state usbpd_policy_drs_evaluate_send_port(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int power_role = 0;

	/**********************************************
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	if (policy->modal_operation) {
		pd_data->phy_ops.get_power_role(pd_data, &power_role);

		if (power_role == USBPD_SOURCE)
			return PE_SRC_Hard_Reset;
		else
			return PE_SNK_Hard_Reset;
	}

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	if (data_role == USBPD_DFP)
		return PE_DRS_DFP_UFP_Send_DR_Swap;
	else
		return PE_DRS_UFP_DFP_Send_DR_Swap;
}

policy_state usbpd_policy_drs_dfp_ufp_evaluate_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	bool drs_ok;

	/**********************************************
	DR_Swap message received & not in Modal Operation

	Actions on entry:
	Get evaluation of Data Role Swap
	request from Device Policy Manager
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	drs_ok = usbpd_manager_data_role_swap(pd_data);

	if (drs_ok)
		return PE_DRS_DFP_UFP_Accept_DR_Swap;
	else
		return PE_DRS_DFP_UFP_Reject_DR_Swap;
}

policy_state usbpd_policy_drs_dfp_ufp_accept_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Accept message
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	mutex_lock(&pd_data->accept_mutex);
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
						USBPD_Accept, USBPD_DFP, power_role);

	pd_data->phy_ops.set_data_role(pd_data, USBPD_UFP);
	mutex_unlock(&pd_data->accept_mutex);

	return PE_DRS_DFP_UFP_Change_to_UFP;
}

policy_state usbpd_policy_drs_dfp_ufp_change_to_ufp(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Request Device Policy Manager to
	change port to UFP
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_drs_dfp_ufp_send_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;
	int ret = 0;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Send Swap DR message
	Initialize and run SenderResponseTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);
	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_DR_Swap, USBPD_DFP, power_role)) {
		usbpd_timer1_start(pd_data);
		while (1) {
			if (policy->plug_valid == 0) {
				ret = PE_DRS_DFP_UFP_Send_DR_Swap;
				break;
			}
			ms = usbpd_check_time1(pd_data);
			if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
				dev_info(pd_data->dev, "%s, got Accept\n", __func__);
				ret = PE_DRS_DFP_UFP_Change_to_UFP;
				break;
			}
			if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT)) {
				dev_info(pd_data->dev, "%s, got Reject\n", __func__);
				break;
			}
			if (pd_data->phy_ops.get_status(pd_data, MSG_WAIT)) {
				dev_info(pd_data->dev, "%s, got Wait\n", __func__);
				break;
			}
			if (ms >= tSenderResponse)
				break;
		}
		if (ret > 0)
			return ret;
	}

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_drs_dfp_ufp_reject_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Reject or Wait message as appropriate
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);
	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Reject, USBPD_DFP, power_role)) {
		if (power_role == USBPD_SOURCE)
			return PE_SRC_Ready;
		else
			return PE_SNK_Ready;
	}

	return PE_DRS_DFP_UFP_Reject_DR_Swap;
}

policy_state usbpd_policy_drs_ufp_dfp_evaluate_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	bool drs_ok;

	/**********************************************
	Actions on entry:
	Get evaluation of Data Role Swap
	request from Device Policy Manager
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	drs_ok = usbpd_manager_data_role_swap(pd_data);

	if (drs_ok)
		return PE_DRS_UFP_DFP_Accept_DR_Swap;
	else
		return PE_DRS_UFP_DFP_Reject_DR_Swap;
}

policy_state usbpd_policy_drs_ufp_dfp_accept_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Accept message
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);
	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	mutex_lock(&pd_data->accept_mutex);
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
					USBPD_Accept, USBPD_UFP, power_role);
	pd_data->phy_ops.set_data_role(pd_data, USBPD_DFP);
	mutex_unlock(&pd_data->accept_mutex);
	return PE_DRS_UFP_DFP_Change_to_DFP;
}

policy_state usbpd_policy_drs_ufp_dfp_change_to_dfp(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Request Device Policy Manager to change port to DFP
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_drs_ufp_dfp_send_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;
	int ret = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send Swap DR message
	Initialize and run SenderResponseTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);
	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_DR_Swap, USBPD_UFP, power_role)) {
		usbpd_timer1_start(pd_data);
		while (1) {
			if (policy->plug_valid == 0) {
				ret = PE_DRS_UFP_DFP_Send_DR_Swap;
				break;
			}
			ms = usbpd_check_time1(pd_data);
			if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
				dev_info(pd_data->dev, "%s, got Accept\n", __func__);
				ret = PE_DRS_UFP_DFP_Change_to_DFP;
				break;
			}
			if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT)) {
				dev_info(pd_data->dev, "%s, got Reject\n", __func__);
				break;
			}
			if (pd_data->phy_ops.get_status(pd_data, MSG_WAIT)) {
				dev_info(pd_data->dev, "%s, got Wait\n", __func__);
				break;
			}
			if (ms > tSenderResponse)
				break;
		}
		if (ret > 0)
			return ret;
	}

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_drs_ufp_dfp_reject_dr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Send Reject or Wait message as appropriate
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);
	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Reject, data_role, USBPD_SINK)) {
		if (power_role == USBPD_SOURCE)
			return PE_SRC_Ready;
		else
			return PE_SNK_Ready;
	}

	return PE_DRS_UFP_DFP_Reject_DR_Swap;
}

policy_state usbpd_policy_prs_src_snk_reject_pr_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Send Reject or Wait message as appropriate
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Reject, data_role, USBPD_SOURCE))
		return PE_SRC_Ready;

	return PE_PRS_SRC_SNK_Reject_PR_Swap;
}

policy_state usbpd_policy_prs_src_snk_evaluate_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	bool prs_ok;

	/**********************************************
	Actions on entry:
	Get evaluation of swap request from Device Policy Manager
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) DPM Check to support roleswap */
	prs_ok = usbpd_manager_power_role_swap(pd_data);

	/* 3) Branch */
	if (prs_ok)
		return PE_PRS_SRC_SNK_Accept_Swap;
	else
		return PE_PRS_SRC_SNK_Reject_PR_Swap;
}

policy_state usbpd_policy_prs_src_snk_send_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int ret = PE_SRC_Ready;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send PR_Swap message
	Initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
								USBPD_PR_Swap, data_role, USBPD_SOURCE);

	/* 3) Start Timer */
	usbpd_timer1_start(pd_data);

	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SRC_SNK_Send_Swap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
			ret = PE_PRS_SRC_SNK_Transition_off;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT)) {
			ret = PE_SRC_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_WAIT)) {
			ret = PE_SRC_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= tSenderResponse + 5) {
			ret = PE_SRC_Ready;
			break;
		}

	}

	return PE_SRC_Ready;
}

policy_state usbpd_policy_prs_src_snk_accept_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Send Accept message
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
					USBPD_Accept, data_role, USBPD_SOURCE);

	return PE_PRS_SRC_SNK_Transition_off;

}

policy_state usbpd_policy_prs_src_snk_transition_to_off(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	struct usbpd_manager_data *manager = &pd_data->manager;
	int ret = PE_PRS_SRC_SNK_Assert_Rd;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Tell Device Policy Manager to turn off power supply
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Delay */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SRC_SNK_Transition_off;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= tSrcTransition)
			break;
	}

	if (ret == PE_PRS_SRC_SNK_Transition_off)
		return ret;

	pd_data->phy_ops.pr_swap(pd_data, USBPD_SOURCE_OFF);

	/* 3) VBUS off */
	pd_data->phy_ops.set_otg_control(pd_data, 0);

	pr_info("%s, %d\n", __func__, manager->acc_type);

	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SRC_SNK_Transition_off;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= 500)
			break;
	}
#if 0
	/* TODO: svid_0 == 0 confition check?
	 * based on 004 code 600ms no condition vs here 150ms w/ condition */
	/* skip delay when GEARVR is attached */
	if (manager->acc_type != CCIC_DOCK_HMT || manager->SVID_0 == 0)
		msleep(600);
#endif

	return ret;
}

policy_state usbpd_policy_prs_src_snk_assert_rd(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request DPM to assert Rd
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Asserted Rd */
	pd_data->phy_ops.set_power_role(pd_data, USBPD_SINK);

	return PE_PRS_SRC_SNK_Wait_Source_on;
}

policy_state usbpd_policy_prs_src_snk_wait_source_on(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = 0;
	int data_role = 0;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Send PS_RDY message
	Initialize and run PSSourceOnTimer
	**********************************************/

	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 2) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
						USBPD_PS_RDY, data_role, USBPD_SINK);
	usbpd_timer1_start(pd_data);

	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SRC_SNK_Wait_Source_on;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_PSRDY)) {
			dev_info(pd_data->dev, "got PSRDY.\n");
			pd_data->counter.swap_hard_reset_counter = 0;
			/* Self SoftReset for Message ID Clear */
			pd_data->phy_ops.soft_reset(pd_data);
			mdelay(15);
			ret = PE_SNK_Startup;
			break;
		}
		if (ms >= tPSSourceOn) {
			ret = PE_SNK_Hard_Reset;
			if (pd_data->counter.hard_reset_counter > USBPD_nHardResetCount)
				ret = Error_Recovery;
			break;
		}
	}

	pd_data->phy_ops.set_power_role(pd_data, USBPD_DRP);

	return ret;
}

policy_state usbpd_policy_prs_snk_src_reject_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Send Reject or Wait message as appropriate
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Reject, USBPD_UFP, USBPD_SINK))
		return PE_SNK_Ready;

	return PE_PRS_SNK_SRC_Reject_Swap;
}

policy_state usbpd_policy_prs_snk_src_evaluate_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	bool prs_ok;
	int ret = PE_PRS_SNK_SRC_Evaluate_Swap;

	/**********************************************
	Actions on entry:
	Get evaluation of swap request from Device Policy Manager
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Power Role Swap Check */
	prs_ok = usbpd_manager_power_role_swap(pd_data);

	if (prs_ok)
		ret = PE_PRS_SNK_SRC_Accept_Swap;
	else
		ret = PE_PRS_SNK_SRC_Reject_Swap;

	return ret;
}

policy_state usbpd_policy_prs_snk_src_send_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int ret = PE_SNK_Ready;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send PR_Swap message
	Initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
						USBPD_PR_Swap, data_role, USBPD_SINK);

	usbpd_timer1_start(pd_data);

	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SNK_SRC_Send_Swap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_ACCEPT)) {
			ret = PE_PRS_SNK_SRC_Transition_off;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT)) {
			ret = PE_SNK_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_WAIT)) {
			ret = PE_SNK_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= tSenderResponse) {
			ret = PE_SNK_Ready;
			break;
		}

	}

	return ret;
}

policy_state usbpd_policy_prs_snk_src_accept_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Send Accept message
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* Send Accept Message */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Accept,
													data_role, USBPD_SINK);
	pd_data->phy_ops.set_power_role(pd_data, USBPD_SINK);

	return PE_PRS_SNK_SRC_Transition_off;
}

policy_state usbpd_policy_prs_snk_src_transition_to_off(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = 0;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Initialize and run PSSourceOffTimer
	Tell Device Policy Manager to turn off Power Sink.
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.pr_swap(pd_data, USBPD_SINK_OFF);

	/* Start Timer 750ms */
	usbpd_timer1_start(pd_data);

	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SNK_SRC_Transition_off;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_PSRDY)) {
			dev_info(pd_data->dev, "got PSRDY.\n");
			ret = PE_PRS_SNK_SRC_Assert_Rp;
			break;
		}
		if (ms >= tPSSourceOff) {
			ret = PE_SRC_Hard_Reset;
			pd_data->phy_ops.set_power_role(pd_data, USBPD_DRP);
			if (pd_data->counter.hard_reset_counter > USBPD_nHardResetCount)
				ret = Error_Recovery;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_prs_snk_src_assert_rp(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request DPM to assert Rp
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.set_power_role(pd_data, USBPD_SOURCE);

	return PE_PRS_SNK_SRC_Source_on;
}

policy_state usbpd_policy_prs_snk_src_source_on(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int ret = 0;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Tell Device Policy Manager to turn on Source
	Initialize and run SourceActivityTimer (see Section 8.3.3.6.1.2)1
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	pd_data->phy_ops.pr_swap(pd_data, USBPD_SOURCE_ON);

	/* 3) VBUS on */
	pd_data->phy_ops.set_otg_control(pd_data, 1);

	/* 4) Dealy */
	usbpd_timer1_start(pd_data);
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_PRS_SNK_SRC_Source_on;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (ms >= 200)
			break;
	}

	if (ret == PE_PRS_SNK_SRC_Source_on)
		return ret;

	/* 5) send PS_RDY */
	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_PS_RDY, data_role, USBPD_SOURCE)) {
		pd_data->phy_ops.set_power_role(pd_data, USBPD_DRP);
		usbpd_timer1_start(pd_data);
		while (1) {
			if (policy->plug_valid == 0) {
				ret = PE_PRS_SNK_SRC_Source_on;
				break;
			}
			ms = usbpd_check_time1(pd_data);
			if (ms >= tSwapSourceStart)
				break;
		}

		if (ret == PE_PRS_SNK_SRC_Source_on)
			return ret;
		/*  TODO: 4) check GoodCRC : may need to be added for certification */

		/* Self SoftReset for Message ID Clear */
		pd_data->phy_ops.soft_reset(pd_data);

		return PE_SRC_Startup;
	}

	return PE_PRS_SNK_SRC_Source_on;
}

policy_state usbpd_policy_vcs_evaluate_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	bool vcs_ok;

	/**********************************************
	Actions on entry:
	Get evaluation of VCONN swap
	request from Device Policy Manager
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Request from DPM */
	vcs_ok = usbpd_manager_vconn_source_swap(pd_data);

	if (vcs_ok)
		return PE_VCS_Accept_Swap;
	else
		return PE_VCS_Reject_VCONN_Swap;
}

policy_state usbpd_policy_vcs_accept_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int vconn_source = 0;
	int power_role = 0;
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Send Accept message
	**********************************************/

	pd_data->phy_ops.get_vconn_source(pd_data, &vconn_source);
	pd_data->phy_ops.get_power_role(pd_data, &power_role);
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Accept, data_role, power_role)) {
		if (vconn_source)
			return PE_VCS_Wait_for_VCONN;
		else
			return PE_VCS_Turn_On_VCONN;
	}

	return PE_VCS_Accept_Swap;
}

policy_state usbpd_policy_vcs_send_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int vconn_source = 0;
	int power_role = 0;
	int ret = PE_VCS_Send_Swap;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Send VCONN_Swap message
	Initialize and run SenderResponseTimer
	**********************************************/


	/* 1) PD State Inform for AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Get Vconn Source */
	pd_data->phy_ops.get_vconn_source(pd_data, &vconn_source);

	/* 3) Get Power Role */
	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	/* 4) Send Vconn Swap */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_VCONN_Swap, USBPD_DFP, power_role);

	/* 5) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 6) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_VCS_Send_Swap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			if (vconn_source)
				ret = PE_VCS_Wait_for_VCONN;
			else
				ret = PE_VCS_Turn_On_VCONN;
			break;
		}
		/* TimeOver Check */
		if (ms >= 5) {
			if (power_role == USBPD_SINK)
				ret = PE_SNK_Hard_Reset;
			else
				ret = PE_SRC_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_vcs_wait_for_vconn(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_VCS_Wait_for_VCONN;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Start VCONNOnTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	/* 5) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 6) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_VCS_Wait_for_VCONN;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_PSRDY)) {
			pd_data->counter.swap_hard_reset_counter = 0;
			ret = PE_VCS_Turn_Off_VCONN;	
			break;
		}
		/* TimeOver Check */
		if (ms >= tVCONNSourceOn) {
			if (pd_data->counter.swap_hard_reset_counter > USBPD_nHardResetCount)
				ret = Error_Recovery;
			else
				ret = PE_SNK_Hard_Reset;
			break;
		}
	}

	return ret;
}

policy_state usbpd_policy_vcs_turn_off_vconn(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Tell Device Policy Manager to turn off VCONN
	**********************************************/

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.set_vconn_source(pd_data, USBPD_VCONN_OFF);

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_vcs_turn_on_vconn(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Tell Device Policy Manager to turn on VCONN
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.set_vconn_source(pd_data, USBPD_VCONN_ON);

	return PE_VCS_Send_PS_RDY;
}

policy_state usbpd_policy_vcs_send_ps_rdy(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;
	int data_role = 0;

	/**********************************************
	Actions on entry:
	Send PS_RDY message
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	mdelay(5);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_PS_RDY, data_role, data_role)) {
		if (power_role == USBPD_SOURCE)
			return PE_SRC_Ready;
		else
			return PE_SNK_Ready;
	}

	return PE_VCS_Send_PS_RDY;
}

policy_state usbpd_policy_vcs_reject_vconn_swap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Reject or Wait message as appropriate
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
				USBPD_Reject, USBPD_DFP, power_role)) {
		if (power_role == USBPD_SOURCE)
			return PE_SRC_Ready;
		else
			return PE_SNK_Ready;
	}

	return PE_VCS_Reject_VCONN_Swap;
}

policy_state usbpd_policy_ufp_vdm_get_identity(struct policy_data *policy)
{

	/**********************************************
	Actions on entry:
	Request Identity information from DPM
	**********************************************/

	return PE_UFP_VDM_Get_Identity_NAK;
}

policy_state usbpd_policy_ufp_vdm_send_identity(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover Identity ACK
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = Discover_Identity;

	/* TODO: data object should be prepared from device manager */

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Send_Identity;
}

policy_state usbpd_policy_ufp_vdm_get_identity_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry: NAK
	Send Discover Identity NAK/BUSY Command
	response as requested
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = Discover_Identity;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Get_Identity_NAK;
}

policy_state usbpd_policy_ufp_vdm_get_svids(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request SVIDs information from DPM
	**********************************************/

	if (usbpd_manager_get_svids(pd_data) == 0)
		return PE_UFP_VDM_Send_SVIDs;
	else
		return PE_UFP_VDM_Get_SVIDs_NAK;

}

policy_state usbpd_policy_ufp_vdm_send_svids(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover SVIDs ACK
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 2;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = Discover_SVIDs;

	policy->tx_data_obj[1].vdm_svid.svid_0 = PD_SID;
	policy->tx_data_obj[1].vdm_svid.svid_1 = 0xFF01;

	/* TODO: data object should be prepared from device manager */

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}

	return PE_UFP_VDM_Send_SVIDs;
}

policy_state usbpd_policy_ufp_vdm_get_svids_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover SVIDs NAK/BUSY Command
	response as requested
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = Discover_SVIDs;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Get_SVIDs_NAK;
}

policy_state usbpd_policy_ufp_vdm_get_modes(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request Modes information from DPM
	**********************************************/

	if (usbpd_manager_get_modes(pd_data) == 0)
		return PE_UFP_VDM_Send_Modes;
	else
		return PE_UFP_VDM_Get_Modes_NAK;
}

policy_state usbpd_policy_ufp_vdm_send_modes(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover Modes ACK
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 2;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = Discover_Modes;

	/* TODO: data object should be prepared from device manager */

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Send_Modes;
}

policy_state usbpd_policy_ufp_vdm_get_modes_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover Modes NAK/BUSY Command
	response as requested
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = Discover_Modes;

	/* TODO: data object should be prepared from device manager */

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Get_Modes_NAK;
}

policy_state usbpd_policy_ufp_vdm_evaluate_mode_entry(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request DPM to evaluate request to enter a Mode
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	/* Certification: Ellisys: TD.PD.VDMU.E15.Applicability */
	if (usbpd_manager_get_svids(pd_data) == 0)
		return PE_UFP_VDM_Mode_Entry_ACK;
	else
		return PE_UFP_VDM_Mode_Entry_NAK;

	/* Todo
	check DPM evaluate request to enter a mode
	*/
/*
	if (usbpd_manager_enter_mode(pd_data, mode_pos,
				mode_vdo) == 0)
		return PE_UFP_VDM_Mode_Entry_ACK;
	else
		return PE_UFP_VDM_Mode_Entry_NAK;
*/
	return PE_UFP_VDM_Evaluate_Mode_Entry;
}

policy_state usbpd_policy_ufp_vdm_mode_entry_ack(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Enter Mode ACK Command
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = Enter_Mode;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		/* TODO: may need to wait a while(5ms) and send status_update */
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Mode_Entry_ACK;
}

policy_state usbpd_policy_ufp_vdm_mode_entry_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Enter Mode NAK Command response as requested
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = Enter_Mode;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Mode_Entry_NAK;
}

policy_state usbpd_policy_ufp_vdm_mode_exit(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request DPM to evaluate request to exit the requested Mode
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	if (pd_data->phy_ops.get_status(pd_data, VDM_EXIT_MODE)) {
		if (policy->rx_data_obj[0].structured_vdm.command
				== Exit_Mode) {
			unsigned mode_pos;

			/* get mode to exit */
			mode_pos = policy->rx_data_obj[0].structured_vdm.obj_pos;
			if (usbpd_manager_exit_mode(pd_data, mode_pos) == 0)
				return PE_UFP_VDM_Mode_Exit_ACK;
			else
				return PE_UFP_VDM_Mode_Exit_NAK;
		}
	}
	return PE_UFP_VDM_Mode_Exit;

}

policy_state usbpd_policy_ufp_vdm_mode_exit_ack(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Exit Mode ACK Command
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = Exit_Mode;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Mode_Exit_NAK;
}

policy_state usbpd_policy_ufp_vdm_mode_exit_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Exit Mode NAK Command
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = Exit_Mode;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Mode_Exit_NAK;
}

policy_state usbpd_policy_ufp_vdm_attention_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Attention Command request
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
/*	policy->tx_msg_header.num_data_objs = 1; number of objects*/

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 0;
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = Attention;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Attention_Request;

}

policy_state usbpd_policy_ufp_vdm_evaluate_status(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SINK)
		return PE_SNK_Ready;
	else
		return PE_SRC_Ready;

	/* Todo
	check DPM evaluate request to inform status
	*/
/*
	if (usbpd_manager_enter_mode(pd_data, mode_pos,
				mode_vdo) == 0)
		return PE_UFP_VDM_Mode_Entry_ACK;
	else
		return PE_UFP_VDM_Mode_Entry_NAK;
*/
}

policy_state usbpd_policy_ufp_vdm_status_ack(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = DisplayPort_Status_Update;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Status_ACK;
}

policy_state usbpd_policy_ufp_vdm_status_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = DisplayPort_Status_Update;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Status_NAK;
}

policy_state usbpd_policy_ufp_vdm_evaluate_configure(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SINK)
		return PE_SNK_Ready;
	else
		return PE_SRC_Ready;

	/* Todo
	check DPM evaluate request to inform status
	*/
/*
	if (usbpd_manager_enter_mode(pd_data, mode_pos,
				mode_vdo) == 0)
		return PE_UFP_VDM_Mode_Entry_ACK;
	else
		return PE_UFP_VDM_Mode_Entry_NAK;
*/
}

policy_state usbpd_policy_ufp_vdm_configure_ack(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_ACK;
	policy->tx_data_obj[0].structured_vdm.command = DisplayPort_Configure;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Configure_ACK;
}

policy_state usbpd_policy_ufp_vdm_configure_nak(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_UFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Responder_NAK;
	policy->tx_data_obj[0].structured_vdm.command = DisplayPort_Configure;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		if (power_role == USBPD_SINK)
			return PE_SNK_Ready;
		else
			return PE_SRC_Ready;
	}
	return PE_UFP_VDM_Configure_NAK;
}

/* the end ufp */

policy_state usbpd_policy_dfp_vdm_identity_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover Identity request
	Start VDMResponseTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = Discover_Identity;

	pd_data->counter.discover_identity_counter++;
	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_Identity_Request;
		if (usbpd_wait_msg(pd_data, VDM_DISCOVER_IDENTITY,
					tVDMSenderResponse)) {
			pd_data->counter.discover_identity_counter = 0;

			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_Identity_ACKed;
			else if (policy->rx_data_obj[0].structured_vdm.command_type == Responder_NAK
				|| policy->rx_data_obj[0].structured_vdm.command_type == Responder_BUSY)
				return PE_DFP_VDM_Identity_NAKed;
		}
	}

	if (pd_data->counter.swap_hard_reset_counter > USBPD_nHardResetCount)
		return Error_Recovery;

	return PE_SRC_Hard_Reset;
}

static policy_state usbpd_policy_dfp_vdm_response(struct policy_data *policy,
					usbpd_manager_event_type event)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	usbpd_manager_inform_event(pd_data, event);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SINK)
		return PE_SNK_Ready;
	else
		return PE_SRC_Ready;
}

policy_state usbpd_policy_dfp_vdm_identity_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of identity
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy,
				MANAGER_DISCOVER_IDENTITY_ACKED);
}

policy_state usbpd_policy_dfp_vdm_identity_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of result
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy,
				MANAGER_DISCOVER_IDENTITY_NAKED);
}

policy_state usbpd_policy_dfp_vdm_svids_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover SVIDs request
	Start VDMResponseTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = Discover_SVIDs;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_SVIDs_Request;
		if (usbpd_wait_msg(pd_data, VDM_DISCOVER_SVID,
					tVDMSenderResponse)) {
			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_SVIDs_ACKed;
		}
	}
	return PE_DFP_VDM_SVIDs_NAKed;
}

policy_state usbpd_policy_dfp_vdm_svids_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of SVIDs
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy,
				MANAGER_DISCOVER_SVID_ACKED);
}

policy_state usbpd_policy_dfp_vdm_svids_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of result
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy,
				MANAGER_DISCOVER_SVID_NAKED);
}

policy_state usbpd_policy_dfp_vdm_modes_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	struct usbpd_manager_data *manager = &pd_data->manager;
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Discover Modes request
	Start VDMResponseTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = manager->SVID_0;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = Discover_Modes;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_Modes_Request;
		if (usbpd_wait_msg(pd_data, VDM_DISCOVER_MODE,
					tVDMSenderResponse)) {
			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_Modes_ACKed;
		}
	}
	return PE_DFP_VDM_Modes_NAKed;
}

policy_state usbpd_policy_dfp_vdm_modes_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of Modes
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy,
				MANAGER_DISCOVER_MODE_ACKED);
}

policy_state usbpd_policy_dfp_vdm_modes_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of result
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy,
				MANAGER_DISCOVER_MODE_NAKED);
}

policy_state usbpd_policy_dfp_vdm_entry_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	struct usbpd_manager_data *manager = &pd_data->manager;
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Mode Entry request
	Start VDMModeEntryTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].object = 0;
	policy->tx_data_obj[0].structured_vdm.svid = manager->SVID_0;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;/* Todo select which_mode */
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = Enter_Mode;

	/* TODO: obj_pos , vdo should be set by device manager */

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_Mode_Entry_Request;
		if (usbpd_wait_msg(pd_data, VDM_ENTER_MODE,
					tVDMWaitModeEntry)) {
			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_Mode_Entry_ACKed;
		}
	}
	return PE_DFP_VDM_Mode_Entry_NAKed;
}

policy_state usbpd_policy_dfp_vdm_entry_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Request DPM to enter the mode
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_ENTER_MODE_ACKED);
}

policy_state usbpd_policy_dfp_vdm_entry_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of reason for failure
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_ENTER_MODE_NAKED);
}

policy_state usbpd_policy_dfp_vdm_exit_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	Actions on entry:
	Send Exit Mode request
	Start VDMModeExitTimer
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
/*	policy->tx_data_obj[0].structured_vdm.obj_pos = which_mode; */
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = Exit_Mode;

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_Mode_Exit_Request;
		if (usbpd_wait_msg(pd_data, VDM_EXIT_MODE,
					tVDMWaitModeExit)) {
			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_Mode_Exit_ACKed;
		}
	}
	return PE_DFP_VDM_Mode_Exit_NAKed;
}

policy_state usbpd_policy_dfp_vdm_exit_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of ACK
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_EXIT_MODE_ACKED);
}

policy_state usbpd_policy_dfp_vdm_exit_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform DPM of NAK
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_EXIT_MODE_NAKED);
}

policy_state usbpd_policy_dfp_vdm_attention_request(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	Actions on entry:
	Inform Device Policy Manager of Attention Command request
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_ATTENTION_REQUEST);
}

policy_state usbpd_policy_dfp_vdm_status_update(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);
	pd_data->phy_ops.set_check_msg_pass(pd_data, CHECK_MSG_PASS);


	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 2;

	policy->tx_data_obj[0].object = 0;
	policy->tx_data_obj[0].structured_vdm.svid = PD_SID_1;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;/* Todo select which_mode */
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = DisplayPort_Status_Update;

	/* second object for vdo */
	policy->tx_data_obj[1].object = 0;
	policy->tx_data_obj[1].displayport_status.port_connected = 1;

	/* TODO: obj_pos , vdo should be set by device manager */

	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_Status_Update;
		if (usbpd_wait_msg(pd_data, MSG_PASS,
					tVDMWaitModeEntry)) {
			pd_data->phy_ops.set_check_msg_pass(pd_data, NONE_CHECK_MSG_PASS);
			pr_info("%s : command(%d), command_type(%d), obj_pos(%d), version(%d), vdm_type(%d)\n",
			__func__, policy->rx_data_obj[0].structured_vdm.command,
			policy->rx_data_obj[0].structured_vdm.command_type,
			policy->rx_data_obj[0].structured_vdm.obj_pos,
			policy->rx_data_obj[0].structured_vdm.version,
			policy->rx_data_obj[0].structured_vdm.vdm_type);

			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_Status_Update_ACKed;
		}
	}
	pd_data->phy_ops.set_check_msg_pass(pd_data, NONE_CHECK_MSG_PASS);

	return PE_DFP_VDM_Status_Update_NAKed;
}

policy_state usbpd_policy_dfp_vdm_status_update_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_STATUS_UPDATE_ACKED);
}

policy_state usbpd_policy_dfp_vdm_status_update_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_STATUS_UPDATE_NAKED);
}

policy_state usbpd_policy_dfp_vdm_displayport_configure(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);
	pd_data->phy_ops.set_check_msg_pass(pd_data, CHECK_MSG_PASS);

	policy->tx_msg_header.msg_type = USBPD_Vendor_Defined;
	policy->tx_msg_header.port_data_role = USBPD_DFP;
	policy->tx_msg_header.port_power_role = power_role;
	policy->tx_msg_header.num_data_objs = 2;

	policy->tx_data_obj[0].structured_vdm.svid = PD_SID_1;
	policy->tx_data_obj[0].structured_vdm.vdm_type = Structured_VDM;
	policy->tx_data_obj[0].structured_vdm.version = 0;
	policy->tx_data_obj[0].structured_vdm.obj_pos = 1;/* Todo select which_mode */
	policy->tx_data_obj[0].structured_vdm.command_type = Initiator;
	policy->tx_data_obj[0].structured_vdm.command = DisplayPort_Configure;

	/* second object for vdo */
	policy->tx_data_obj[1].object = 0;
	policy->tx_data_obj[1].displayport_configurations.select_configuration = USB_U_AS_UFP_D;
	policy->tx_data_obj[1].displayport_configurations.displayport_protocol = DP_V_1_3;
	policy->tx_data_obj[1].displayport_configurations.ufp_u_pin_assignment = PIN_ASSIGNMENT_D;

	/* TODO: obj_pos , vdo should be set by device manager */
	if (usbpd_send_msg(pd_data, &policy->tx_msg_header,
				policy->tx_data_obj)) {
		pd_data->policy.state = PE_DFP_VDM_DisplayPort_Configure;
		if (usbpd_wait_msg(pd_data, MSG_PASS,
					tVDMWaitModeEntry)) {
			pd_data->phy_ops.set_check_msg_pass(pd_data, NONE_CHECK_MSG_PASS);
			if (policy->rx_data_obj[0].structured_vdm.command_type
					== Responder_ACK)
				return PE_DFP_VDM_DisplayPort_Configure_ACKed;
		}
	}
	pd_data->phy_ops.set_check_msg_pass(pd_data, NONE_CHECK_MSG_PASS);

	return PE_DFP_VDM_DisplayPort_Configure_NAKed;
}

policy_state usbpd_policy_dfp_vdm_displayport_configure_acked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_DisplayPort_Configure_ACKED);
}

policy_state usbpd_policy_dfp_vdm_displayport_configure_naked(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	return usbpd_policy_dfp_vdm_response(policy, MANAGER_DisplayPort_Configure_NACKED);
}

policy_state usbpd_policy_dfp_uvdm_send_message(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	struct usbpd_manager_data *manager = &pd_data->manager;
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->phy_ops.set_check_msg_pass(pd_data, CHECK_MSG_PASS);

	usbpd_send_msg(pd_data, &manager->uvdm_msg_header, manager->uvdm_data_obj);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_dfp_uvdm_receive_message(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int power_role = 0;

	/**********************************************
	**********************************************/

	dev_info(pd_data->dev, "%s\n", __func__);

	usbpd_manager_inform_event(pd_data, MANAGER_UVDM_RECEIVE_MESSAGE);

	pd_data->phy_ops.get_power_role(pd_data, &power_role);

	if (power_role == USBPD_SOURCE)
		return PE_SRC_Ready;
	else
		return PE_SNK_Ready;
}

policy_state usbpd_policy_dr_src_get_source_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_DR_SRC_Get_Source_Cap;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	get source capabilities request from Device Policy Manager

	Actions on entry:
	Send Get_Source_Cap message
	Initialize and run SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 3) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header, USBPD_Get_Source_Cap, data_role, USBPD_SOURCE);

	/* 4) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 5) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_DR_SRC_Get_Source_Cap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SRC_Send_Soft_Reset;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_SRC_CAP)) {
			ret = PE_SRC_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT)) {
			ret = PE_SRC_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= tSenderResponse) {
			ret = PE_SRC_Ready;
			break;
		}
	}

	/**********************************************
	Actions on exit:
	Pass source capabilities/outcome to Device Policy Manager

	Source capabilities message received
	| SenderResponseTimer Timeout | Reject message received
	**********************************************/

	return ret;
}

policy_state usbpd_policy_dr_src_give_sink_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int data_role = 0;
	int ret = PE_DR_SRC_Give_Sink_Cap;
	int ms = 0;

	/**********************************************
	Actions on entry:
	Get present sink capabilities from Device Policy Manager
	Send Capabilities message (based on Device Policy Manager response)
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 3) Sink Cap Message Setting */
	policy->tx_msg_header.word = pd_data->sink_msg_header.word;
	policy->tx_msg_header.port_data_role = data_role;
	policy->tx_data_obj[0].object = pd_data->sink_data_obj[0].object;
	policy->tx_data_obj[1].object = pd_data->sink_data_obj[1].object;

	/* 4) Send Message */
	usbpd_send_msg(pd_data, &policy->tx_msg_header, policy->tx_data_obj);

	/* 5) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 6) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_DR_SRC_Give_Sink_Cap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			dev_info(pd_data->dev, "got dr_src_give_sink_cap MSG_GOODCRC.\n");
			ret = PE_SRC_Ready;
			break;
		}

		if (ms >= 5) {
			ret = PE_SRC_Send_Soft_Reset;
			dev_info(pd_data->dev, "got dr_src_give_sink_cap Timer1_overflag.\n");
			break;
		}
	}

	/* 8) Sink Capabilities message sent */
	return ret;
}

policy_state usbpd_policy_dr_snk_get_sink_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_DR_SNK_Get_Sink_Cap;
	int data_role = 0;
	int ms = 0;

	/**********************************************
	get sink capabilities request from Device Policy Manager

	Actions on entry:
	Send Get_Sink_Cap message
	Initialize and run
	SenderResponseTimer
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 3) Send Message */
	usbpd_send_ctrl_msg(pd_data, &policy->tx_msg_header,
							USBPD_Get_Sink_Cap, data_role, USBPD_SINK);

	/* 4) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 5) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0) {
			ret = PE_DR_SNK_Get_Sink_Cap;
			break;
		}
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_ERROR)) {
			ret = PE_SNK_Send_Soft_Reset;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_SNK_CAP)) {
			ret = PE_SNK_Ready;
			break;
		}

		if (pd_data->phy_ops.get_status(pd_data, MSG_REJECT)) {
			ret = PE_SNK_Ready;
			break;
		}

		/* TimeOver Check */
		if (ms >= tSenderResponse) {
			ret = PE_SNK_Ready;
			break;
		}
	}

	/**********************************************
	Actions on exit:
	Pass sink capabilities/outcome to
	Device Policy Manager

	Sink capabilities message received
	| SenderResponseTimer Timeout | Reject message received
	**********************************************/

	return ret;
}

policy_state usbpd_policy_dr_snk_give_source_cap(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_DR_SNK_Give_Source_Cap;
	int data_role = 0;
	int ms = 0;
	/**********************************************
	Actions on entry:
	Request source capabilities from
	Device Policy Manager
	Send Capabilities message
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	if (!pd_data->id_matched)
		return PE_SNK_Ready;

#if 0
	/* 2) Message ID Check and Ignored */
	if (pd_data->phy_ops.compare_message_id(pd_data) != 0) {
		dev_info(pd_data->dev, "After compare_message_id not Match move to PE_SNK_Ready.\n");
		return PE_SNK_Ready;
	} else {
		dev_info(pd_data->dev, "After compare_message_id Match move to PE_SNK_Ready.\n");
	}
#endif

	/* 3) Read Data Role */
	pd_data->phy_ops.get_data_role(pd_data, &data_role);

	/* 4) Message Setting */
	policy->tx_msg_header.msg_type = USBPD_Source_Capabilities;
	policy->tx_msg_header.port_data_role = data_role;
	policy->tx_msg_header.port_power_role = USBPD_SINK;
	policy->tx_msg_header.num_data_objs = 1;

	policy->tx_data_obj[0].power_data_obj.max_current = 1500 / 10;
	policy->tx_data_obj[0].power_data_obj.voltage = 5000 / 50;
	policy->tx_data_obj[0].power_data_obj.peak_current = 0;
	policy->tx_data_obj[0].power_data_obj.rsvd = 0;
	policy->tx_data_obj[0].power_data_obj.data_role_swap = 1;
	policy->tx_data_obj[0].power_data_obj.usb_comm_capable = 1;
	policy->tx_data_obj[0].power_data_obj.externally_powered = 0;
	policy->tx_data_obj[0].power_data_obj.usb_suspend_support = 1;
	policy->tx_data_obj[0].power_data_obj.dual_role_power = 1;
	policy->tx_data_obj[0].power_data_obj.supply = 0;

	/* 5) Send Message */
	usbpd_send_msg(pd_data, &policy->tx_msg_header, policy->tx_data_obj);

	/* 6) Start Timer */
	usbpd_timer1_start(pd_data);

	/* 7) Wait Message or State */
	while (1) {
		if (policy->plug_valid == 0)
			break;
		ms = usbpd_check_time1(pd_data);
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			ret = PE_SNK_Ready;
			break;
		}

		if (ms >= 5) {
			ret = PE_SNK_Send_Soft_Reset;
			break;
		}
	}

	return ret;
}
#if 1 //JETSEO
policy_state usbpd_policy_bist_receive_mode(struct policy_data *policy)
{

	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_BIST_Receive_Mode;

	/**********************************************
	Actions on entry:
	Tell Protocol Layer to go to BIST
	Receive Mode
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/* 2) Interrupt Status All Mask */

	/**********************************************
	Actions on exit:
	Goto bist frame received state
	or Hard Reset signaling received
	**********************************************/
	ret = PE_BIST_Frame_Received;

	return ret;

}

policy_state usbpd_policy_bist_frame_received(struct policy_data *policy)
{

	struct usbpd_data *pd_data = policy_to_usbpd(policy);
	int ret = PE_BIST_Frame_Received;

	/**********************************************
	Actions on entry:
	Consume Frame
	**********************************************/

	/* 1) PD State Inform to AP */
	dev_info(pd_data->dev, "%s\n", __func__);

	/**********************************************
	Actions on exit:
    Detach
    or Hard Reset signaling received
	**********************************************/

	return ret;

}
#endif
policy_state usbpd_error_recovery(struct policy_data *policy)
{
	struct usbpd_data *pd_data = policy_to_usbpd(policy);

	/**********************************************
	**********************************************/

	dev_err(pd_data->dev, "%s\n", __func__);

	return Error_Recovery;
}

void usbpd_policy_work(struct work_struct *work)
{
	struct usbpd_data *pd_data = container_of(work, struct usbpd_data,
			worker);
	struct policy_data *policy = &pd_data->policy;
	int power_role = 0;
	policy_state next_state = policy->state;
	policy_state saved_state;

	do {
		if (!policy->plug_valid) {
			pr_info("%s : usbpd cable is empty\n", __func__);
			break;
		}

		if (policy->rx_hardreset || policy->rx_softreset
				|| policy->plug) {
			saved_state = 0;
			next_state = 0; /* default */
		}
		saved_state = next_state;
		switch (next_state) {
		case PE_SRC_Startup:
			next_state = usbpd_policy_src_startup(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SRC_Discovery:
			next_state = usbpd_policy_src_discovery(policy);
			break;
		case PE_SRC_Send_Capabilities:
			next_state = usbpd_policy_src_send_capabilities(policy);
			break;
		case PE_SRC_Negotiate_Capability:
			next_state = usbpd_policy_src_negotiate_capability(policy);
			break;
		case PE_SRC_Transition_Supply:
			next_state = usbpd_policy_src_transition_supply(policy);
			break;
		case PE_SRC_Ready:
			next_state = usbpd_policy_src_ready(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SRC_Disabled:
			next_state = usbpd_policy_src_disabled(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SRC_Capability_Response:
			next_state = usbpd_policy_src_capability_response(policy);
			break;
		case PE_SRC_Hard_Reset:
			next_state = usbpd_policy_src_hard_reset(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SRC_Hard_Reset_Received:
			next_state = usbpd_policy_src_hard_reset_received(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SRC_Transition_to_default:
			next_state = usbpd_policy_src_transition_to_default(policy);
			break;
		case PE_SRC_Give_Source_Cap:
			next_state = usbpd_policy_src_give_source_cap(policy);
			break;
		case PE_SRC_Get_Sink_Cap:
			next_state = usbpd_policy_src_get_sink_cap(policy);
			break;
		case PE_SRC_Wait_New_Capabilities:
			next_state = usbpd_policy_src_wait_new_capabilities(policy);
			break;
		case PE_SRC_Send_Soft_Reset:
			next_state = usbpd_policy_src_send_soft_reset(policy);
			break;
		case PE_SRC_Soft_Reset:
			next_state = usbpd_policy_src_soft_reset(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;

		case PE_SNK_Startup:
			next_state = usbpd_policy_snk_startup(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SNK_Discovery:
			next_state = usbpd_policy_snk_discovery(policy);
			break;
		case PE_SNK_Wait_for_Capabilities:
			next_state = usbpd_policy_snk_wait_for_capabilities(policy);
			break;
		case PE_SNK_Evaluate_Capability:
			next_state = usbpd_policy_snk_evaluate_capability(policy);
			break;
		case PE_SNK_Select_Capability:
			next_state = usbpd_policy_snk_select_capability(policy);
			break;
		case PE_SNK_Transition_Sink:
			next_state = usbpd_policy_snk_transition_sink(policy);
			break;
		case PE_SNK_Ready:
			next_state = usbpd_policy_snk_ready(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SNK_Hard_Reset:
			next_state = usbpd_policy_snk_hard_reset(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SNK_Transition_to_default:
			next_state = usbpd_policy_snk_transition_to_default(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SNK_Give_Sink_Cap:
			next_state = usbpd_policy_snk_give_sink_cap(policy);
			break;
		case PE_SNK_Get_Source_Cap:
			next_state = usbpd_policy_snk_get_source_cap(policy);
			break;
		case PE_SNK_Send_Soft_Reset:
			next_state = usbpd_policy_snk_send_soft_reset(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_SNK_Soft_Reset:
			next_state = usbpd_policy_snk_soft_reset(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_DRS_Evaluate_Port:
			next_state = usbpd_policy_drs_evaluate_port(policy);
			break;
		case PE_DRS_Evaluate_Send_Port:
			next_state = usbpd_policy_drs_evaluate_send_port(policy);
			break;
		case PE_DRS_DFP_UFP_Evaluate_DR_Swap:
			next_state = usbpd_policy_drs_dfp_ufp_evaluate_dr_swap(policy);
			break;
		case PE_DRS_DFP_UFP_Accept_DR_Swap:
			next_state = usbpd_policy_drs_dfp_ufp_accept_dr_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_DRS_DFP_UFP_Change_to_UFP:
			next_state = usbpd_policy_drs_dfp_ufp_change_to_ufp(policy);
			break;
		case PE_DRS_DFP_UFP_Send_DR_Swap:
			next_state = usbpd_policy_drs_dfp_ufp_send_dr_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_DRS_DFP_UFP_Reject_DR_Swap:
			next_state = usbpd_policy_drs_dfp_ufp_reject_dr_swap(policy);
			break;
		case PE_DRS_UFP_DFP_Evaluate_DR_Swap:
			next_state = usbpd_policy_drs_ufp_dfp_evaluate_dr_swap(policy);
			break;
		case PE_DRS_UFP_DFP_Accept_DR_Swap:
			next_state = usbpd_policy_drs_ufp_dfp_accept_dr_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_DRS_UFP_DFP_Change_to_DFP:
			next_state = usbpd_policy_drs_ufp_dfp_change_to_dfp(policy);
			break;
		case PE_DRS_UFP_DFP_Send_DR_Swap:
			next_state = usbpd_policy_drs_ufp_dfp_send_dr_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_DRS_UFP_DFP_Reject_DR_Swap:
			next_state = usbpd_policy_drs_ufp_dfp_reject_dr_swap(policy);
			break;

		case PE_PRS_SRC_SNK_Reject_PR_Swap:
			next_state = usbpd_policy_prs_src_snk_reject_pr_swap(policy);
			break;
		case PE_PRS_SRC_SNK_Evaluate_Swap:
			next_state = usbpd_policy_prs_src_snk_evaluate_swap(policy);
			break;
		case PE_PRS_SRC_SNK_Send_Swap:
			next_state = usbpd_policy_prs_src_snk_send_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_PRS_SRC_SNK_Accept_Swap:
			next_state = usbpd_policy_prs_src_snk_accept_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_PRS_SRC_SNK_Transition_off:
			next_state = usbpd_policy_prs_src_snk_transition_to_off(policy);
			break;
		case PE_PRS_SRC_SNK_Assert_Rd:
			next_state = usbpd_policy_prs_src_snk_assert_rd(policy);
			break;
		case PE_PRS_SRC_SNK_Wait_Source_on:
			next_state = usbpd_policy_prs_src_snk_wait_source_on(policy);
			break;
		case PE_PRS_SNK_SRC_Reject_Swap:
			next_state = usbpd_policy_prs_snk_src_reject_swap(policy);
			break;
		case PE_PRS_SNK_SRC_Evaluate_Swap:
			next_state = usbpd_policy_prs_snk_src_evaluate_swap(policy);
			break;
		case PE_PRS_SNK_SRC_Send_Swap:
			next_state = usbpd_policy_prs_snk_src_send_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_PRS_SNK_SRC_Accept_Swap:
			next_state = usbpd_policy_prs_snk_src_accept_swap(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;
		case PE_PRS_SNK_SRC_Transition_off:
			next_state = usbpd_policy_prs_snk_src_transition_to_off(policy);
			break;
		case PE_PRS_SNK_SRC_Assert_Rp:
			next_state = usbpd_policy_prs_snk_src_assert_rp(policy);
			break;
		case PE_PRS_SNK_SRC_Source_on:
			next_state = usbpd_policy_prs_snk_src_source_on(policy);
			break;
		case PE_VCS_Evaluate_Swap:
			next_state = usbpd_policy_vcs_evaluate_swap(policy);
			break;
		case PE_VCS_Accept_Swap:
			next_state = usbpd_policy_vcs_accept_swap(policy);
			break;
		case PE_VCS_Wait_for_VCONN:
			next_state = usbpd_policy_vcs_wait_for_vconn(policy);
			break;
		case PE_VCS_Turn_Off_VCONN:
			next_state = usbpd_policy_vcs_turn_off_vconn(policy);
			break;
		case PE_VCS_Turn_On_VCONN:
			next_state = usbpd_policy_vcs_turn_on_vconn(policy);
			break;
		case PE_VCS_Send_PS_RDY:
			next_state = usbpd_policy_vcs_send_ps_rdy(policy);
			break;
		case PE_VCS_Send_Swap:
			next_state = usbpd_policy_vcs_send_swap(policy);
			break;
		case PE_VCS_Reject_VCONN_Swap:
			next_state = usbpd_policy_vcs_reject_vconn_swap(policy);
			break;

		case PE_UFP_VDM_Get_Identity:
			next_state = usbpd_policy_ufp_vdm_get_identity(policy);
			break;
		case PE_UFP_VDM_Send_Identity:
			next_state = usbpd_policy_ufp_vdm_send_identity(policy);
			break;
		case PE_UFP_VDM_Get_Identity_NAK:
			next_state = usbpd_policy_ufp_vdm_get_identity_nak(policy);
			break;
		case PE_UFP_VDM_Get_SVIDs:
			next_state = usbpd_policy_ufp_vdm_get_svids(policy);
			break;
		case PE_UFP_VDM_Send_SVIDs:
			next_state = usbpd_policy_ufp_vdm_send_svids(policy);
			break;
		case PE_UFP_VDM_Get_SVIDs_NAK:
			next_state = usbpd_policy_ufp_vdm_get_svids_nak(policy);
			break;
		case PE_UFP_VDM_Get_Modes:
			next_state = usbpd_policy_ufp_vdm_get_modes(policy);
			break;
		case PE_UFP_VDM_Send_Modes:
			next_state = usbpd_policy_ufp_vdm_send_modes(policy);
			break;
		case PE_UFP_VDM_Get_Modes_NAK:
			next_state = usbpd_policy_ufp_vdm_get_modes_nak(policy);
			break;
		case PE_UFP_VDM_Evaluate_Mode_Entry:
			next_state = usbpd_policy_ufp_vdm_evaluate_mode_entry(policy);
			break;
		case PE_UFP_VDM_Mode_Entry_ACK:
			next_state = usbpd_policy_ufp_vdm_mode_entry_ack(policy);
			break;
		case PE_UFP_VDM_Mode_Entry_NAK:
			next_state = usbpd_policy_ufp_vdm_mode_entry_nak(policy);
			break;
		case PE_UFP_VDM_Mode_Exit:
			next_state = usbpd_policy_ufp_vdm_mode_exit(policy);
			break;
		case PE_UFP_VDM_Mode_Exit_ACK:
			next_state = usbpd_policy_ufp_vdm_mode_exit_ack(policy);
			break;
		case PE_UFP_VDM_Mode_Exit_NAK:
			next_state = usbpd_policy_ufp_vdm_mode_exit_nak(policy);
			break;
		case PE_UFP_VDM_Attention_Request:
			next_state = usbpd_policy_ufp_vdm_attention_request(policy);
			break;
		case PE_UFP_VDM_Evaluate_Status:
			next_state = usbpd_policy_ufp_vdm_evaluate_status(policy);
			break;
		case PE_UFP_VDM_Status_ACK:
			next_state = usbpd_policy_ufp_vdm_status_ack(policy);
			break;
		case PE_UFP_VDM_Status_NAK:
			next_state = usbpd_policy_ufp_vdm_status_nak(policy);
			break;
		case PE_UFP_VDM_Evaluate_Configure:
			next_state = usbpd_policy_ufp_vdm_evaluate_configure(policy);
			break;
		case PE_UFP_VDM_Configure_ACK:
			next_state = usbpd_policy_ufp_vdm_configure_ack(policy);
			break;
		case PE_UFP_VDM_Configure_NAK:
			next_state = usbpd_policy_ufp_vdm_configure_nak(policy);
			break;
		case PE_DFP_VDM_Identity_Request:
			next_state = usbpd_policy_dfp_vdm_identity_request(policy);
			break;
		case PE_DFP_VDM_Identity_ACKed:
			next_state = usbpd_policy_dfp_vdm_identity_acked(policy);
			break;
		case PE_DFP_VDM_Identity_NAKed:
			next_state = usbpd_policy_dfp_vdm_identity_naked(policy);
			break;
		case PE_DFP_VDM_SVIDs_Request:
			next_state = usbpd_policy_dfp_vdm_svids_request(policy);
			break;
		case PE_DFP_VDM_SVIDs_ACKed:
			next_state = usbpd_policy_dfp_vdm_svids_acked(policy);
			break;
		case PE_DFP_VDM_SVIDs_NAKed:
			next_state = usbpd_policy_dfp_vdm_svids_naked(policy);
			break;
		case PE_DFP_VDM_Modes_Request:
			next_state = usbpd_policy_dfp_vdm_modes_request(policy);
			break;
		case PE_DFP_VDM_Modes_ACKed:
			next_state = usbpd_policy_dfp_vdm_modes_acked(policy);
			break;
		case PE_DFP_VDM_Modes_NAKed:
			next_state = usbpd_policy_dfp_vdm_modes_naked(policy);
			break;
		case PE_DFP_VDM_Mode_Entry_Request:
			next_state = usbpd_policy_dfp_vdm_entry_request(policy);
			break;
		case PE_DFP_VDM_Mode_Entry_ACKed:
			next_state = usbpd_policy_dfp_vdm_entry_acked(policy);
			break;
		case PE_DFP_VDM_Mode_Entry_NAKed:
			next_state = usbpd_policy_dfp_vdm_entry_naked(policy);
			break;
		case PE_DFP_VDM_Mode_Exit_Request:
			next_state = usbpd_policy_dfp_vdm_exit_request(policy);
			break;
		case PE_DFP_VDM_Mode_Exit_ACKed:
			next_state = usbpd_policy_dfp_vdm_exit_acked(policy);
			break;
		case PE_DFP_VDM_Mode_Exit_NAKed:
			next_state = usbpd_policy_dfp_vdm_exit_naked(policy);
			break;
		case PE_DFP_VDM_Attention_Request:
			next_state = usbpd_policy_dfp_vdm_attention_request(policy);
			break;
		case PE_DFP_VDM_Status_Update:
			next_state = usbpd_policy_dfp_vdm_status_update(policy);
			break;
		case PE_DFP_VDM_Status_Update_ACKed:
			next_state = usbpd_policy_dfp_vdm_status_update_acked(policy);
			break;
		case PE_DFP_VDM_Status_Update_NAKed:
			next_state = usbpd_policy_dfp_vdm_status_update_naked(policy);
			break;
		case PE_DFP_VDM_DisplayPort_Configure:
			next_state = usbpd_policy_dfp_vdm_displayport_configure(policy);
			break;
		case PE_DFP_VDM_DisplayPort_Configure_ACKed:
			next_state = usbpd_policy_dfp_vdm_displayport_configure_acked(policy);
			break;
		case PE_DFP_VDM_DisplayPort_Configure_NAKed:
			next_state = usbpd_policy_dfp_vdm_displayport_configure_naked(policy);
			break;
		case PE_DFP_UVDM_Send_Message:
			next_state = usbpd_policy_dfp_uvdm_send_message(policy);
			break;
		case PE_DFP_UVDM_Receive_Message:
			next_state = usbpd_policy_dfp_uvdm_receive_message(policy);
			break;
		case PE_DR_SRC_Get_Source_Cap:
			next_state = usbpd_policy_dr_src_get_source_cap(policy);
			break;
		case PE_DR_SRC_Give_Sink_Cap:
			next_state = usbpd_policy_dr_src_give_sink_cap(policy);
			break;
		case PE_DR_SNK_Get_Sink_Cap:
			next_state = usbpd_policy_dr_snk_get_sink_cap(policy);
			break;
		case PE_DR_SNK_Give_Source_Cap:
			next_state = usbpd_policy_dr_snk_give_source_cap(policy);
			break;
#if 1	//JETSEO
		case PE_BIST_Receive_Mode:
			next_state = usbpd_policy_bist_receive_mode(policy);
			break;
		case PE_BIST_Frame_Received:
			next_state = usbpd_policy_bist_frame_received(policy);
			break;
#endif
		case Error_Recovery:
			next_state = usbpd_error_recovery(policy);
			store_usblog_notify(NOTIFY_FUNCSTATE, (void *)&next_state, NULL);
			break;

		default:
			pd_data->phy_ops.get_power_role(pd_data, &power_role);
			pr_info("%s, %d\n", __func__, power_role);

			if (power_role == USBPD_SINK) {
				pr_info("%s, SINK\n", __func__);
				if (policy->rx_hardreset) {
					policy->rx_hardreset = 0;
					next_state = PE_SNK_Transition_to_default;
				} else if (policy->rx_softreset) {
					policy->rx_softreset = 0;
					next_state = PE_SNK_Soft_Reset;
				} else if (policy->plug) {
					policy->plug = 0;
					next_state = PE_SNK_Startup;
				} else {
					next_state = PE_SNK_Startup;
				}
			} else {
				pr_info("%s, SOURCE\n", __func__);
				if (policy->rx_hardreset) {
					policy->rx_hardreset = 0;
					next_state = PE_SRC_Hard_Reset_Received;
				} else if (policy->rx_softreset) {
					policy->rx_softreset = 0;
					next_state = PE_SRC_Soft_Reset;
				} else if (policy->plug) {
					policy->plug = 0;
					next_state = PE_SRC_Startup;
				} else {
					next_state = PE_SRC_Startup;
				}
			}

			break;
		}
	dev_info(pd_data->dev, "%s saved state %x next_state %x \n", __func__, saved_state, next_state);
	} while (saved_state != next_state);

	policy->state = next_state;
	dev_info(pd_data->dev, "%s Finished\n", __func__);
}

void usbpd_init_policy(struct usbpd_data *pd_data)
{
	int i;
	struct policy_data *policy = &pd_data->policy;

	policy->state = 0;
	policy->rx_hardreset = 0;
	policy->rx_softreset = 0;
	policy->plug = 0;
	policy->rx_msg_header.word = 0;
	policy->tx_msg_header.word = 0;
	policy->modal_operation = 0;
	policy->sink_cap_received = 0;
	policy->send_sink_cap = 0;
	policy->txhardresetflag = 0;
	policy->pd_support = 0;
	for (i = 0; i < USBPD_MAX_COUNT_MSG_OBJECT; i++) {
		policy->rx_data_obj[i].object = 0;
		policy->tx_data_obj[i].object = 0;
	}
}

void usbpd_kick_policy_work(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);

	if (pd_data->policy_wqueue)
		queue_work(pd_data->policy_wqueue, &pd_data->worker);
	else
		schedule_work(&pd_data->worker);
}

void usbpd_cancel_policy_work(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);

	if (pd_data->policy_wqueue)
		flush_workqueue(pd_data->policy_wqueue);
}
