/*****************************************************************************
 *
 * Copyright (c) 2012 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include "debug.h"
#include "mlme.h"

#define SLSI_FAPI_NAN_ATTRIBUTE_PUT_U8(req, attribute, val) \
	{ \
		u16 attribute_len = 1; \
		struct sk_buff *req_p = req; \
		fapi_append_data((req_p), (u8 *)&(attribute), 2); \
		fapi_append_data((req_p), (u8 *)&attribute_len, 2); \
		fapi_append_data((req_p), (u8 *)&(val), 1); \
	}

#define SLSI_FAPI_NAN_ATTRIBUTE_PUT_U16(req, attribute, val) \
	{ \
		u16 attribute_len = 2; \
		__le16 le16val = cpu_to_le16(val); \
		struct sk_buff *req_p = req; \
		fapi_append_data((req_p), (u8 *)&(attribute), 2); \
		fapi_append_data((req_p), (u8 *)&attribute_len, 2); \
		fapi_append_data((req_p), (u8 *)&le16val, 2); \
	}

#define SLSI_FAPI_NAN_ATTRIBUTE_PUT_U32(req, attribute, val) \
	{ \
		u16 attribute_len = 4; \
		__le32 le32val = cpu_to_le32(val);\
		struct sk_buff *req_p = req; \
		fapi_append_data((req_p), (u8 *)&(attribute), 2); \
		fapi_append_data((req_p), (u8 *)&attribute_len, 2); \
		fapi_append_data((req_p), (u8 *)&le32val, 4); \
	}

#define SLSI_FAPI_NAN_ATTRIBUTE_PUT_DATA(req, attribute, val, val_len) \
	{ \
		u16 attribute_len = (val_len); \
		struct sk_buff *req_p = req; \
		fapi_append_data((req_p), (u8 *)&(attribute), 2); \
		fapi_append_data((req_p), (u8 *)&attribute_len, 2); \
		fapi_append_data((req_p), (val), (attribute_len)); \
	}

static u32 slsi_mlme_nan_append_tlv(struct sk_buff *req, u16 tag, u16 len, u8 *data)
{
	u8 *p;

	p = fapi_append_data_u16(req, tag);
	p = fapi_append_data_u16(req, len);
	p = fapi_append_data(req, data, len);
	if (p)
		return 0;
	return 1;
}

static u32 slsi_mlme_nan_append_config_tlv(struct sk_buff *req, u8 master_pref, u16 include_ps_id, u8 ps_id_count,
					   u16 include_ss_id, u8 ss_id_count, u16 rssi_window, u32 nmi_rand_interval,
					   u16 cluster_merge)
{
	u8 *p;

	p = fapi_append_data_u16(req, SLSI_NAN_TLV_TAG_CONFIGURATION);
	p = fapi_append_data_u16(req, 0x000f);
	p = fapi_append_data_u8(req, master_pref);

	/* publish service ID inclusion in beacon */
	p = fapi_append_data_bool(req, include_ps_id);
	p = fapi_append_data_u8(req, ps_id_count);

	/* subscribe service ID inclusion in beacon */
	p = fapi_append_data_bool(req, include_ss_id);
	p = fapi_append_data_u8(req, ss_id_count);

	p = fapi_append_data_u16(req, rssi_window);
	p = fapi_append_data_u32(req, nmi_rand_interval);
	p = fapi_append_data_u16(req, cluster_merge);

	if (p)
		return 0;
	return 1;
}

static u32 slsi_mlme_nan_append_band_specific_config(struct sk_buff *req, u16 tag, u8 rssi_close, u8 rssi_middle,
						     u8 rssi_proximity, u8 dwell_time, u16 scan_period,
						     u16 use_dw_int_val, u8 dw_interval)
{
	u8 *p;

	p = fapi_append_data_u16(req, tag);
	p = fapi_append_data_u16(req, 0x0009);
	p = fapi_append_data_u8(req, rssi_close);
	p = fapi_append_data_u8(req, rssi_middle);
	p = fapi_append_data_u8(req, rssi_proximity);
	p = fapi_append_data_u8(req, dwell_time);
	p = fapi_append_data_u16(req, scan_period);
	p = fapi_append_data_bool(req, use_dw_int_val);
	p = fapi_append_data_u8(req, dw_interval);
	if (p)
		return 0;
	return 1;
}

static u32 slsi_mlme_nan_append_2g4_band_specific_config(struct sk_buff *req, u8 rssi_close, u8 rssi_middle,
							 u8 rssi_proximity, u8 dwell_time, u16 scan_period,
							 u16 use_dw_int_val, u8 dw_interval)
{
	return slsi_mlme_nan_append_band_specific_config(req, SLSI_NAN_TLV_TAG_2G4_BAND_SPECIFIC_CONFIG, rssi_close,
							 rssi_middle, rssi_proximity, dwell_time, scan_period,
							 use_dw_int_val, dw_interval);
}

static u32 slsi_mlme_nan_append_5g_band_specific_config(struct sk_buff *req, u8 rssi_close, u8 rssi_middle,
							u8 rssi_proximity, u8 dwell_time, u16 scan_period,
							u16 use_dw_int_val, u8 dw_interval)
{
	return slsi_mlme_nan_append_band_specific_config(req, SLSI_NAN_TLV_TAG_5G_BAND_SPECIFIC_CONFIG, rssi_close,
							 rssi_middle, rssi_proximity, dwell_time, scan_period,
							 use_dw_int_val, dw_interval);
}

static u32 slsi_mlme_nan_append_discovery_config(struct sk_buff *req, u8 sd_type, u8 tx_type, u16 ttl, u16 dw_period,
						 u8 dw_count, u8 disc_match_ind, u16 use_rssi_thres, u16 ranging_req,
						 u16 data_path_req)
{
	u8 *p;

	p = fapi_append_data_u16(req, SLSI_NAN_TLV_TAG_DISCOVERY_COMMON_SPECIFIC);
	p = fapi_append_data_u16(req, 0x000e);
	p = fapi_append_data_u8(req, sd_type);
	p = fapi_append_data_u8(req, tx_type);
	p = fapi_append_data_u16(req, ttl);
	p = fapi_append_data_u16(req, dw_period);
	p = fapi_append_data_u8(req, dw_count);
	p = fapi_append_data_u8(req, disc_match_ind);
	p = fapi_append_data_bool(req, use_rssi_thres);
	p = fapi_append_data_bool(req, ranging_req);
	p = fapi_append_data_bool(req, data_path_req);
	if (p)
		return 0;
	return 1;
}

static u32 slsi_mlme_nan_append_subscribe_specific(struct sk_buff *req, u8 srf_type, u16 respond_if_in_address_set,
						   u16 use_srf, u16 ssi_required_for_match)
{
	u8 *p;

	p = fapi_append_data_u16(req, SLSI_NAN_TLV_TAG_SUBSCRIBE_SPECIFIC);
	p = fapi_append_data_u16(req, 0x0007);
	p = fapi_append_data_u8(req, srf_type);
	p = fapi_append_data_u16(req, respond_if_in_address_set);
	p = fapi_append_data_u16(req, use_srf);
	p = fapi_append_data_u16(req, ssi_required_for_match);
	if (p)
		return 0;
	return 1;
}

static u32 slsi_mlme_nan_append_address_set(struct sk_buff *req, u16 count, u8 *addresses)
{
	if (!count)
		return 0;
	return slsi_mlme_nan_append_tlv(req, SLSI_NAN_TLV_TAG_INTERFACE_ADDRESS_SET, count * 6, addresses);
}

static u32 slsi_mlme_nan_append_service_name(struct sk_buff *req, u16 len, u8 *data)
{
	if (!len)
		return 0;
	return slsi_mlme_nan_append_tlv(req, SLSI_NAN_TLV_TAG_SERVICE_NAME, len, data);
}

static u32 slsi_mlme_nan_append_service_specific_info(struct sk_buff *req, u16 len, u8 *data)
{
	if (!len)
		return 0;
	return slsi_mlme_nan_append_tlv(req, SLSI_NAN_TLV_TAG_SERVICE_SPECIFIC_INFO, len, data);
}

static u32 slsi_mlme_nan_append_ext_service_specific_info(struct sk_buff *req, u16 len, u8 *data)
{
	if (!len)
		return 0;
	return slsi_mlme_nan_append_tlv(req, SLSI_NAN_TLV_TAG_EXT_SERVICE_SPECIFIC_INFO, len, data);
}

static u32 slsi_mlme_nan_append_tx_match_filter(struct sk_buff *req, u16 len, u8 *data)
{
	if (!len)
		return 0;
	return slsi_mlme_nan_append_tlv(req, SLSI_NAN_TLV_TAG_TX_MATCH_FILTER, len, data);
}

static u32 slsi_mlme_nan_append_rx_match_filter(struct sk_buff *req, u16 len, u8 *data)
{
	if (!len)
		return 0;
	return slsi_mlme_nan_append_tlv(req, SLSI_NAN_TLV_TAG_RX_MATCH_FILTER, len, data);
}

static u32 slsi_mlme_nan_append_data_path_sec(struct sk_buff *req, struct slsi_nan_security_info *sec_info)
{
	u8 *p, *len_p;
	u32 pmk = 0;
	u8 passphrase_len = 0;
	u8 *passphrase;
	u8 sec_type = sec_info->key_info.key_type;

	if (sec_info->key_info.key_type == 1) {
		pmk = *(u32 *)sec_info->key_info.body.pmk_info.pmk;
	} else if (sec_info->key_info.key_type == 2) {
		passphrase_len = sec_info->key_info.body.passphrase_info.passphrase_len;
		passphrase = sec_info->key_info.body.passphrase_info.passphrase;
	} else {
		sec_type = 0;
	}

	p = fapi_append_data_u16(req, SLSI_NAN_TLV_TAG_DATA_PATH_SECURITY);
	p = fapi_append_data_u16(req, 0x0007);
	len_p = p;
	p = fapi_append_data_u8(req, sec_type);
	p = fapi_append_data_u8(req, sec_info->cipher_type);
	p = fapi_append_data_u32(req, sec_info->key_info.key_type);
	p = fapi_append_data_u8(req, passphrase_len);
	if (passphrase_len)
		p = fapi_append_data(req, passphrase, passphrase_len);
	if (!p)
		return 1;
	*len_p = 7 + passphrase_len;
	return 0;
}

static u32 slsi_mlme_nan_append_ranging(struct sk_buff *req, struct slsi_nan_ranging_cfg *ranging_cfg)
{
	u8 *p;

	p = fapi_append_data_u16(req, SLSI_NAN_TLV_TAG_RANGING);
	p = fapi_append_data_u16(req, 0x0007);
	p = fapi_append_data_u32(req, ranging_cfg->ranging_interval_msec);
	p = fapi_append_data_u8(req, ranging_cfg->config_ranging_indications);
	p = fapi_append_data_u16(req, ranging_cfg->distance_ingress_mm / 10);
	p = fapi_append_data_u16(req, ranging_cfg->distance_egress_mm / 10);
	if (p)
		return 0;
	return 1;
}

static u32 slsi_mlme_nan_enable_fapi_data(struct sk_buff *req, struct slsi_hal_nan_enable_req *hal_req)
{
	u16 publish_id_inc, service_id_inc;
	u8  publish_id_inc_count = 0;
	u8  service_id_inc_count = 0;
	u8  rssi_close, rssi_middle, rssi_proximity;
	u16 rssi_window = hal_req->config_rssi_window_size ? hal_req->rssi_window_size_val : 8;
	u32 ret;

	/* NAN configuration TLV */
	publish_id_inc = hal_req->config_sid_beacon && (hal_req->sid_beacon_val & 0x01);
	if (publish_id_inc)
		publish_id_inc_count = hal_req->sid_beacon_val >> 1;
	service_id_inc = hal_req->config_subscribe_sid_beacon && (hal_req->subscribe_sid_beacon_val & 0x01);
	if (service_id_inc)
		service_id_inc_count = hal_req->subscribe_sid_beacon_val >> 1;
	ret = slsi_mlme_nan_append_config_tlv(req, hal_req->master_pref, publish_id_inc, publish_id_inc_count,
					      service_id_inc, service_id_inc_count, rssi_window,
					      hal_req->disc_mac_addr_rand_interval_sec, 1);
	if (ret) {
		SLSI_WARN_NODEV("Error append config TLV\n");
		return ret;
	}

	/* 2.4G NAN band specific config TLV*/
	rssi_close = hal_req->config_2dot4g_rssi_close ? hal_req->rssi_close_2dot4g_val : 0;
	rssi_middle = hal_req->config_2dot4g_rssi_middle ? hal_req->rssi_middle_2dot4g_val : 0;
	rssi_proximity = hal_req->config_2dot4g_rssi_proximity ? hal_req->rssi_proximity_2dot4g_val : 0;
	ret = slsi_mlme_nan_append_2g4_band_specific_config(req, rssi_close, rssi_middle, rssi_proximity,
							    hal_req->scan_params_val.dwell_time[0],
							    hal_req->scan_params_val.scan_period[0],
							    (u16)hal_req->config_2dot4g_dw_band,
							    hal_req->dw_2dot4g_interval_val);
	if (ret) {
		SLSI_WARN_NODEV("Error append 2.4G band specific TLV\n");
		return ret;
	}

	/* 5G NAN band specific config TLV*/
	if (hal_req->config_support_5g && hal_req->support_5g_val) {
		rssi_close = hal_req->config_5g_rssi_close ? hal_req->rssi_close_5g_val : 0;
		rssi_middle = hal_req->config_5g_rssi_middle ? hal_req->rssi_middle_5g_val : 0;
		rssi_proximity = hal_req->config_5g_rssi_close_proximity ? hal_req->rssi_close_proximity_5g_val : 0;
		ret = slsi_mlme_nan_append_5g_band_specific_config(req, rssi_close, rssi_middle, rssi_proximity,
								   hal_req->scan_params_val.dwell_time[1],
								   hal_req->scan_params_val.scan_period[1],
								   (u16)hal_req->config_5g_dw_band,
								   hal_req->dw_5g_interval_val);
		if (ret) {
			SLSI_WARN_NODEV("Error append 5G band specific TLV\n");
			return ret;
		}
	}

	return ret;
}

int slsi_mlme_nan_enable(struct slsi_dev *sdev, struct net_device *dev, struct slsi_hal_nan_enable_req *hal_req)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_oper_ctrl = 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");

	/* mbulk data length = 0x0f + 4 + 2 * (9 + 4) = 45*/
	req = fapi_alloc(mlme_nan_start_req, MLME_NAN_START_REQ, ndev_vif->ifnum, 45);
	if (!req) {
		SLSI_NET_ERR(dev, "fapi alloc failure\n");
		return -ENOMEM;
	}

	nan_oper_ctrl |= FAPI_NANOPERATIONCONTROL_MAC_ADDRESS_EVENT | FAPI_NANOPERATIONCONTROL_START_CLUSTER_EVENT |
			 FAPI_NANOPERATIONCONTROL_JOINED_CLUSTER_EVENT;

	fapi_set_u16(req, u.mlme_nan_start_req.nan_operation_control_flags, nan_oper_ctrl);

	r = slsi_mlme_nan_enable_fapi_data(req, hal_req);
	if (r) {
		SLSI_NET_ERR(dev, "Failed to construct mbulkdata\n");
		slsi_kfree_skb(req);
		return -EINVAL;
	}

	cfm = slsi_mlme_req_cfm(sdev, dev, req, MLME_NAN_START_CFM);
	if (!cfm)
		return -EIO;

	if (fapi_get_u16(cfm, u.mlme_nan_start_cfm.result_code) != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "MLME_NAN_START_CFM(result:0x%04x) ERROR\n",
			     fapi_get_u16(cfm, u.mlme_host_state_cfm.result_code));
		r = -EINVAL;
	}

	slsi_kfree_skb(cfm);
	return r;
}

static u32 slsi_mlme_nan_publish_fapi_data(struct sk_buff *req, struct slsi_hal_nan_publish_req *hal_req)
{
	u32 ret;

	ret = slsi_mlme_nan_append_discovery_config(req, hal_req->publish_type, hal_req->tx_type, hal_req->ttl,
						    hal_req->period, hal_req->publish_count, hal_req->publish_match_indicator,
						    (u16)hal_req->rssi_threshold_flag, (u16)0, (u16)0);
	if (ret) {
		SLSI_WARN_NODEV("Error append disovery config TLV\n");
		return ret;
	}

	if (hal_req->service_name_len) {
		ret = slsi_mlme_nan_append_service_name(req, hal_req->service_name_len, hal_req->service_name);
		if (ret) {
			SLSI_WARN_NODEV("Error append servicename TLV\n");
			return ret;
		}
	}

	if (hal_req->service_specific_info_len) {
		ret = slsi_mlme_nan_append_service_specific_info(req, hal_req->service_specific_info_len,
								 hal_req->service_specific_info);
		if (ret) {
			SLSI_WARN_NODEV("Error append servSpecInfo TLV\n");
			return ret;
		}
	}

	if (hal_req->sdea_service_specific_info_len) {
		ret = slsi_mlme_nan_append_ext_service_specific_info(req, hal_req->sdea_service_specific_info_len,
								     hal_req->sdea_service_specific_info);
		if (ret) {
			SLSI_WARN_NODEV("Error append extServSpecInfo TLV\n");
			return ret;
		}
	}

	if (hal_req->rx_match_filter_len) {
		ret = slsi_mlme_nan_append_rx_match_filter(req, hal_req->rx_match_filter_len, hal_req->rx_match_filter);
		if (ret) {
			SLSI_WARN_NODEV("Error append rx match filter TLV\n");
			return ret;
		}
	}

	if (hal_req->tx_match_filter_len) {
		ret = slsi_mlme_nan_append_tx_match_filter(req, hal_req->tx_match_filter_len, hal_req->tx_match_filter);
		if (ret) {
			SLSI_WARN_NODEV("Error append tx match filter TLV\n");
			return ret;
		}
	}

	ret = slsi_mlme_nan_append_data_path_sec(req, &hal_req->sec_info);
	if (ret) {
		SLSI_WARN_NODEV("Error append datapath sec TLV\n");
		return ret;
	}
	ret = slsi_mlme_nan_append_ranging(req, &hal_req->ranging_cfg);
	if (ret)
		SLSI_WARN_NODEV("Error append ranging config TLV\n");
	return ret;
}

int slsi_mlme_nan_publish(struct slsi_dev *sdev, struct net_device *dev, struct slsi_hal_nan_publish_req *hal_req,
			  u16 publish_id)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_sdf_flags = 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");
	if (hal_req) {
		/* discovery_config_tlv, datapath_sec_tlv, ranging_cfg_tlv */
		u16 length = 18 + 70 + 11;

		length += hal_req->service_name_len ? hal_req->service_name_len + 4 : 0;
		length += hal_req->service_specific_info_len ? hal_req->service_specific_info_len + 4 : 0;
		length += hal_req->rx_match_filter_len ? hal_req->rx_match_filter_len + 4 : 0;
		length += hal_req->tx_match_filter_len ? hal_req->tx_match_filter_len + 4 : 0;
		length += hal_req->sdea_service_specific_info_len ? hal_req->sdea_service_specific_info_len + 4 : 0;

		req = fapi_alloc(mlme_nan_publish_req, MLME_NAN_PUBLISH_REQ, ndev_vif->ifnum, length);
		if (!req) {
			SLSI_NET_ERR(dev, "fapi alloc failure\n");
			return -ENOMEM;
		}

		/* Set/Enable corresponding bits to disable any indications
		 * that follow a publish.
		 * BIT0 - Disable publish termination indication.
		 * BIT1 - Disable match expired indication.
		 * BIT2 - Disable followUp indication received (OTA).
		 */
		if (hal_req->recv_indication_cfg & BIT(0))
			nan_sdf_flags |= FAPI_NANSDFCONTROL_PUBLISH_END_EVENT;
		if (hal_req->recv_indication_cfg & BIT(1))
			nan_sdf_flags |= FAPI_NANSDFCONTROL_MATCH_EXPIRED_EVENT;
		if (hal_req->recv_indication_cfg & BIT(2))
			nan_sdf_flags |= FAPI_NANSDFCONTROL_RECEIVED_FOLLOWUP_EVENT;
	} else {
		req = fapi_alloc(mlme_nan_publish_req, MLME_NAN_PUBLISH_REQ, ndev_vif->ifnum, 0);
		if (!req) {
			SLSI_NET_ERR(dev, "fapi alloc failure\n");
			return -ENOMEM;
		}
	}

	fapi_set_u16(req, u.mlme_nan_publish_req.publish_id, publish_id);
	fapi_set_u16(req, u.mlme_nan_publish_req.nan_sdf_flags, nan_sdf_flags);

	if (hal_req) {
		r = slsi_mlme_nan_publish_fapi_data(req, hal_req);
		if (r) {
			SLSI_NET_ERR(dev, "Failed to construct mbulkdata\n");
			slsi_kfree_skb(req);
			return -EINVAL;
		}
	}

	cfm = slsi_mlme_req_cfm(sdev, dev, req, MLME_NAN_PUBLISH_CFM);
	if (!cfm)
		return -EIO;

	if (fapi_get_u16(cfm, u.mlme_nan_publish_cfm.result_code) != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "MLME_NAN_PUBLISH_CFM(result:0x%04x) ERROR\n",
			     fapi_get_u16(cfm, u.mlme_host_state_cfm.result_code));
		r = -EINVAL;
	}

	if (hal_req && !r)
		ndev_vif->nan.publish_id_map |= (u32)BIT(publish_id);
	else
		ndev_vif->nan.publish_id_map &= (u32)~BIT(publish_id);
	slsi_kfree_skb(cfm);
	return r;
}

static u32 slsi_mlme_nan_subscribe_fapi_data(struct sk_buff *req, struct slsi_hal_nan_subscribe_req *hal_req)
{
	u32 ret;

	ret = slsi_mlme_nan_append_subscribe_specific(req, hal_req->service_response_filter,
						      hal_req->service_response_include,
						      hal_req->use_service_response_filter,
						      hal_req->ssi_required_for_match_indication);
	if (ret) {
		SLSI_WARN_NODEV("Error append subscribe specific TLV\n");
		return ret;
	}

	ret = slsi_mlme_nan_append_discovery_config(req, hal_req->subscribe_type,
						    hal_req->subscribe_type ? 0 : 1, hal_req->ttl,
						    hal_req->period, hal_req->subscribe_count, hal_req->subscribe_match_indicator,
						    hal_req->rssi_threshold_flag, (u16)0, (u16)0);
	if (ret) {
		SLSI_WARN_NODEV("Error append discovery config TLV\n");
		return ret;
	}

	if (hal_req->service_name_len) {
		ret = slsi_mlme_nan_append_service_name(req, hal_req->service_name_len, hal_req->service_name);
		if (ret) {
			SLSI_WARN_NODEV("Error append service name TLV\n");
			return ret;
		}
	}

	if (hal_req->service_specific_info_len) {
		ret = slsi_mlme_nan_append_service_specific_info(req, hal_req->service_specific_info_len,
								 hal_req->service_specific_info);
		if (ret) {
			SLSI_WARN_NODEV("Error append servSpecInfo TLV\n");
			return ret;
		}
	}

	if (hal_req->rx_match_filter_len) {
		ret = slsi_mlme_nan_append_rx_match_filter(req, hal_req->rx_match_filter_len, hal_req->rx_match_filter);
		if (ret) {
			SLSI_WARN_NODEV("Error append rx match filter TLV\n");
			return ret;
		}
	}

	if (hal_req->tx_match_filter_len) {
		ret = slsi_mlme_nan_append_tx_match_filter(req, hal_req->tx_match_filter_len, hal_req->tx_match_filter);
		if (ret) {
			SLSI_WARN_NODEV("Error append tx match filter TLV\n");
			return ret;
		}
	}

	if (hal_req->sdea_service_specific_info_len) {
		ret = slsi_mlme_nan_append_ext_service_specific_info(req, hal_req->sdea_service_specific_info_len,
								     hal_req->sdea_service_specific_info);
		if (ret) {
			SLSI_WARN_NODEV("Error append extServSpecInfo TLV\n");
			return ret;
		}
	}

	ret = slsi_mlme_nan_append_data_path_sec(req, &hal_req->sec_info);
	if (ret) {
		SLSI_WARN_NODEV("Error append datapath sec TLV\n");
		return ret;
	}

	ret = slsi_mlme_nan_append_ranging(req, &hal_req->ranging_cfg);
	if (ret) {
		SLSI_WARN_NODEV("Error append ranging config TLV\n");
		return ret;
	}

	ret = slsi_mlme_nan_append_address_set(req, hal_req->num_intf_addr_present, (u8 *)hal_req->intf_addr);
	if (ret)
		SLSI_WARN_NODEV("Error append address set TLV\n");
	return ret;
}

int slsi_mlme_nan_subscribe(struct slsi_dev *sdev, struct net_device *dev, struct slsi_hal_nan_subscribe_req *hal_req,
			    u16 subscribe_id)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_sdf_flags = 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");
	if (hal_req) {
		/* subscribespecific + discovery + data_path sec + ranging */
		u16 length = 11 + 18 + 70 + 11;

		length += hal_req->service_name_len ? hal_req->service_name_len + 4 : 0;
		length += hal_req->service_specific_info_len ? hal_req->service_specific_info_len + 4 : 0;
		length += hal_req->rx_match_filter_len ? hal_req->rx_match_filter_len + 4 : 0;
		length += hal_req->tx_match_filter_len ? hal_req->tx_match_filter_len + 4 : 0;
		length += hal_req->sdea_service_specific_info_len ? hal_req->sdea_service_specific_info_len + 4 : 0;
		length += hal_req->num_intf_addr_present ? hal_req->num_intf_addr_present * 6 + 4 : 0;

		req = fapi_alloc(mlme_nan_subscribe_req, MLME_NAN_SUBSCRIBE_REQ, ndev_vif->ifnum, length);
		if (!req) {
			SLSI_NET_ERR(dev, "fapi alloc failure\n");
			return -ENOMEM;
		}
		/* Set/Enable corresponding bits to disable
		 * indications that follow a subscribe.
		 * BIT0 - Disable subscribe termination indication.
		 * BIT1 - Disable match expired indication.
		 * BIT2 - Disable followUp indication received (OTA).
		 */
		if (hal_req->recv_indication_cfg & BIT(0))
			nan_sdf_flags |= FAPI_NANSDFCONTROL_SUBSCRIBE_END_EVENT;
		if (hal_req->recv_indication_cfg & BIT(1))
			nan_sdf_flags |= FAPI_NANSDFCONTROL_MATCH_EXPIRED_EVENT;
		if (hal_req->recv_indication_cfg & BIT(2))
			nan_sdf_flags |= FAPI_NANSDFCONTROL_RECEIVED_FOLLOWUP_EVENT;
	} else {
		req = fapi_alloc(mlme_nan_subscribe_req, MLME_NAN_SUBSCRIBE_REQ, ndev_vif->ifnum, 0);
		if (!req) {
			SLSI_NET_ERR(dev, "fapi alloc failure\n");
			return -ENOMEM;
		}
	}

	fapi_set_u16(req, u.mlme_nan_subscribe_req.subscribe_id, subscribe_id);
	fapi_set_u16(req, u.mlme_nan_subscribe_req.nan_sdf_flags, nan_sdf_flags);

	if (hal_req) {
		r = slsi_mlme_nan_subscribe_fapi_data(req, hal_req);
		if (r) {
			SLSI_NET_ERR(dev, "Failed to construct mbulkdata\n");
			slsi_kfree_skb(req);
			return -EINVAL;
		}
	}

	cfm = slsi_mlme_req_cfm(sdev, dev, req, MLME_NAN_SUBSCRIBE_CFM);
	if (!cfm)
		return -EIO;

	if (fapi_get_u16(cfm, u.mlme_nan_subscribe_cfm.result_code) != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "MLME_NAN_SUBSCRIBE_CFM(res:0x%04x)\n",
			     fapi_get_u16(cfm, u.mlme_host_state_cfm.result_code));
		r = -EINVAL;
	}

	if (hal_req && !r)
		ndev_vif->nan.subscribe_id_map |= (u32)BIT(subscribe_id);
	else
		ndev_vif->nan.subscribe_id_map &= (u32)~BIT(subscribe_id);
	slsi_kfree_skb(cfm);
	return r;
}

static u32 slsi_mlme_nan_followup_fapi_data(struct sk_buff *req, struct slsi_hal_nan_transmit_followup_req *hal_req)
{
	u32 ret;

	ret = slsi_mlme_nan_append_service_specific_info(req, hal_req->service_specific_info_len,
							 hal_req->service_specific_info);
	if (ret) {
		SLSI_WARN_NODEV("Error append service specific info TLV\n");
		return ret;
	}

	ret = slsi_mlme_nan_append_ext_service_specific_info(req, hal_req->sdea_service_specific_info_len,
							     hal_req->sdea_service_specific_info);
	if (ret)
		SLSI_WARN_NODEV("Error append extServSpecInfo TLV\n");
	return ret;
}

int slsi_mlme_nan_tx_followup(struct slsi_dev *sdev, struct net_device *dev,
			      struct slsi_hal_nan_transmit_followup_req *hal_req)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_sdf_flags = 0;
	u16               data_len;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");

	data_len = hal_req->service_specific_info_len ? (hal_req->service_specific_info_len + 4) : 0;
	data_len = hal_req->sdea_service_specific_info_len ? (hal_req->sdea_service_specific_info_len + 4) : 0;

	/* max possible length for publish attributes: 5*255 */
	req = fapi_alloc(mlme_nan_followup_req, MLME_NAN_FOLLOWUP_REQ, ndev_vif->ifnum, 5 * 255);
	if (!req) {
		SLSI_NET_ERR(dev, "fapi alloc failure\n");
		return -ENOMEM;
	}

	fapi_set_u16(req, u.mlme_nan_followup_req.publish_subscribe_id, hal_req->publish_subscribe_id);
	fapi_set_u16(req, u.mlme_nan_followup_req.match_id, hal_req->requestor_instance_id);
	fapi_set_u16(req, u.mlme_nan_followup_req.nan_sdf_flags, nan_sdf_flags);

	r = slsi_mlme_nan_followup_fapi_data(req, hal_req);
	if (r) {
		SLSI_NET_ERR(dev, "Failed to construct mbulkdata\n");
		slsi_kfree_skb(req);
		return -EINVAL;
	}

	cfm = slsi_mlme_req_cfm(sdev, dev, req, MLME_NAN_FOLLOWUP_CFM);
	if (!cfm)
		return -EIO;

	if (fapi_get_u16(cfm, u.mlme_nan_followup_cfm.result_code) != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "MLME_NAN_FOLLOWUP_CFM(res:0x%04x)\n",
			     fapi_get_u16(cfm, u.mlme_host_state_cfm.result_code));
		r = -EINVAL;
	}

	slsi_kfree_skb(cfm);
	return r;
}

static u32 slsi_mlme_nan_config_fapi_data(struct sk_buff *req, struct slsi_hal_nan_config_req *hal_req)
{
	u16 rssi_window = hal_req->config_rssi_window_size ? hal_req->rssi_window_size_val : 8;
	u32 ret;
	u8  rssi_close = 0, rssi_middle = 0, rssi_proximity = 0;
	u16 is_sid_in_beacon = hal_req->config_subscribe_sid_beacon && (hal_req->subscribe_sid_beacon_val & 0x01);
	u8  sid_count_in_beacon = hal_req->config_subscribe_sid_beacon ? hal_req->subscribe_sid_beacon_val >> 1 : 0;

	ret = slsi_mlme_nan_append_config_tlv(req, hal_req->master_pref,
					      hal_req->config_sid_beacon && (hal_req->sid_beacon & 0x01),
					      hal_req->config_sid_beacon ? hal_req->sid_beacon >> 1 : 0,
					      is_sid_in_beacon, sid_count_in_beacon, rssi_window,
					      hal_req->disc_mac_addr_rand_interval_sec, 1);
	if (ret) {
		SLSI_WARN_NODEV("Error append config TLV\n");
		return ret;
	}

	/* 2.4G NAN band specific config*/
	rssi_proximity = hal_req->config_rssi_proximity ? hal_req->rssi_proximity : 0;
	ret = slsi_mlme_nan_append_2g4_band_specific_config(req, rssi_close, rssi_middle, rssi_proximity,
							    hal_req->scan_params_val.dwell_time[0],
							    hal_req->scan_params_val.scan_period[0],
							    hal_req->config_2dot4g_dw_band,
							    hal_req->dw_2dot4g_interval_val);
	if (ret) {
		SLSI_WARN_NODEV("Error append 2.4G band specific TLV\n");
		return ret;
	}

	/* 5G NAN band specific config*/
	rssi_proximity = hal_req->config_5g_rssi_close_proximity ? hal_req->rssi_close_proximity_5g_val : 0;
	ret = slsi_mlme_nan_append_5g_band_specific_config(req, rssi_close, rssi_middle, rssi_proximity,
							   hal_req->scan_params_val.dwell_time[1],
							   hal_req->scan_params_val.scan_period[1],
							   hal_req->config_5g_dw_band,
							   hal_req->dw_5g_interval_val);
	if (ret)
		SLSI_WARN_NODEV("Error append 5G band specific TLV\n");
	return ret;
}

int slsi_mlme_nan_set_config(struct slsi_dev *sdev, struct net_device *dev, struct slsi_hal_nan_config_req *hal_req)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_oper_ctrl = 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");
	/* mbulk data length = 0x0f + 4 + 2 * (9 + 4) = 45 */
	req = fapi_alloc(mlme_nan_config_req, MLME_NAN_CONFIG_REQ, ndev_vif->ifnum, 45);
	if (!req) {
		SLSI_NET_ERR(dev, "fapi alloc failure\n");
		return -ENOMEM;
	}

	nan_oper_ctrl |= FAPI_NANOPERATIONCONTROL_MAC_ADDRESS_EVENT | FAPI_NANOPERATIONCONTROL_START_CLUSTER_EVENT |
			FAPI_NANOPERATIONCONTROL_JOINED_CLUSTER_EVENT;
	fapi_set_u16(req, u.mlme_nan_config_req.nan_operation_control_flags, nan_oper_ctrl);

	r = slsi_mlme_nan_config_fapi_data(req, hal_req);
	if (r) {
		SLSI_NET_ERR(dev, "Failed to construct mbulkdata\n");
		slsi_kfree_skb(req);
		return -EINVAL;
	}

	cfm = slsi_mlme_req_cfm(sdev, dev, req, MLME_NAN_CONFIG_CFM);
	if (!cfm)
		return -EIO;

	if (fapi_get_u16(cfm, u.mlme_nan_followup_cfm.result_code) != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "MLME_NAN_FOLLOWUP_CFM(res:0x%04x)\n",
			     fapi_get_u16(cfm, u.mlme_host_state_cfm.result_code));
		r = -EINVAL;
	}

	slsi_kfree_skb(cfm);
	return r;
}
