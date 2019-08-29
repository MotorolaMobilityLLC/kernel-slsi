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

static void slsi_mlme_nan_enable_fapi_data(struct sk_buff *req, struct slsi_hal_nan_enable_req *hal_req)
{
	u8  nan_config_fields_header[] = {0xdd, 0x24, 0x00, 0x16, 0x32, 0x0b, 0x01};
	u16 fapi_bool;
	u8  fapi_u8 = 0;
	u16 rssi_window = hal_req->config_rssi_window_size ? hal_req->rssi_window_size_val : 8;

	fapi_append_data(req, nan_config_fields_header, sizeof(nan_config_fields_header));

	fapi_append_data(req, &hal_req->master_pref, 1);

	/* publish service ID inclusion in beacon */
	fapi_bool = hal_req->config_sid_beacon && (hal_req->sid_beacon_val & 0x01);
	fapi_append_data(req, (u8 *)&fapi_bool, 2);
	if (fapi_bool)
		fapi_u8 = hal_req->sid_beacon_val >> 1;
	fapi_append_data(req, &fapi_u8, 1);

	/* subscribe service ID inclusion in beacon */
	fapi_bool = hal_req->config_subscribe_sid_beacon && (hal_req->subscribe_sid_beacon_val & 0x01);
	fapi_append_data(req, (u8 *)&fapi_bool, 2);
	if (fapi_bool)
		fapi_u8 = hal_req->subscribe_sid_beacon_val >> 1;
	fapi_append_data(req, &fapi_u8, 1);

	fapi_append_data(req, (u8 *)&rssi_window, 2);
	fapi_append_data(req, (u8 *)&hal_req->disc_mac_addr_rand_interval_sec, 4);

	/* 2.4G NAN band specific config*/
	fapi_u8 = hal_req->config_2dot4g_rssi_close ? hal_req->rssi_close_2dot4g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_u8 = hal_req->config_2dot4g_rssi_middle ? hal_req->rssi_middle_2dot4g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_u8 = hal_req->config_2dot4g_rssi_proximity ? hal_req->rssi_proximity_2dot4g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_append_data(req, &hal_req->scan_params_val.dwell_time[0], 1);
	fapi_append_data(req, (u8 *)&hal_req->scan_params_val.scan_period[0], 2);
	fapi_bool = hal_req->config_2dot4g_dw_band;
	fapi_append_data(req, (u8 *)&fapi_bool, 2);
	fapi_append_data(req, (u8 *)&hal_req->dw_2dot4g_interval_val, 1);

	/* 5G NAN band specific config*/
	fapi_u8 = hal_req->config_5g_rssi_close ? hal_req->rssi_close_5g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_u8 = hal_req->config_5g_rssi_middle ? hal_req->rssi_middle_5g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_u8 = hal_req->config_5g_rssi_close_proximity ? hal_req->rssi_close_proximity_5g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_append_data(req, &hal_req->scan_params_val.dwell_time[1], 1);
	fapi_append_data(req, (u8 *)&hal_req->scan_params_val.scan_period[1], 2);
	fapi_bool = hal_req->config_5g_dw_band;
	fapi_append_data(req, (u8 *)&fapi_bool, 2);
	fapi_append_data(req, (u8 *)&hal_req->dw_5g_interval_val, 1);
}

int slsi_mlme_nan_enable(struct slsi_dev *sdev, struct net_device *dev, struct slsi_hal_nan_enable_req *hal_req)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_oper_ctrl = 0;
	u16               operatein5gband = hal_req->config_support_5g && hal_req->support_5g_val;
	u16               hopcountmax = hal_req->config_hop_count_limit ? hal_req->hop_count_limit_val : 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");

	/* max mbulk data IE info length is 0x24. So need 0x26 bytes */
	req = fapi_alloc(mlme_nan_start_req, MLME_NAN_START_REQ, ndev_vif->ifnum, 0x26);
	if (!req) {
		SLSI_NET_ERR(dev, "fapi alloc failure\n");
		return -ENOMEM;
	}

	nan_oper_ctrl |= FAPI_NANOPERATIONCONTROL_MAC_ADDRESS_EVENT | FAPI_NANOPERATIONCONTROL_START_CLUSTER_EVENT |
			FAPI_NANOPERATIONCONTROL_JOINED_CLUSTER_EVENT;

	fapi_set_u16(req, u.mlme_nan_start_req.operatein5gband, operatein5gband);
	fapi_set_u16(req, u.mlme_nan_start_req.hopcountmax, hopcountmax);
	fapi_set_u16(req, u.mlme_nan_start_req.nan_operation_control_flags, nan_oper_ctrl);

	slsi_mlme_nan_enable_fapi_data(req, hal_req);

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

static void slsi_mlme_nan_append_tlv(struct sk_buff *req, __le16 tlv_t, __le16 tlv_l, u8 *tlv_v, u8 **header_ptr,
				     u8 header_ie_generic_len, u8 **end_ptr)
{
	u8 tmp_buf[255 + 4];
	u8 *tmp_buf_pos;
	int tmp_buf_len, len1, ip_ie_len;

	memcpy(tmp_buf, &tlv_t, 2);
	memcpy(tmp_buf + 2, &tlv_l, 2);
	memcpy(tmp_buf + 4, tlv_v, tlv_l);
	tmp_buf_len = 4 + tlv_l;
	ip_ie_len = *end_ptr - *header_ptr - 2;
	tmp_buf_pos = tmp_buf;

	while (tmp_buf_len + ip_ie_len > 255) {
		len1 = 255 - ip_ie_len;
		fapi_append_data(req, tmp_buf_pos, len1);
		(*header_ptr)[1] = 255;
		tmp_buf_len -= len1;
		tmp_buf_pos += len1;
		ip_ie_len = 0;
		if (tmp_buf_len) {
			*header_ptr = fapi_append_data(req, *header_ptr, header_ie_generic_len);
			*end_ptr = *header_ptr + header_ie_generic_len;
		} else {
			*end_ptr = *header_ptr + header_ie_generic_len + 255;
		}
	}
	if (tmp_buf_len) {
		fapi_append_data(req, tmp_buf, tmp_buf_len);
		*end_ptr += tmp_buf_len;
	}
}

static void slsi_mlme_nan_fapi_put_data_path_security_ie(struct sk_buff *req, struct slsi_nan_security_info *sec_info)
{
	u8 ie_header[] = {0xdd, 0x00, 0x00, 0x16, 0x32, 0x0b, 0x07};
	u8 *header_ptr;
	u8  u8val, i, key_len = 0;

	header_ptr = fapi_append_data(req, ie_header, sizeof(ie_header));
	u8val = sec_info->key_info.key_type == 1 || sec_info->key_info.key_type == 2 ? sec_info->key_info.key_type : 0;
	fapi_append_data(req, &u8val, 1);
	fapi_append_data(req, (u8 *)&sec_info->cipher_type, 1);
	if (sec_info->key_info.key_type == 1) {
		fapi_append_data(req, sec_info->key_info.body.pmk_info.pmk, 32);
		u8val = 0;
		fapi_append_data(req, &u8val, 1);
	} else {
		u8val = 0;
		for (i = 0; i < 32; i++)
			fapi_append_data(req, &u8val, 1);
		if (sec_info->key_info.key_type == 2) {
			key_len = sec_info->key_info.body.passphrase_info.passphrase_len;
			fapi_append_data(req, &key_len, 1);
			fapi_append_data(req, sec_info->key_info.body.passphrase_info.passphrase, key_len);
		} else {
			fapi_append_data(req, &u8val, 1);
		}
	}
	header_ptr[1] = 40 + key_len;
}

static void slsi_mlme_nan_fapi_put_nan_ranging_ie(struct sk_buff *req, struct slsi_nan_ranging_cfg *cfg)
{
	u8 ie_header[] = {0xdd, 0x0b, 0x00, 0x16, 0x32, 0x0b, 0x09};

	fapi_append_data(req, ie_header, sizeof(ie_header));
	fapi_append_data(req, (u8 *)&cfg->ranging_interval_msec, 2);
	fapi_append_data(req, (u8 *)&cfg->config_ranging_indications, 1);
	fapi_append_data(req, (u8 *)&cfg->distance_ingress_mm, 2);
	fapi_append_data(req, (u8 *)&cfg->distance_egress_mm, 2);
}

static void slsi_mlme_nan_publish_fapi_data(struct sk_buff *req, struct slsi_hal_nan_publish_req *hal_req)
{
	u8  nan_publish_fields_header[] = {0xdd, 0x00, 0x00, 0x16, 0x32, 0x0b, 0x02};
	u8 *header_ptr, *end_ptr;
	__le16 le16val;
	u8  u8val;

	header_ptr = fapi_append_data(req, nan_publish_fields_header, sizeof(nan_publish_fields_header));
	fapi_append_data(req, &hal_req->publish_type, 1);
	fapi_append_data(req, &hal_req->tx_type, 1);

	le16val = cpu_to_le16(hal_req->ttl);
	fapi_append_data(req, (u8 *)&le16val, 2);
	le16val = cpu_to_le16(hal_req->period);
	fapi_append_data(req, (u8 *)&le16val, 2);
	fapi_append_data(req, &hal_req->publish_count, 1);
	fapi_append_data(req, &hal_req->publish_match_indicator, 1);
	le16val = cpu_to_le16(hal_req->rssi_threshold_flag);
	fapi_append_data(req, (u8 *)&le16val, 2);
	u8val = 0;
	fapi_append_data(req, (u8 *)&u8val, 1); /* Ranging required */
	fapi_append_data(req, (u8 *)&u8val, 1); /* Data path required */

	end_ptr = header_ptr + sizeof(nan_publish_fields_header) + 12;

	if (hal_req->service_name_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SERVICE_NAME),
					 cpu_to_le16 (hal_req->service_name_len), hal_req->service_name, &header_ptr,
					 sizeof(nan_publish_fields_header), &end_ptr);

	if (hal_req->service_specific_info_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SERVICE_SPECIFIC_INFO),
					 cpu_to_le16 (hal_req->service_specific_info_len),
					 hal_req->service_specific_info, &header_ptr,
					 sizeof(nan_publish_fields_header), &end_ptr);

	if (hal_req->sdea_service_specific_info_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SDEA),
					 cpu_to_le16 (hal_req->sdea_service_specific_info_len),
					 hal_req->sdea_service_specific_info,
					 &header_ptr, sizeof(nan_publish_fields_header), &end_ptr);

	if (hal_req->rx_match_filter_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_RX_MATCH_FILTER),
					 cpu_to_le16 (hal_req->rx_match_filter_len), hal_req->rx_match_filter,
					 &header_ptr, sizeof(nan_publish_fields_header), &end_ptr);

	if (hal_req->tx_match_filter_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_TX_MATCH_FILTER),
					 cpu_to_le16 (hal_req->tx_match_filter_len), hal_req->tx_match_filter,
					 &header_ptr, sizeof(nan_publish_fields_header), &end_ptr);

	/* update len */
	header_ptr[1] = end_ptr - header_ptr - 2;
	slsi_mlme_nan_fapi_put_data_path_security_ie(req, &hal_req->sec_info);
	slsi_mlme_nan_fapi_put_nan_ranging_ie(req, &hal_req->ranging_cfg);

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
		u16 max_mbulk_data_len;
		u16 length = 17; /* non tlv info in fapi publish IE */

		length += hal_req->service_name_len ? hal_req->service_name_len + 4 : 0;
		length += hal_req->service_specific_info_len ? hal_req->service_specific_info_len + 4 : 0;
		length += hal_req->rx_match_filter_len ? hal_req->rx_match_filter_len + 4 : 0;
		length += hal_req->tx_match_filter_len ? hal_req->tx_match_filter_len + 4 : 0;
		length += hal_req->sdea_service_specific_info_len ? hal_req->sdea_service_specific_info_len + 4 : 0;
		if (length > 255)
			/* 2 = ie_id _ie_len, 5 = oui+type+sub_type*/
			max_mbulk_data_len = (255 + 2) * (length / (255 - (2 + 5)) + 1);
		else
			max_mbulk_data_len = length + 2;
		max_mbulk_data_len += 42 + 64; /* max length for NAN Data Path Security IE */
		max_mbulk_data_len += 14; /* NAN Ranging IE*/

		req = fapi_alloc(mlme_nan_publish_req, MLME_NAN_PUBLISH_REQ, ndev_vif->ifnum, max_mbulk_data_len);
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

	if (hal_req)
		slsi_mlme_nan_publish_fapi_data(req, hal_req);

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

static void slsi_mlme_nan_subscribe_fapi_data(struct sk_buff *req, struct slsi_hal_nan_subscribe_req *hal_req)
{
	u8  nan_subscribe_fields_header[] = {0xdd, 0x00, 0x00, 0x16, 0x32, 0x0b, 0x03};
	u8 *header_ptr, *end_ptr;
	__le16 le16val;
	u8  u8val = 0;

	header_ptr = fapi_append_data(req, nan_subscribe_fields_header, sizeof(nan_subscribe_fields_header));
	fapi_append_data(req, &hal_req->subscribe_type, 1);
	fapi_append_data(req, &hal_req->service_response_filter, 1);
	fapi_append_data(req, &hal_req->service_response_include, 1);
	fapi_append_data(req, &hal_req->use_service_response_filter, 1);
	fapi_append_data(req, &hal_req->ssi_required_for_match_indication, 1);

	le16val = cpu_to_le16(hal_req->ttl);
	fapi_append_data(req, (u8 *)&le16val, 2);
	le16val = cpu_to_le16(hal_req->period);
	fapi_append_data(req, (u8 *)&le16val, 2);
	fapi_append_data(req, &hal_req->subscribe_count, 1);
	fapi_append_data(req, &hal_req->subscribe_match_indicator, 1);
	le16val = cpu_to_le16(hal_req->rssi_threshold_flag);
	fapi_append_data(req, (u8 *)&le16val, 2);
	fapi_append_data(req, &u8val, 1); /* ranging required */
	end_ptr = fapi_append_data(req, &u8val, 1); /* datapath required */
	end_ptr += 1;

	if (hal_req->service_name_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SERVICE_NAME),
					 cpu_to_le16 (hal_req->service_name_len), hal_req->service_name, &header_ptr,
					 sizeof(nan_subscribe_fields_header), &end_ptr);

	if (hal_req->service_specific_info_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SERVICE_SPECIFIC_INFO),
					 cpu_to_le16 (hal_req->service_specific_info_len),
					 hal_req->service_specific_info, &header_ptr,
					 sizeof(nan_subscribe_fields_header), &end_ptr);

	if (hal_req->rx_match_filter_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_RX_MATCH_FILTER),
					 cpu_to_le16 (hal_req->rx_match_filter_len), hal_req->rx_match_filter,
					 &header_ptr, sizeof(nan_subscribe_fields_header), &end_ptr);

	if (hal_req->tx_match_filter_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_TX_MATCH_FILTER),
					 cpu_to_le16 (hal_req->tx_match_filter_len), hal_req->tx_match_filter,
					 &header_ptr, sizeof(nan_subscribe_fields_header), &end_ptr);

	if (hal_req->sdea_service_specific_info_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SDEA),
					 cpu_to_le16 (hal_req->sdea_service_specific_info_len),
					 hal_req->sdea_service_specific_info,
					 &header_ptr, sizeof(nan_subscribe_fields_header), &end_ptr);

	/* update len */
	header_ptr[1] = end_ptr - header_ptr - 2;
	slsi_mlme_nan_fapi_put_data_path_security_ie(req, &hal_req->sec_info);
	slsi_mlme_nan_fapi_put_nan_ranging_ie(req, &hal_req->ranging_cfg);
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
		/*max possible length for publish attributes: 8*255 */
		u16 max_mbulk_data_len;
		u16 length = 17; /* non tlv info in fapi publish IE */

		length += hal_req->service_name_len ? hal_req->service_name_len + 4 : 0;
		length += hal_req->service_specific_info_len ? hal_req->service_specific_info_len + 4 : 0;
		length += hal_req->rx_match_filter_len ? hal_req->rx_match_filter_len + 4 : 0;
		length += hal_req->tx_match_filter_len ? hal_req->tx_match_filter_len + 4 : 0;
		length += hal_req->sdea_service_specific_info_len ? hal_req->sdea_service_specific_info_len + 4 : 0;
		if (length > 255)
			/* 2 = ie_id _ie_len, 5 = oui+type+sub_type*/
			max_mbulk_data_len = (255 + 2) * (length / (255 - (2 + 5)) + 1);
		else
			max_mbulk_data_len = length + 2;

		req = fapi_alloc(mlme_nan_subscribe_req, MLME_NAN_SUBSCRIBE_REQ, ndev_vif->ifnum, max_mbulk_data_len);
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

	if (hal_req)
		slsi_mlme_nan_subscribe_fapi_data(req, hal_req);

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

static void slsi_mlme_nan_followup_fapi_data(struct sk_buff *req, struct slsi_hal_nan_transmit_followup_req *hal_req)
{
	u8  nan_followup_fields_header[] = {0xdd, 0x00, 0x00, 0x16, 0x32, 0x0b, 0x05};
	u8 *header_ptr, *end_ptr;

	header_ptr = fapi_append_data(req, nan_followup_fields_header, sizeof(nan_followup_fields_header));
	fapi_append_data(req, hal_req->addr, ETH_ALEN);
	fapi_append_data(req, &hal_req->priority, 1);
	end_ptr = fapi_append_data(req, &hal_req->dw_or_faw, 1);
	end_ptr += 1;

	if (hal_req->service_specific_info_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SERVICE_SPECIFIC_INFO),
					 cpu_to_le16 (hal_req->service_specific_info_len),
					 hal_req->service_specific_info, &header_ptr,
					 sizeof(nan_followup_fields_header), &end_ptr);
	if (hal_req->sdea_service_specific_info_len)
		slsi_mlme_nan_append_tlv(req, cpu_to_le16 (SLSI_FAPI_NAN_SDEA),
					 cpu_to_le16 (hal_req->sdea_service_specific_info_len),
					 hal_req->sdea_service_specific_info, &header_ptr,
					 sizeof(nan_followup_fields_header), &end_ptr);

	/* update len */
	header_ptr[1] = end_ptr - header_ptr - 2;
}

int slsi_mlme_nan_tx_followup(struct slsi_dev *sdev, struct net_device *dev,
			      struct slsi_hal_nan_transmit_followup_req *hal_req)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_sdf_flags = 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");

	/* max possible length for publish attributes: 5*255 */
	req = fapi_alloc(mlme_nan_followup_req, MLME_NAN_FOLLOWUP_REQ, ndev_vif->ifnum, 5 * 255);
	if (!req) {
		SLSI_NET_ERR(dev, "fapi alloc failure\n");
		return -ENOMEM;
	}

	fapi_set_u16(req, u.mlme_nan_followup_req.publish_subscribe_id, hal_req->publish_subscribe_id);
	fapi_set_u16(req, u.mlme_nan_followup_req.peer_id, hal_req->requestor_instance_id);
	fapi_set_u16(req, u.mlme_nan_followup_req.nan_sdf_flags, nan_sdf_flags);

	slsi_mlme_nan_followup_fapi_data(req, hal_req);

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

static void slsi_mlme_nan_config_fapi_data(struct sk_buff *req, struct slsi_hal_nan_config_req *hal_req)
{
	u8  nan_config_fields_header[] = {0xdd, 0x24, 0x00, 0x16, 0x32, 0x0b, 0x01};
	u16 fapi_bool;
	u8  fapi_u8 = 0;
	u16 rssi_window = hal_req->config_rssi_window_size ? hal_req->rssi_window_size_val : 8;

	fapi_append_data(req, nan_config_fields_header, sizeof(nan_config_fields_header));

	fapi_append_data(req, &hal_req->master_pref, 1);

	/* publish service ID inclusion in beacon */
	fapi_bool = hal_req->config_sid_beacon && (hal_req->sid_beacon & 0x01);
	fapi_append_data(req, (u8 *)&fapi_bool, 2);

	fapi_u8 = fapi_bool ? hal_req->sid_beacon >> 1 : 0;
	fapi_append_data(req, &fapi_u8, 1);

	/* subscribe service ID inclusion in beacon */
	fapi_bool = hal_req->config_subscribe_sid_beacon && (hal_req->subscribe_sid_beacon_val & 0x01);
	fapi_append_data(req, (u8 *)&fapi_bool, 2);

	fapi_u8 = fapi_bool ? hal_req->subscribe_sid_beacon_val >> 1 : 0;
	fapi_append_data(req, &fapi_u8, 1);

	fapi_append_data(req, (u8 *)&rssi_window, 2);
	fapi_append_data(req, (u8 *)&hal_req->disc_mac_addr_rand_interval_sec, 4);

	/* 2.4G NAN band specific config*/
	fapi_u8 = 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_append_data(req, &fapi_u8, 1);
	fapi_u8 = hal_req->config_rssi_proximity ? hal_req->rssi_proximity : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_append_data(req, &hal_req->scan_params_val.dwell_time[0], 1);
	fapi_append_data(req, (u8 *)&hal_req->scan_params_val.scan_period[0], 2);
	fapi_bool = hal_req->config_2dot4g_dw_band;
	fapi_append_data(req, (u8 *)&fapi_bool, 2);
	fapi_append_data(req, (u8 *)&hal_req->dw_2dot4g_interval_val, 1);

	/* 5G NAN band specific config*/
	fapi_u8 = 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_append_data(req, &fapi_u8, 1);
	fapi_u8 = hal_req->config_5g_rssi_close_proximity ? hal_req->rssi_close_proximity_5g_val : 0;
	fapi_append_data(req, &fapi_u8, 1);
	fapi_append_data(req, &hal_req->scan_params_val.dwell_time[1], 1);
	fapi_append_data(req, (u8 *)&hal_req->scan_params_val.scan_period[1], 2);
	fapi_bool = hal_req->config_5g_dw_band;
	fapi_append_data(req, (u8 *)&fapi_bool, 2);
	fapi_append_data(req, (u8 *)&hal_req->dw_5g_interval_val, 1);
}

int slsi_mlme_nan_set_config(struct slsi_dev *sdev, struct net_device *dev, struct slsi_hal_nan_config_req *hal_req)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               r = 0;
	u16               nan_oper_ctrl = 0;

	SLSI_NET_DBG3(dev, SLSI_MLME, "\n");
	/* max possible length for publish attributes 5*255 */
	req = fapi_alloc(mlme_nan_config_req, MLME_NAN_CONFIG_REQ, ndev_vif->ifnum, 5 * 255);
	if (!req) {
		SLSI_NET_ERR(dev, "fapi alloc failure\n");
		return -ENOMEM;
	}

	nan_oper_ctrl |= FAPI_NANOPERATIONCONTROL_MAC_ADDRESS_EVENT | FAPI_NANOPERATIONCONTROL_START_CLUSTER_EVENT |
			FAPI_NANOPERATIONCONTROL_JOINED_CLUSTER_EVENT;
	fapi_set_u16(req, u.mlme_nan_config_req.nan_operation_control_flags, nan_oper_ctrl);

	slsi_mlme_nan_config_fapi_data(req, hal_req);

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
