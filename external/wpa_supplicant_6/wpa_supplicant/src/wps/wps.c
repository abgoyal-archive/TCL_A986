

#include "includes.h"

#include "common.h"
#include "common/ieee802_11_defs.h"
#include "wps_i.h"
#include "wps_dev_attr.h"
#include "ieee802_11_defs.h"

#ifdef CONFIG_WPS_TESTING
int wps_version_number = 0x20;
int wps_testing_dummy_cred = 0;
#endif /* CONFIG_WPS_TESTING */


struct wps_data * wps_init(const struct wps_config *cfg)
{
	struct wps_data *data = os_zalloc(sizeof(*data));
	if (data == NULL)
		return NULL;
	data->wps = cfg->wps;
	data->registrar = cfg->registrar;
	if (cfg->registrar) {
		os_memcpy(data->uuid_r, cfg->wps->uuid, WPS_UUID_LEN);
	} else {
		os_memcpy(data->mac_addr_e, cfg->wps->dev.mac_addr, ETH_ALEN);
		os_memcpy(data->uuid_e, cfg->wps->uuid, WPS_UUID_LEN);
	}
	if (cfg->pin) {
		data->dev_pw_id = DEV_PW_DEFAULT;
		data->dev_password = os_malloc(cfg->pin_len);
		if (data->dev_password == NULL) {
			os_free(data);
			return NULL;
		}
		os_memcpy(data->dev_password, cfg->pin, cfg->pin_len);
		data->dev_password_len = cfg->pin_len;
	}

	data->pbc = cfg->pbc;
	if (cfg->pbc) {
		/* Use special PIN '00000000' for PBC */
		data->dev_pw_id = DEV_PW_PUSHBUTTON;
		os_free(data->dev_password);
		data->dev_password = os_malloc(8);
		if (data->dev_password == NULL) {
			os_free(data);
			return NULL;
		}
		os_memset(data->dev_password, '0', 8);
		data->dev_password_len = 8;
	}

	data->state = data->registrar ? RECV_M1 : SEND_M1;

	if (cfg->assoc_wps_ie) {
		struct wps_parse_attr attr;
		wpa_hexdump_buf(MSG_DEBUG, "WPS: WPS IE from (Re)AssocReq",
				cfg->assoc_wps_ie);
		if (wps_parse_msg(cfg->assoc_wps_ie, &attr) < 0) {
			wpa_printf(MSG_DEBUG, "WPS: Failed to parse WPS IE "
				   "from (Re)AssocReq");
		} else if (attr.request_type == NULL) {
			wpa_printf(MSG_DEBUG, "WPS: No Request Type attribute "
				   "in (Re)AssocReq WPS IE");
		} else {
			wpa_printf(MSG_DEBUG, "WPS: Request Type (from WPS IE "
				   "in (Re)AssocReq WPS IE): %d",
				   *attr.request_type);
			data->request_type = *attr.request_type;
		}
	}

	return data;
}


void wps_deinit(struct wps_data *data)
{
	if (data->wps_pin_revealed) {
		wpa_printf(MSG_DEBUG, "WPS: Full PIN information revealed and "
			   "negotiation failed");
		if (data->registrar)
			wps_registrar_invalidate_pin(data->wps->registrar,
						     data->uuid_e);
	} else if (data->registrar)
		wps_registrar_unlock_pin(data->wps->registrar, data->uuid_e);

	wpabuf_free(data->dh_privkey);
	wpabuf_free(data->dh_pubkey_e);
	wpabuf_free(data->dh_pubkey_r);
	wpabuf_free(data->last_msg);
	os_free(data->dev_password);
	os_free(data->new_psk);
	wps_device_data_free(&data->peer_dev);
	os_free(data);
}


enum wps_process_res wps_process_msg(struct wps_data *wps,
				     enum wsc_op_code op_code,
				     const struct wpabuf *msg)
{
	if (wps->registrar)
		return wps_registrar_process_msg(wps, op_code, msg);
	else
		return wps_enrollee_process_msg(wps, op_code, msg);
}


struct wpabuf * wps_get_msg(struct wps_data *wps, enum wsc_op_code *op_code)
{
	if (wps->registrar)
		return wps_registrar_get_msg(wps, op_code);
	else
		return wps_enrollee_get_msg(wps, op_code);
}


int wps_is_selected_pbc_registrar(const struct wpabuf *msg)
{
	struct wps_parse_attr attr;

	/*
	 * In theory, this could also verify that attr.sel_reg_config_methods
	 * includes WPS_CONFIG_PUSHBUTTON, but some deployed AP implementations
	 * do not set Selected Registrar Config Methods attribute properly, so
	 * it is safer to just use Device Password ID here.
	 */

	if (wps_parse_msg(msg, &attr) < 0 ||
	    !attr.selected_registrar || *attr.selected_registrar == 0 ||
	    !attr.dev_password_id ||
	    WPA_GET_BE16(attr.dev_password_id) != DEV_PW_PUSHBUTTON)
		return 0;

	return 1;
}


int wps_is_selected_pin_registrar(const struct wpabuf *msg)
{
	struct wps_parse_attr attr;

	/*
	 * In theory, this could also verify that attr.sel_reg_config_methods
	 * includes WPS_CONFIG_LABEL, WPS_CONFIG_DISPLAY, or WPS_CONFIG_KEYPAD,
	 * but some deployed AP implementations do not set Selected Registrar
	 * Config Methods attribute properly, so it is safer to just use
	 * Device Password ID here.
	 */

	if (wps_parse_msg(msg, &attr) < 0)
		return 0;

	if (!attr.selected_registrar || *attr.selected_registrar == 0)
		return 0;

	if (attr.dev_password_id != NULL &&
	    WPA_GET_BE16(attr.dev_password_id) == DEV_PW_PUSHBUTTON)
		return 0;

	return 1;
}


const u8 * wps_get_uuid_e(const struct wpabuf *msg)
{
	struct wps_parse_attr attr;

	if (wps_parse_msg(msg, &attr) < 0)
		return NULL;
	return attr.uuid_e;
}


struct wpabuf * wps_build_assoc_req_ie(enum wps_request_type req_type)
{
	struct wpabuf *ie;
	u8 *len;

	wpa_printf(MSG_DEBUG, "WPS: Building WPS IE for (Re)Association "
		   "Request");
	ie = wpabuf_alloc(100);
	if (ie == NULL)
		return NULL;

	wpabuf_put_u8(ie, WLAN_EID_VENDOR_SPECIFIC);
	len = wpabuf_put(ie, 1);
	wpabuf_put_be32(ie, WPS_DEV_OUI_WFA);

	if (wps_build_version(ie) ||
	    wps_build_req_type(ie, req_type) ||
	    wps_build_wfa_ext(ie, 0, NULL, 0)) {
		wpabuf_free(ie);
		return NULL;
	}

	*len = wpabuf_len(ie) - 2;

	return ie;
}


struct wpabuf * wps_build_assoc_resp_ie(void)
{
	struct wpabuf *ie;
	u8 *len;

	wpa_printf(MSG_DEBUG, "WPS: Building WPS IE for (Re)Association "
		   "Response");
	ie = wpabuf_alloc(100);
	if (ie == NULL)
		return NULL;

	wpabuf_put_u8(ie, WLAN_EID_VENDOR_SPECIFIC);
	len = wpabuf_put(ie, 1);
	wpabuf_put_be32(ie, WPS_DEV_OUI_WFA);

	if (wps_build_version(ie) ||
	    wps_build_resp_type(ie, WPS_RESP_AP) ||
	    wps_build_wfa_ext(ie, 0, NULL, 0)) {
		wpabuf_free(ie);
		return NULL;
	}

	*len = wpabuf_len(ie) - 2;

	return ie;
}


struct wpabuf * wps_build_probe_req_ie(int pbc, struct wps_device_data *dev,
				       const u8 *uuid,
				       enum wps_request_type req_type)
{
	struct wpabuf *ie;
	u8 *len;
	u16 methods = 0;

	wpa_printf(MSG_DEBUG, "WPS: Building WPS IE for Probe Request");

	ie = wpabuf_alloc(500);
	if (ie == NULL)
		return NULL;

	methods |= WPS_CONFIG_PUSHBUTTON;
#ifdef CONFIG_WPS2
	/*
	 * TODO: Should figure out whether this device has a physical or
	 * virtual pushbutton.
	 */
	methods |= WPS_CONFIG_VIRT_PUSHBUTTON;
#endif /* CONFIG_WPS2 */

	/*
	 * TODO: Should figure out whether this Probe Request was triggered
	 * using physical or virtual display. Also, if the device has a PIN on
	 * a label, that should be indicated here.
	 */
	methods |= WPS_CONFIG_DISPLAY |
#ifdef CONFIG_WPS2
		WPS_CONFIG_VIRT_DISPLAY |
#endif /* CONFIG_WPS2 */
			WPS_CONFIG_KEYPAD;
#ifdef CONFIG_WPS_UFD
	methods |= WPS_CONFIG_USBA;
#endif /* CONFIG_WPS_UFD */
#ifdef CONFIG_WPS_NFC
	methods |= WPS_CONFIG_NFC_INTERFACE;
#endif /* CONFIG_WPS_NFC */

	if (wps_build_version(ie) ||
	    wps_build_req_type(ie, req_type) ||
	    wps_build_config_methods(ie, methods) ||
	    wps_build_uuid_e(ie, uuid) ||
	    wps_build_primary_dev_type(dev, ie) ||
	    wps_build_rf_bands(dev, ie) ||
	    wps_build_assoc_state(NULL, ie) ||
	    wps_build_config_error(ie, WPS_CFG_NO_ERROR) ||
	    wps_build_dev_password_id(ie, pbc ? DEV_PW_PUSHBUTTON :
				      DEV_PW_DEFAULT) || 
#ifdef CONFIG_WPS2
	    wps_build_manufacturer(dev, ie) ||
	    wps_build_model_name(dev, ie) ||
	    wps_build_model_number(dev, ie) ||
	    wps_build_dev_name(dev, ie) ||
	    wps_build_wfa_ext(ie, req_type == WPS_REQ_ENROLLEE, NULL, 0)
#else /* CONFIG_WPS2 */
	    0
#endif /* CONFIG_WPS2 */
		) {
		wpabuf_free(ie);
		return NULL;
	}

	return wps_ie_encapsulate(ie);
}


void wps_free_pending_msgs(struct upnp_pending_message *msgs)
{
	struct upnp_pending_message *p, *prev;
	p = msgs;
	while (p) {
		prev = p;
		p = p->next;
		wpabuf_free(prev->msg);
		os_free(prev);
	}
}
