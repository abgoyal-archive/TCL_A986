

#include "includes.h"

#include "common.h"
#include "eap_defs.h"
#include "eap_common.h"

const u8 * eap_hdr_validate(int vendor, EapType eap_type,
			    const struct wpabuf *msg, size_t *plen)
{
	const struct eap_hdr *hdr;
	const u8 *pos;
	size_t len;

	hdr = wpabuf_head(msg);

	if (wpabuf_len(msg) < sizeof(*hdr)) {
		wpa_printf(MSG_INFO, "EAP: Too short EAP frame");
		return NULL;
	}

	len = be_to_host16(hdr->length);
	if (len < sizeof(*hdr) + 1 || len > wpabuf_len(msg)) {
		wpa_printf(MSG_INFO, "EAP: Invalid EAP length");
		return NULL;
	}

	pos = (const u8 *) (hdr + 1);

	if (*pos == EAP_TYPE_EXPANDED) {
		int exp_vendor;
		u32 exp_type;
		if (len < sizeof(*hdr) + 8) {
			wpa_printf(MSG_INFO, "EAP: Invalid expanded EAP "
				   "length");
			return NULL;
		}
		pos++;
		exp_vendor = WPA_GET_BE24(pos);
		pos += 3;
		exp_type = WPA_GET_BE32(pos);
		pos += 4;
		if (exp_vendor != vendor || exp_type != (u32) eap_type) {
			wpa_printf(MSG_INFO, "EAP: Invalid expanded frame "
				   "type");
			return NULL;
		}

		*plen = len - sizeof(*hdr) - 8;
		return pos;
	} else {
		if (vendor != EAP_VENDOR_IETF || *pos != eap_type) {
			wpa_printf(MSG_INFO, "EAP: Invalid frame type");
			return NULL;
		}
		*plen = len - sizeof(*hdr) - 1;
		return pos + 1;
	}
}


struct wpabuf * eap_msg_alloc(int vendor, EapType type, size_t payload_len,
			      u8 code, u8 identifier)
{
	struct wpabuf *buf;
	struct eap_hdr *hdr;
	size_t len;

	len = sizeof(struct eap_hdr) + (vendor == EAP_VENDOR_IETF ? 1 : 8) +
		payload_len;
	buf = wpabuf_alloc(len);
	if (buf == NULL)
		return NULL;

	hdr = wpabuf_put(buf, sizeof(*hdr));
	hdr->code = code;
	hdr->identifier = identifier;
	hdr->length = host_to_be16(len);

	if (vendor == EAP_VENDOR_IETF) {
		wpabuf_put_u8(buf, type);
	} else {
		wpabuf_put_u8(buf, EAP_TYPE_EXPANDED);
		wpabuf_put_be24(buf, vendor);
		wpabuf_put_be32(buf, type);
	}

	return buf;
}


void eap_update_len(struct wpabuf *msg)
{
	struct eap_hdr *hdr;
	hdr = wpabuf_mhead(msg);
	if (wpabuf_len(msg) < sizeof(*hdr))
		return;
	hdr->length = host_to_be16(wpabuf_len(msg));
}


u8 eap_get_id(const struct wpabuf *msg)
{
	const struct eap_hdr *eap;

	if (wpabuf_len(msg) < sizeof(*eap))
		return 0;

	eap = wpabuf_head(msg);
	return eap->identifier;
}


EapType eap_get_type(const struct wpabuf *msg)
{
	if (wpabuf_len(msg) < sizeof(struct eap_hdr) + 1)
		return EAP_TYPE_NONE;

	return ((const u8 *) wpabuf_head(msg))[sizeof(struct eap_hdr)];
}
