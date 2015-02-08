
#include "includes.h"

#include "common.h"
#include "eap_i.h"
#include "eap_tlv.h"


u8 * eap_tlv_build_nak(int id, u16 nak_type, size_t *resp_len)
{
	struct eap_hdr *hdr;
	u8 *pos;

	hdr = eap_msg_alloc(EAP_VENDOR_IETF, EAP_TYPE_TLV, resp_len,
			    10, EAP_CODE_RESPONSE, id, &pos);
	if (hdr == NULL)
		return NULL;

	*pos++ = 0x80; /* Mandatory */
	*pos++ = EAP_TLV_NAK_TLV;
	/* Length */
	*pos++ = 0;
	*pos++ = 6;
	/* Vendor-Id */
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = 0;
	/* NAK-Type */
	WPA_PUT_BE16(pos, nak_type);

	return (u8 *) hdr;
}


u8 * eap_tlv_build_result(int id, u16 status, size_t *resp_len)
{
	struct eap_hdr *hdr;
	u8 *pos;

	hdr = eap_msg_alloc(EAP_VENDOR_IETF, EAP_TYPE_TLV, resp_len,
			    6, EAP_CODE_RESPONSE, id, &pos);
	if (hdr == NULL)
		return NULL;

	*pos++ = 0x80; /* Mandatory */
	*pos++ = EAP_TLV_RESULT_TLV;
	/* Length */
	*pos++ = 0;
	*pos++ = 2;
	/* Status */
	WPA_PUT_BE16(pos, status);

	return (u8 *) hdr;
}


int eap_tlv_process(struct eap_sm *sm, struct eap_method_ret *ret,
		    const struct eap_hdr *hdr, u8 **resp, size_t *resp_len,
		    int force_failure)
{
	size_t left, tlv_len;
	const u8 *pos;
	const u8 *result_tlv = NULL;
	size_t result_tlv_len = 0;
	int tlv_type, mandatory;

	/* Parse TLVs */
	left = be_to_host16(hdr->length) - sizeof(struct eap_hdr) - 1;
	pos = (const u8 *) (hdr + 1);
	pos++;
	wpa_hexdump(MSG_DEBUG, "EAP-TLV: Received TLVs", pos, left);
	while (left >= 4) {
		mandatory = !!(pos[0] & 0x80);
		tlv_type = WPA_GET_BE16(pos) & 0x3fff;
		pos += 2;
		tlv_len = WPA_GET_BE16(pos);
		pos += 2;
		left -= 4;
		if (tlv_len > left) {
			wpa_printf(MSG_DEBUG, "EAP-TLV: TLV underrun "
				   "(tlv_len=%lu left=%lu)",
				   (unsigned long) tlv_len,
				   (unsigned long) left);
			return -1;
		}
		switch (tlv_type) {
		case EAP_TLV_RESULT_TLV:
			result_tlv = pos;
			result_tlv_len = tlv_len;
			break;
		default:
			wpa_printf(MSG_DEBUG, "EAP-TLV: Unsupported TLV Type "
				   "%d%s", tlv_type,
				   mandatory ? " (mandatory)" : "");
			if (mandatory) {
				/* NAK TLV and ignore all TLVs in this packet.
				 */
				*resp = eap_tlv_build_nak(hdr->identifier,
							  tlv_type, resp_len);
				return *resp == NULL ? -1 : 0;
			}
			/* Ignore this TLV, but process other TLVs */
			break;
		}

		pos += tlv_len;
		left -= tlv_len;
	}
	if (left) {
		wpa_printf(MSG_DEBUG, "EAP-TLV: Last TLV too short in "
			   "Request (left=%lu)", (unsigned long) left);
		return -1;
	}

	/* Process supported TLVs */
	if (result_tlv) {
		int status, resp_status;
		wpa_hexdump(MSG_DEBUG, "EAP-TLV: Result TLV",
			    result_tlv, result_tlv_len);
		if (result_tlv_len < 2) {
			wpa_printf(MSG_INFO, "EAP-TLV: Too short Result TLV "
				   "(len=%lu)",
				   (unsigned long) result_tlv_len);
			return -1;
		}
		status = WPA_GET_BE16(result_tlv);
		if (status == EAP_TLV_RESULT_SUCCESS) {
			wpa_printf(MSG_INFO, "EAP-TLV: TLV Result - Success "
				   "- EAP-TLV/Phase2 Completed");
			if (force_failure) {
				wpa_printf(MSG_INFO, "EAP-TLV: Earlier failure"
					   " - force failed Phase 2");
				resp_status = EAP_TLV_RESULT_FAILURE;
				ret->decision = DECISION_FAIL;
			} else {
				resp_status = EAP_TLV_RESULT_SUCCESS;
				ret->decision = DECISION_UNCOND_SUCC;
			}
		} else if (status == EAP_TLV_RESULT_FAILURE) {
			wpa_printf(MSG_INFO, "EAP-TLV: TLV Result - Failure");
			resp_status = EAP_TLV_RESULT_FAILURE;
			ret->decision = DECISION_FAIL;
		} else {
			wpa_printf(MSG_INFO, "EAP-TLV: Unknown TLV Result "
				   "Status %d", status);
			resp_status = EAP_TLV_RESULT_FAILURE;
			ret->decision = DECISION_FAIL;
		}
		ret->methodState = METHOD_DONE;

		*resp = eap_tlv_build_result(hdr->identifier, resp_status,
					     resp_len);
	}

	return 0;
}
