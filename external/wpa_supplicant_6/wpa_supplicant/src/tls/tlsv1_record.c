

#include "includes.h"

#include "common.h"
#include "md5.h"
#include "sha1.h"
#include "tlsv1_common.h"
#include "tlsv1_record.h"


int tlsv1_record_set_cipher_suite(struct tlsv1_record_layer *rl,
				  u16 cipher_suite)
{
	const struct tls_cipher_suite *suite;
	const struct tls_cipher_data *data;

	wpa_printf(MSG_DEBUG, "TLSv1: Selected cipher suite: 0x%04x",
		   cipher_suite);
	rl->cipher_suite = cipher_suite;

	suite = tls_get_cipher_suite(cipher_suite);
	if (suite == NULL)
		return -1;

	if (suite->hash == TLS_HASH_MD5) {
		rl->hash_alg = CRYPTO_HASH_ALG_HMAC_MD5;
		rl->hash_size = MD5_MAC_LEN;
	} else if (suite->hash == TLS_HASH_SHA) {
		rl->hash_alg = CRYPTO_HASH_ALG_HMAC_SHA1;
		rl->hash_size = SHA1_MAC_LEN;
	}

	data = tls_get_cipher_data(suite->cipher);
	if (data == NULL)
		return -1;

	rl->key_material_len = data->key_material;
	rl->iv_size = data->block_size;
	rl->cipher_alg = data->alg;

	return 0;
}


int tlsv1_record_change_write_cipher(struct tlsv1_record_layer *rl)
{
	wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - New write cipher suite "
		   "0x%04x", rl->cipher_suite);
	rl->write_cipher_suite = rl->cipher_suite;
	os_memset(rl->write_seq_num, 0, TLS_SEQ_NUM_LEN);

	if (rl->write_cbc) {
		crypto_cipher_deinit(rl->write_cbc);
		rl->write_cbc = NULL;
	}
	if (rl->cipher_alg != CRYPTO_CIPHER_NULL) {
		rl->write_cbc = crypto_cipher_init(rl->cipher_alg,
						   rl->write_iv, rl->write_key,
						   rl->key_material_len);
		if (rl->write_cbc == NULL) {
			wpa_printf(MSG_DEBUG, "TLSv1: Failed to initialize "
				   "cipher");
			return -1;
		}
	}

	return 0;
}


int tlsv1_record_change_read_cipher(struct tlsv1_record_layer *rl)
{
	wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - New read cipher suite "
		   "0x%04x", rl->cipher_suite);
	rl->read_cipher_suite = rl->cipher_suite;
	os_memset(rl->read_seq_num, 0, TLS_SEQ_NUM_LEN);

	if (rl->read_cbc) {
		crypto_cipher_deinit(rl->read_cbc);
		rl->read_cbc = NULL;
	}
	if (rl->cipher_alg != CRYPTO_CIPHER_NULL) {
		rl->read_cbc = crypto_cipher_init(rl->cipher_alg,
						  rl->read_iv, rl->read_key,
						  rl->key_material_len);
		if (rl->read_cbc == NULL) {
			wpa_printf(MSG_DEBUG, "TLSv1: Failed to initialize "
				   "cipher");
			return -1;
		}
	}

	return 0;
}


int tlsv1_record_send(struct tlsv1_record_layer *rl, u8 content_type, u8 *buf,
		      size_t buf_size, size_t payload_len, size_t *out_len)
{
	u8 *pos, *ct_start, *length, *payload;
	struct crypto_hash *hmac;
	size_t clen;

	pos = buf;
	/* ContentType type */
	ct_start = pos;
	*pos++ = content_type;
	/* ProtocolVersion version */
	WPA_PUT_BE16(pos, TLS_VERSION);
	pos += 2;
	/* uint16 length */
	length = pos;
	WPA_PUT_BE16(length, payload_len);
	pos += 2;

	/* opaque fragment[TLSPlaintext.length] */
	payload = pos;
	pos += payload_len;

	if (rl->write_cipher_suite != TLS_NULL_WITH_NULL_NULL) {
		hmac = crypto_hash_init(rl->hash_alg, rl->write_mac_secret,
					rl->hash_size);
		if (hmac == NULL) {
			wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - Failed "
				   "to initialize HMAC");
			return -1;
		}
		crypto_hash_update(hmac, rl->write_seq_num, TLS_SEQ_NUM_LEN);
		/* type + version + length + fragment */
		crypto_hash_update(hmac, ct_start, pos - ct_start);
		clen = buf + buf_size - pos;
		if (clen < rl->hash_size) {
			wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - Not "
				   "enough room for MAC");
			crypto_hash_finish(hmac, NULL, NULL);
			return -1;
		}

		if (crypto_hash_finish(hmac, pos, &clen) < 0) {
			wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - Failed "
				   "to calculate HMAC");
			return -1;
		}
		wpa_hexdump(MSG_MSGDUMP, "TLSv1: Record Layer - Write HMAC",
			    pos, clen);
		pos += clen;
		if (rl->iv_size) {
			size_t len = pos - payload;
			size_t pad;
			pad = (len + 1) % rl->iv_size;
			if (pad)
				pad = rl->iv_size - pad;
			if (pos + pad + 1 > buf + buf_size) {
				wpa_printf(MSG_DEBUG, "TLSv1: No room for "
					   "block cipher padding");
				return -1;
			}
			os_memset(pos, pad, pad + 1);
			pos += pad + 1;
		}

		if (crypto_cipher_encrypt(rl->write_cbc, payload,
					  payload, pos - payload) < 0)
			return -1;
	}

	WPA_PUT_BE16(length, pos - length - 2);
	inc_byte_array(rl->write_seq_num, TLS_SEQ_NUM_LEN);

	*out_len = pos - buf;

	return 0;
}


int tlsv1_record_receive(struct tlsv1_record_layer *rl,
			 const u8 *in_data, size_t in_len,
			 u8 *out_data, size_t *out_len, u8 *alert)
{
	size_t i, rlen, hlen;
	u8 padlen;
	struct crypto_hash *hmac;
	u8 len[2], hash[100];

	wpa_hexdump(MSG_MSGDUMP, "TLSv1: Record Layer - Received",
		    in_data, in_len);

	if (in_len < TLS_RECORD_HEADER_LEN) {
		wpa_printf(MSG_DEBUG, "TLSv1: Too short record (in_len=%lu)",
			   (unsigned long) in_len);
		*alert = TLS_ALERT_DECODE_ERROR;
		return -1;
	}

	wpa_printf(MSG_DEBUG, "TLSv1: Received content type %d version %d.%d "
		   "length %d", in_data[0], in_data[1], in_data[2],
		   WPA_GET_BE16(in_data + 3));

	if (in_data[0] != TLS_CONTENT_TYPE_HANDSHAKE &&
	    in_data[0] != TLS_CONTENT_TYPE_CHANGE_CIPHER_SPEC &&
	    in_data[0] != TLS_CONTENT_TYPE_ALERT &&
	    in_data[0] != TLS_CONTENT_TYPE_APPLICATION_DATA) {
		wpa_printf(MSG_DEBUG, "TLSv1: Unexpected content type 0x%x",
			   in_data[0]);
		*alert = TLS_ALERT_UNEXPECTED_MESSAGE;
		return -1;
	}

	if (WPA_GET_BE16(in_data + 1) != TLS_VERSION) {
		wpa_printf(MSG_DEBUG, "TLSv1: Unexpected protocol version "
			   "%d.%d", in_data[1], in_data[2]);
		*alert = TLS_ALERT_PROTOCOL_VERSION;
		return -1;
	}

	rlen = WPA_GET_BE16(in_data + 3);

	/* TLSCiphertext must not be more than 2^14+2048 bytes */
	if (TLS_RECORD_HEADER_LEN + rlen > 18432) {
		wpa_printf(MSG_DEBUG, "TLSv1: Record overflow (len=%lu)",
			   (unsigned long) (TLS_RECORD_HEADER_LEN + rlen));
		*alert = TLS_ALERT_RECORD_OVERFLOW;
		return -1;
	}

	in_data += TLS_RECORD_HEADER_LEN;
	in_len -= TLS_RECORD_HEADER_LEN;

	if (rlen > in_len) {
		wpa_printf(MSG_DEBUG, "TLSv1: Not all record data included "
			   "(rlen=%lu > in_len=%lu)",
			   (unsigned long) rlen, (unsigned long) in_len);
		*alert = TLS_ALERT_DECODE_ERROR;
		return -1;
	}

	in_len = rlen;

	if (*out_len < in_len) {
		wpa_printf(MSG_DEBUG, "TLSv1: Not enough output buffer for "
			   "processing received record");
		*alert = TLS_ALERT_INTERNAL_ERROR;
		return -1;
	}

	os_memcpy(out_data, in_data, in_len);
	*out_len = in_len;

	if (rl->read_cipher_suite != TLS_NULL_WITH_NULL_NULL) {
		if (crypto_cipher_decrypt(rl->read_cbc, out_data,
					  out_data, in_len) < 0) {
			*alert = TLS_ALERT_DECRYPTION_FAILED;
			return -1;
		}
		if (rl->iv_size) {
			if (in_len == 0) {
				wpa_printf(MSG_DEBUG, "TLSv1: Too short record"
					   " (no pad)");
				*alert = TLS_ALERT_DECODE_ERROR;
				return -1;
			}
			padlen = out_data[in_len - 1];
			if (padlen >= in_len) {
				wpa_printf(MSG_DEBUG, "TLSv1: Incorrect pad "
					   "length (%u, in_len=%lu) in "
					   "received record",
					   padlen, (unsigned long) in_len);
				*alert = TLS_ALERT_DECRYPTION_FAILED;
				return -1;
			}
			for (i = in_len - padlen; i < in_len; i++) {
				if (out_data[i] != padlen) {
					wpa_hexdump(MSG_DEBUG,
						    "TLSv1: Invalid pad in "
						    "received record",
						    out_data + in_len - padlen,
						    padlen);
					*alert = TLS_ALERT_DECRYPTION_FAILED;
					return -1;
				}
			}

			*out_len -= padlen + 1;
		}

		wpa_hexdump(MSG_MSGDUMP,
			    "TLSv1: Record Layer - Decrypted data",
			    out_data, in_len);

		if (*out_len < rl->hash_size) {
			wpa_printf(MSG_DEBUG, "TLSv1: Too short record; no "
				   "hash value");
			*alert = TLS_ALERT_INTERNAL_ERROR;
			return -1;
		}

		*out_len -= rl->hash_size;

		hmac = crypto_hash_init(rl->hash_alg, rl->read_mac_secret,
					rl->hash_size);
		if (hmac == NULL) {
			wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - Failed "
				   "to initialize HMAC");
			*alert = TLS_ALERT_INTERNAL_ERROR;
			return -1;
		}

		crypto_hash_update(hmac, rl->read_seq_num, TLS_SEQ_NUM_LEN);
		/* type + version + length + fragment */
		crypto_hash_update(hmac, in_data - TLS_RECORD_HEADER_LEN, 3);
		WPA_PUT_BE16(len, *out_len);
		crypto_hash_update(hmac, len, 2);
		crypto_hash_update(hmac, out_data, *out_len);
		hlen = sizeof(hash);
		if (crypto_hash_finish(hmac, hash, &hlen) < 0) {
			wpa_printf(MSG_DEBUG, "TLSv1: Record Layer - Failed "
				   "to calculate HMAC");
			return -1;
		}
		if (hlen != rl->hash_size ||
		    os_memcmp(hash, out_data + *out_len, hlen) != 0) {
			wpa_printf(MSG_DEBUG, "TLSv1: Invalid HMAC value in "
				   "received message");
			*alert = TLS_ALERT_BAD_RECORD_MAC;
			return -1;
		}
	}

	/* TLSCompressed must not be more than 2^14+1024 bytes */
	if (TLS_RECORD_HEADER_LEN + *out_len > 17408) {
		wpa_printf(MSG_DEBUG, "TLSv1: Record overflow (len=%lu)",
			   (unsigned long) (TLS_RECORD_HEADER_LEN + *out_len));
		*alert = TLS_ALERT_RECORD_OVERFLOW;
		return -1;
	}

	inc_byte_array(rl->read_seq_num, TLS_SEQ_NUM_LEN);

	return 0;
}
