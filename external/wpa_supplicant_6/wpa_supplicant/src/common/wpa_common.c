

#include "includes.h"

#include "common.h"
#include "crypto/md5.h"
#include "crypto/sha1.h"
#include "crypto/sha256.h"
#include "crypto/aes_wrap.h"
#include "crypto/crypto.h"
#include "ieee802_11_defs.h"
#include "defs.h"
#include "wpa_common.h"


int wpa_eapol_key_mic(const u8 *key, int ver, const u8 *buf, size_t len,
		      u8 *mic)
{
	u8 hash[SHA1_MAC_LEN];

	switch (ver) {
	case WPA_KEY_INFO_TYPE_HMAC_MD5_RC4:
		hmac_md5(key, 16, buf, len, mic);
		break;
	case WPA_KEY_INFO_TYPE_HMAC_SHA1_AES:
		hmac_sha1(key, 16, buf, len, hash);
		os_memcpy(mic, hash, MD5_MAC_LEN);
		break;
#if defined(CONFIG_IEEE80211R) || defined(CONFIG_IEEE80211W)
	case WPA_KEY_INFO_TYPE_AES_128_CMAC:
		return omac1_aes_128(key, buf, len, mic);
#endif /* CONFIG_IEEE80211R || CONFIG_IEEE80211W */
	default:
		return -1;
	}

	return 0;
}


void wpa_pmk_to_ptk(const u8 *pmk, size_t pmk_len, const char *label,
		    const u8 *addr1, const u8 *addr2,
		    const u8 *nonce1, const u8 *nonce2,
		    u8 *ptk, size_t ptk_len, int use_sha256)
{
	u8 data[2 * ETH_ALEN + 2 * WPA_NONCE_LEN];

	if (os_memcmp(addr1, addr2, ETH_ALEN) < 0) {
		os_memcpy(data, addr1, ETH_ALEN);
		os_memcpy(data + ETH_ALEN, addr2, ETH_ALEN);
	} else {
		os_memcpy(data, addr2, ETH_ALEN);
		os_memcpy(data + ETH_ALEN, addr1, ETH_ALEN);
	}

	if (os_memcmp(nonce1, nonce2, WPA_NONCE_LEN) < 0) {
		os_memcpy(data + 2 * ETH_ALEN, nonce1, WPA_NONCE_LEN);
		os_memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce2,
			  WPA_NONCE_LEN);
	} else {
		os_memcpy(data + 2 * ETH_ALEN, nonce2, WPA_NONCE_LEN);
		os_memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce1,
			  WPA_NONCE_LEN);
	}

#ifdef CONFIG_IEEE80211W
	if (use_sha256)
		sha256_prf(pmk, pmk_len, label, data, sizeof(data),
			   ptk, ptk_len);
	else
#endif /* CONFIG_IEEE80211W */
		sha1_prf(pmk, pmk_len, label, data, sizeof(data), ptk,
			 ptk_len);

	wpa_printf(MSG_DEBUG, "WPA: PTK derivation - A1=" MACSTR " A2=" MACSTR,
		   MAC2STR(addr1), MAC2STR(addr2));
	wpa_hexdump_key(MSG_DEBUG, "WPA: PMK", pmk, pmk_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: PTK", ptk, ptk_len);
}


#ifdef CONFIG_IEEE80211R
int wpa_ft_mic(const u8 *kck, const u8 *sta_addr, const u8 *ap_addr,
	       u8 transaction_seqnum, const u8 *mdie, size_t mdie_len,
	       const u8 *ftie, size_t ftie_len,
	       const u8 *rsnie, size_t rsnie_len,
	       const u8 *ric, size_t ric_len, u8 *mic)
{
	u8 *buf, *pos;
	size_t buf_len;

	buf_len = 2 * ETH_ALEN + 1 + mdie_len + ftie_len + rsnie_len + ric_len;
	buf = os_malloc(buf_len);
	if (buf == NULL)
		return -1;

	pos = buf;
	os_memcpy(pos, sta_addr, ETH_ALEN);
	pos += ETH_ALEN;
	os_memcpy(pos, ap_addr, ETH_ALEN);
	pos += ETH_ALEN;
	*pos++ = transaction_seqnum;
	if (rsnie) {
		os_memcpy(pos, rsnie, rsnie_len);
		pos += rsnie_len;
	}
	if (mdie) {
		os_memcpy(pos, mdie, mdie_len);
		pos += mdie_len;
	}
	if (ftie) {
		struct rsn_ftie *_ftie;
		os_memcpy(pos, ftie, ftie_len);
		if (ftie_len < 2 + sizeof(*_ftie)) {
			os_free(buf);
			return -1;
		}
		_ftie = (struct rsn_ftie *) (pos + 2);
		os_memset(_ftie->mic, 0, sizeof(_ftie->mic));
		pos += ftie_len;
	}
	if (ric) {
		os_memcpy(pos, ric, ric_len);
		pos += ric_len;
	}

	wpa_hexdump(MSG_MSGDUMP, "FT: MIC data", buf, pos - buf);
	if (omac1_aes_128(kck, buf, pos - buf, mic)) {
		os_free(buf);
		return -1;
	}

	os_free(buf);

	return 0;
}
#endif /* CONFIG_IEEE80211R */


#ifndef CONFIG_NO_WPA2
static int rsn_selector_to_bitfield(const u8 *s)
{
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_NONE)
		return WPA_CIPHER_NONE;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_WEP40)
		return WPA_CIPHER_WEP40;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_TKIP)
		return WPA_CIPHER_TKIP;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_CCMP)
		return WPA_CIPHER_CCMP;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_WEP104)
		return WPA_CIPHER_WEP104;
#ifdef CONFIG_IEEE80211W
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_AES_128_CMAC)
		return WPA_CIPHER_AES_128_CMAC;
#endif /* CONFIG_IEEE80211W */
	return 0;
}


static int rsn_key_mgmt_to_bitfield(const u8 *s)
{
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_UNSPEC_802_1X)
		return WPA_KEY_MGMT_IEEE8021X;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_PSK_OVER_802_1X)
		return WPA_KEY_MGMT_PSK;
#ifdef CONFIG_IEEE80211R
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_FT_802_1X)
		return WPA_KEY_MGMT_FT_IEEE8021X;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_FT_PSK)
		return WPA_KEY_MGMT_FT_PSK;
#endif /* CONFIG_IEEE80211R */
#ifdef CONFIG_IEEE80211W
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_802_1X_SHA256)
		return WPA_KEY_MGMT_IEEE8021X_SHA256;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_PSK_SHA256)
		return WPA_KEY_MGMT_PSK_SHA256;
#endif /* CONFIG_IEEE80211W */
	return 0;
}
#endif /* CONFIG_NO_WPA2 */


int wpa_parse_wpa_ie_rsn(const u8 *rsn_ie, size_t rsn_ie_len,
			 struct wpa_ie_data *data)
{
#ifndef CONFIG_NO_WPA2
	const struct rsn_ie_hdr *hdr;
	const u8 *pos;
	int left;
	int i, count;

	os_memset(data, 0, sizeof(*data));
	data->proto = WPA_PROTO_RSN;
	data->pairwise_cipher = WPA_CIPHER_CCMP;
	data->group_cipher = WPA_CIPHER_CCMP;
	data->key_mgmt = WPA_KEY_MGMT_IEEE8021X;
	data->capabilities = 0;
	data->pmkid = NULL;
	data->num_pmkid = 0;
#ifdef CONFIG_IEEE80211W
	data->mgmt_group_cipher = WPA_CIPHER_AES_128_CMAC;
#else /* CONFIG_IEEE80211W */
	data->mgmt_group_cipher = 0;
#endif /* CONFIG_IEEE80211W */

	if (rsn_ie_len == 0) {
		/* No RSN IE - fail silently */
		return -1;
	}

	if (rsn_ie_len < sizeof(struct rsn_ie_hdr)) {
		wpa_printf(MSG_DEBUG, "%s: ie len too short %lu",
			   __func__, (unsigned long) rsn_ie_len);
		return -1;
	}

	hdr = (const struct rsn_ie_hdr *) rsn_ie;

	if (hdr->elem_id != WLAN_EID_RSN ||
	    hdr->len != rsn_ie_len - 2 ||
	    WPA_GET_LE16(hdr->version) != RSN_VERSION) {
		wpa_printf(MSG_DEBUG, "%s: malformed ie or unknown version",
			   __func__);
		return -2;
	}

	pos = (const u8 *) (hdr + 1);
	left = rsn_ie_len - sizeof(*hdr);

	if (left >= RSN_SELECTOR_LEN) {
		data->group_cipher = rsn_selector_to_bitfield(pos);
#ifdef CONFIG_IEEE80211W
		if (data->group_cipher == WPA_CIPHER_AES_128_CMAC) {
			wpa_printf(MSG_DEBUG, "%s: AES-128-CMAC used as group "
				   "cipher", __func__);
			return -1;
		}
#endif /* CONFIG_IEEE80211W */
		pos += RSN_SELECTOR_LEN;
		left -= RSN_SELECTOR_LEN;
	} else if (left > 0) {
		wpa_printf(MSG_DEBUG, "%s: ie length mismatch, %u too much",
			   __func__, left);
		return -3;
	}

	if (left >= 2) {
		data->pairwise_cipher = 0;
		count = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (count == 0 || left < count * RSN_SELECTOR_LEN) {
			wpa_printf(MSG_DEBUG, "%s: ie count botch (pairwise), "
				   "count %u left %u", __func__, count, left);
			return -4;
		}
		for (i = 0; i < count; i++) {
			data->pairwise_cipher |= rsn_selector_to_bitfield(pos);
			pos += RSN_SELECTOR_LEN;
			left -= RSN_SELECTOR_LEN;
		}
#ifdef CONFIG_IEEE80211W
		if (data->pairwise_cipher & WPA_CIPHER_AES_128_CMAC) {
			wpa_printf(MSG_DEBUG, "%s: AES-128-CMAC used as "
				   "pairwise cipher", __func__);
			return -1;
		}
#endif /* CONFIG_IEEE80211W */
	} else if (left == 1) {
		wpa_printf(MSG_DEBUG, "%s: ie too short (for key mgmt)",
			   __func__);
		return -5;
	}

	if (left >= 2) {
		data->key_mgmt = 0;
		count = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (count == 0 || left < count * RSN_SELECTOR_LEN) {
			wpa_printf(MSG_DEBUG, "%s: ie count botch (key mgmt), "
				   "count %u left %u", __func__, count, left);
			return -6;
		}
		for (i = 0; i < count; i++) {
			data->key_mgmt |= rsn_key_mgmt_to_bitfield(pos);
			pos += RSN_SELECTOR_LEN;
			left -= RSN_SELECTOR_LEN;
		}
	} else if (left == 1) {
		wpa_printf(MSG_DEBUG, "%s: ie too short (for capabilities)",
			   __func__);
		return -7;
	}

	if (left >= 2) {
		data->capabilities = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
	}

	if (left >= 2) {
		data->num_pmkid = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (left < (int) data->num_pmkid * PMKID_LEN) {
			wpa_printf(MSG_DEBUG, "%s: PMKID underflow "
				   "(num_pmkid=%lu left=%d)",
				   __func__, (unsigned long) data->num_pmkid,
				   left);
			data->num_pmkid = 0;
			return -9;
		} else {
			data->pmkid = pos;
			pos += data->num_pmkid * PMKID_LEN;
			left -= data->num_pmkid * PMKID_LEN;
		}
	}

#ifdef CONFIG_IEEE80211W
	if (left >= 4) {
		data->mgmt_group_cipher = rsn_selector_to_bitfield(pos);
		if (data->mgmt_group_cipher != WPA_CIPHER_AES_128_CMAC) {
			wpa_printf(MSG_DEBUG, "%s: Unsupported management "
				   "group cipher 0x%x", __func__,
				   data->mgmt_group_cipher);
			return -10;
		}
		pos += RSN_SELECTOR_LEN;
		left -= RSN_SELECTOR_LEN;
	}
#endif /* CONFIG_IEEE80211W */

	if (left > 0) {
		wpa_printf(MSG_DEBUG, "%s: ie has %u trailing bytes - ignored",
			   __func__, left);
	}

	return 0;
#else /* CONFIG_NO_WPA2 */
	return -1;
#endif /* CONFIG_NO_WPA2 */
}


#ifdef CONFIG_IEEE80211R

void wpa_derive_pmk_r0(const u8 *xxkey, size_t xxkey_len,
		       const u8 *ssid, size_t ssid_len,
		       const u8 *mdid, const u8 *r0kh_id, size_t r0kh_id_len,
		       const u8 *s0kh_id, u8 *pmk_r0, u8 *pmk_r0_name)
{
	u8 buf[1 + WPA_MAX_SSID_LEN + MOBILITY_DOMAIN_ID_LEN + 1 +
	       FT_R0KH_ID_MAX_LEN + ETH_ALEN];
	u8 *pos, r0_key_data[48], hash[32];
	const u8 *addr[2];
	size_t len[2];

	/*
	 * R0-Key-Data = KDF-384(XXKey, "FT-R0",
	 *                       SSIDlength || SSID || MDID || R0KHlength ||
	 *                       R0KH-ID || S0KH-ID)
	 * XXKey is either the second 256 bits of MSK or PSK.
	 * PMK-R0 = L(R0-Key-Data, 0, 256)
	 * PMK-R0Name-Salt = L(R0-Key-Data, 256, 128)
	 */
	if (ssid_len > WPA_MAX_SSID_LEN || r0kh_id_len > FT_R0KH_ID_MAX_LEN)
		return;
	pos = buf;
	*pos++ = ssid_len;
	os_memcpy(pos, ssid, ssid_len);
	pos += ssid_len;
	os_memcpy(pos, mdid, MOBILITY_DOMAIN_ID_LEN);
	pos += MOBILITY_DOMAIN_ID_LEN;
	*pos++ = r0kh_id_len;
	os_memcpy(pos, r0kh_id, r0kh_id_len);
	pos += r0kh_id_len;
	os_memcpy(pos, s0kh_id, ETH_ALEN);
	pos += ETH_ALEN;

	sha256_prf(xxkey, xxkey_len, "FT-R0", buf, pos - buf,
		   r0_key_data, sizeof(r0_key_data));
	os_memcpy(pmk_r0, r0_key_data, PMK_LEN);

	/*
	 * PMKR0Name = Truncate-128(SHA-256("FT-R0N" || PMK-R0Name-Salt)
	 */
	addr[0] = (const u8 *) "FT-R0N";
	len[0] = 6;
	addr[1] = r0_key_data + PMK_LEN;
	len[1] = 16;

	sha256_vector(2, addr, len, hash);
	os_memcpy(pmk_r0_name, hash, WPA_PMK_NAME_LEN);
}


void wpa_derive_pmk_r1_name(const u8 *pmk_r0_name, const u8 *r1kh_id,
			    const u8 *s1kh_id, u8 *pmk_r1_name)
{
	u8 hash[32];
	const u8 *addr[4];
	size_t len[4];

	/*
	 * PMKR1Name = Truncate-128(SHA-256("FT-R1N" || PMKR0Name ||
	 *                                  R1KH-ID || S1KH-ID))
	 */
	addr[0] = (const u8 *) "FT-R1N";
	len[0] = 6;
	addr[1] = pmk_r0_name;
	len[1] = WPA_PMK_NAME_LEN;
	addr[2] = r1kh_id;
	len[2] = FT_R1KH_ID_LEN;
	addr[3] = s1kh_id;
	len[3] = ETH_ALEN;

	sha256_vector(4, addr, len, hash);
	os_memcpy(pmk_r1_name, hash, WPA_PMK_NAME_LEN);
}


void wpa_derive_pmk_r1(const u8 *pmk_r0, const u8 *pmk_r0_name,
		       const u8 *r1kh_id, const u8 *s1kh_id,
		       u8 *pmk_r1, u8 *pmk_r1_name)
{
	u8 buf[FT_R1KH_ID_LEN + ETH_ALEN];
	u8 *pos;

	/* PMK-R1 = KDF-256(PMK-R0, "FT-R1", R1KH-ID || S1KH-ID) */
	pos = buf;
	os_memcpy(pos, r1kh_id, FT_R1KH_ID_LEN);
	pos += FT_R1KH_ID_LEN;
	os_memcpy(pos, s1kh_id, ETH_ALEN);
	pos += ETH_ALEN;

	sha256_prf(pmk_r0, PMK_LEN, "FT-R1", buf, pos - buf, pmk_r1, PMK_LEN);

	wpa_derive_pmk_r1_name(pmk_r0_name, r1kh_id, s1kh_id, pmk_r1_name);
}


void wpa_pmk_r1_to_ptk(const u8 *pmk_r1, const u8 *snonce, const u8 *anonce,
		       const u8 *sta_addr, const u8 *bssid,
		       const u8 *pmk_r1_name,
		       u8 *ptk, size_t ptk_len, u8 *ptk_name)
{
	u8 buf[2 * WPA_NONCE_LEN + 2 * ETH_ALEN];
	u8 *pos, hash[32];
	const u8 *addr[6];
	size_t len[6];

	/*
	 * PTK = KDF-PTKLen(PMK-R1, "FT-PTK", SNonce || ANonce ||
	 *                  BSSID || STA-ADDR)
	 */
	pos = buf;
	os_memcpy(pos, snonce, WPA_NONCE_LEN);
	pos += WPA_NONCE_LEN;
	os_memcpy(pos, anonce, WPA_NONCE_LEN);
	pos += WPA_NONCE_LEN;
	os_memcpy(pos, bssid, ETH_ALEN);
	pos += ETH_ALEN;
	os_memcpy(pos, sta_addr, ETH_ALEN);
	pos += ETH_ALEN;

	sha256_prf(pmk_r1, PMK_LEN, "FT-PTK", buf, pos - buf, ptk, ptk_len);

	/*
	 * PTKName = Truncate-128(SHA-256(PMKR1Name || "FT-PTKN" || SNonce ||
	 *                                ANonce || BSSID || STA-ADDR))
	 */
	addr[0] = pmk_r1_name;
	len[0] = WPA_PMK_NAME_LEN;
	addr[1] = (const u8 *) "FT-PTKN";
	len[1] = 7;
	addr[2] = snonce;
	len[2] = WPA_NONCE_LEN;
	addr[3] = anonce;
	len[3] = WPA_NONCE_LEN;
	addr[4] = bssid;
	len[4] = ETH_ALEN;
	addr[5] = sta_addr;
	len[5] = ETH_ALEN;

	sha256_vector(6, addr, len, hash);
	os_memcpy(ptk_name, hash, WPA_PMK_NAME_LEN);
}

#endif /* CONFIG_IEEE80211R */
