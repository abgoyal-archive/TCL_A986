

#include "includes.h"

#include "common.h"
#include "wpa.h"
#include "eloop.h"
#include "crypto/sha1.h"
#include "crypto/sha256.h"
#include "wpa_i.h"
#include "eapol_supp/eapol_supp_sm.h"
#include "pmksa_cache.h"

#if defined(IEEE8021X_EAPOL) && !defined(CONFIG_NO_WPA2)

static const int pmksa_cache_max_entries = 32;

struct rsn_pmksa_cache {
	struct rsn_pmksa_cache_entry *pmksa; /* PMKSA cache */
	int pmksa_count; /* number of entries in PMKSA cache */
	struct wpa_sm *sm; /* TODO: get rid of this reference(?) */

	void (*free_cb)(struct rsn_pmksa_cache_entry *entry, void *ctx,
			int replace);
	void *ctx;
};


static void rsn_pmkid(const u8 *pmk, size_t pmk_len, const u8 *aa,
		      const u8 *spa, u8 *pmkid, int use_sha256)
{
	char *title = "PMK Name";
	const u8 *addr[3];
	const size_t len[3] = { 8, ETH_ALEN, ETH_ALEN };
	unsigned char hash[SHA256_MAC_LEN];

	addr[0] = (u8 *) title;
	addr[1] = aa;
	addr[2] = spa;

#ifdef CONFIG_IEEE80211W
	if (use_sha256)
		hmac_sha256_vector(pmk, pmk_len, 3, addr, len, hash);
	else
#endif /* CONFIG_IEEE80211W */
		hmac_sha1_vector(pmk, pmk_len, 3, addr, len, hash);
	os_memcpy(pmkid, hash, PMKID_LEN);
}


static void pmksa_cache_set_expiration(struct rsn_pmksa_cache *pmksa);


static void _pmksa_cache_free_entry(struct rsn_pmksa_cache_entry *entry)
{
	os_free(entry);
}


static void pmksa_cache_free_entry(struct rsn_pmksa_cache *pmksa,
				   struct rsn_pmksa_cache_entry *entry,
				   int replace)
{
	pmksa->pmksa_count--;
	pmksa->free_cb(entry, pmksa->ctx, replace);
	_pmksa_cache_free_entry(entry);
}


static void pmksa_cache_expire(void *eloop_ctx, void *timeout_ctx)
{
	struct rsn_pmksa_cache *pmksa = eloop_ctx;
	struct os_time now;

	os_get_time(&now);
	while (pmksa->pmksa && pmksa->pmksa->expiration <= now.sec) {
		struct rsn_pmksa_cache_entry *entry = pmksa->pmksa;
		pmksa->pmksa = entry->next;
		wpa_printf(MSG_DEBUG, "RSN: expired PMKSA cache entry for "
			   MACSTR, MAC2STR(entry->aa));
		pmksa_cache_free_entry(pmksa, entry, 0);
	}

	pmksa_cache_set_expiration(pmksa);
}


static void pmksa_cache_reauth(void *eloop_ctx, void *timeout_ctx)
{
	struct rsn_pmksa_cache *pmksa = eloop_ctx;
	pmksa->sm->cur_pmksa = NULL;
	eapol_sm_request_reauth(pmksa->sm->eapol);
}


static void pmksa_cache_set_expiration(struct rsn_pmksa_cache *pmksa)
{
	int sec;
	struct rsn_pmksa_cache_entry *entry;
	struct os_time now;

	eloop_cancel_timeout(pmksa_cache_expire, pmksa, NULL);
	eloop_cancel_timeout(pmksa_cache_reauth, pmksa, NULL);
	if (pmksa->pmksa == NULL)
		return;
	os_get_time(&now);
	sec = pmksa->pmksa->expiration - now.sec;
	if (sec < 0)
		sec = 0;
	eloop_register_timeout(sec + 1, 0, pmksa_cache_expire, pmksa, NULL);

	entry = pmksa->sm->cur_pmksa ? pmksa->sm->cur_pmksa :
		pmksa_cache_get(pmksa, pmksa->sm->bssid, NULL);
	if (entry) {
		sec = pmksa->pmksa->reauth_time - now.sec;
		if (sec < 0)
			sec = 0;
		eloop_register_timeout(sec, 0, pmksa_cache_reauth, pmksa,
				       NULL);
	}
}


struct rsn_pmksa_cache_entry *
pmksa_cache_add(struct rsn_pmksa_cache *pmksa, const u8 *pmk, size_t pmk_len,
		const u8 *aa, const u8 *spa, void *network_ctx, int akmp)
{
	struct rsn_pmksa_cache_entry *entry, *pos, *prev;
	struct os_time now;

	if (pmksa->sm->proto != WPA_PROTO_RSN || pmk_len > PMK_LEN)
		return NULL;

	entry = os_zalloc(sizeof(*entry));
	if (entry == NULL)
		return NULL;
	os_memcpy(entry->pmk, pmk, pmk_len);
	entry->pmk_len = pmk_len;
	rsn_pmkid(pmk, pmk_len, aa, spa, entry->pmkid,
		  wpa_key_mgmt_sha256(akmp));
	os_get_time(&now);
	entry->expiration = now.sec + pmksa->sm->dot11RSNAConfigPMKLifetime;
	entry->reauth_time = now.sec + pmksa->sm->dot11RSNAConfigPMKLifetime *
		pmksa->sm->dot11RSNAConfigPMKReauthThreshold / 100;
	entry->akmp = akmp;
	os_memcpy(entry->aa, aa, ETH_ALEN);
	entry->network_ctx = network_ctx;

	/* Replace an old entry for the same Authenticator (if found) with the
	 * new entry */
	pos = pmksa->pmksa;
	prev = NULL;
	while (pos) {
		if (os_memcmp(aa, pos->aa, ETH_ALEN) == 0) {
			if (pos->pmk_len == pmk_len &&
			    os_memcmp(pos->pmk, pmk, pmk_len) == 0 &&
			    os_memcmp(pos->pmkid, entry->pmkid, PMKID_LEN) ==
			    0) {
				wpa_printf(MSG_DEBUG, "WPA: reusing previous "
					   "PMKSA entry");
				os_free(entry);
				return pos;
			}
			if (prev == NULL)
				pmksa->pmksa = pos->next;
			else
				prev->next = pos->next;
			if (pos == pmksa->sm->cur_pmksa) {
				/* We are about to replace the current PMKSA
				 * cache entry. This happens when the PMKSA
				 * caching attempt fails, so we don't want to
				 * force pmksa_cache_free_entry() to disconnect
				 * at this point. Let's just make sure the old
				 * PMKSA cache entry will not be used in the
				 * future.
				 */
				wpa_printf(MSG_DEBUG, "RSN: replacing current "
					   "PMKSA entry");
				pmksa->sm->cur_pmksa = NULL;
			}
			wpa_printf(MSG_DEBUG, "RSN: Replace PMKSA entry for "
				   "the current AP");
			pmksa_cache_free_entry(pmksa, pos, 1);
			break;
		}
		prev = pos;
		pos = pos->next;
	}

	if (pmksa->pmksa_count >= pmksa_cache_max_entries && pmksa->pmksa) {
		/* Remove the oldest entry to make room for the new entry */
		pos = pmksa->pmksa;
		pmksa->pmksa = pos->next;
		wpa_printf(MSG_DEBUG, "RSN: removed the oldest PMKSA cache "
			   "entry (for " MACSTR ") to make room for new one",
			   MAC2STR(pos->aa));
		wpa_sm_remove_pmkid(pmksa->sm, pos->aa, pos->pmkid);
		pmksa_cache_free_entry(pmksa, pos, 0);
	}

	/* Add the new entry; order by expiration time */
	pos = pmksa->pmksa;
	prev = NULL;
	while (pos) {
		if (pos->expiration > entry->expiration)
			break;
		prev = pos;
		pos = pos->next;
	}
	if (prev == NULL) {
		entry->next = pmksa->pmksa;
		pmksa->pmksa = entry;
		pmksa_cache_set_expiration(pmksa);
	} else {
		entry->next = prev->next;
		prev->next = entry;
	}
	pmksa->pmksa_count++;
	wpa_printf(MSG_DEBUG, "RSN: added PMKSA cache entry for " MACSTR,
		   MAC2STR(entry->aa));
	wpa_sm_add_pmkid(pmksa->sm, entry->aa, entry->pmkid);

	return entry;
}


void pmksa_cache_deinit(struct rsn_pmksa_cache *pmksa)
{
	struct rsn_pmksa_cache_entry *entry, *prev;

	if (pmksa == NULL)
		return;

	entry = pmksa->pmksa;
	pmksa->pmksa = NULL;
	while (entry) {
		prev = entry;
		entry = entry->next;
		os_free(prev);
	}
	pmksa_cache_set_expiration(pmksa);
	os_free(pmksa);
}


struct rsn_pmksa_cache_entry * pmksa_cache_get(struct rsn_pmksa_cache *pmksa,
					       const u8 *aa, const u8 *pmkid)
{
	struct rsn_pmksa_cache_entry *entry = pmksa->pmksa;
	while (entry) {
		if ((aa == NULL || os_memcmp(entry->aa, aa, ETH_ALEN) == 0) &&
		    (pmkid == NULL ||
		     os_memcmp(entry->pmkid, pmkid, PMKID_LEN) == 0))
			return entry;
		entry = entry->next;
	}
	return NULL;
}


void pmksa_cache_notify_reconfig(struct rsn_pmksa_cache *pmksa)
{
	struct rsn_pmksa_cache_entry *entry = pmksa->pmksa;
	while (entry) {
		entry->network_ctx = NULL;
		entry = entry->next;
	}
}


static struct rsn_pmksa_cache_entry *
pmksa_cache_clone_entry(struct rsn_pmksa_cache *pmksa,
			const struct rsn_pmksa_cache_entry *old_entry,
			const u8 *aa)
{
	struct rsn_pmksa_cache_entry *new_entry;

	new_entry = pmksa_cache_add(pmksa, old_entry->pmk, old_entry->pmk_len,
				    aa, pmksa->sm->own_addr,
				    old_entry->network_ctx, old_entry->akmp);
	if (new_entry == NULL)
		return NULL;

	/* TODO: reorder entries based on expiration time? */
	new_entry->expiration = old_entry->expiration;
	new_entry->opportunistic = 1;

	return new_entry;
}


struct rsn_pmksa_cache_entry *
pmksa_cache_get_opportunistic(struct rsn_pmksa_cache *pmksa, void *network_ctx,
			      const u8 *aa)
{
	struct rsn_pmksa_cache_entry *entry = pmksa->pmksa;

	if (network_ctx == NULL)
		return NULL;
	while (entry) {
		if (entry->network_ctx == network_ctx) {
			entry = pmksa_cache_clone_entry(pmksa, entry, aa);
			if (entry) {
				wpa_printf(MSG_DEBUG, "RSN: added "
					   "opportunistic PMKSA cache entry "
					   "for " MACSTR, MAC2STR(aa));
			}
			return entry;
		}
		entry = entry->next;
	}
	return NULL;
}


struct rsn_pmksa_cache_entry * pmksa_cache_get_current(struct wpa_sm *sm)
{
	if (sm == NULL)
		return NULL;
	return sm->cur_pmksa;
}


void pmksa_cache_clear_current(struct wpa_sm *sm)
{
	if (sm == NULL)
		return;
	sm->cur_pmksa = NULL;
}


int pmksa_cache_set_current(struct wpa_sm *sm, const u8 *pmkid,
			    const u8 *bssid, void *network_ctx,
			    int try_opportunistic)
{
	struct rsn_pmksa_cache *pmksa = sm->pmksa;
	sm->cur_pmksa = NULL;
	if (pmkid)
		sm->cur_pmksa = pmksa_cache_get(pmksa, NULL, pmkid);
	if (sm->cur_pmksa == NULL && bssid)
		sm->cur_pmksa = pmksa_cache_get(pmksa, bssid, NULL);
	if (sm->cur_pmksa == NULL && try_opportunistic && bssid)
		sm->cur_pmksa = pmksa_cache_get_opportunistic(pmksa,
							      network_ctx,
							      bssid);
	if (sm->cur_pmksa) {
		wpa_hexdump(MSG_DEBUG, "RSN: PMKID",
			    sm->cur_pmksa->pmkid, PMKID_LEN);
		return 0;
	}
	return -1;
}


int pmksa_cache_list(struct wpa_sm *sm, char *buf, size_t len)
{
	int i, ret;
	char *pos = buf;
	struct rsn_pmksa_cache_entry *entry;
	struct os_time now;

	os_get_time(&now);
	ret = os_snprintf(pos, buf + len - pos,
			  "Index / AA / PMKID / expiration (in seconds) / "
			  "opportunistic\n");
	if (ret < 0 || ret >= buf + len - pos)
		return pos - buf;
	pos += ret;
	i = 0;
	entry = sm->pmksa->pmksa;
	while (entry) {
		i++;
		ret = os_snprintf(pos, buf + len - pos, "%d " MACSTR " ",
				  i, MAC2STR(entry->aa));
		if (ret < 0 || ret >= buf + len - pos)
			return pos - buf;
		pos += ret;
		pos += wpa_snprintf_hex(pos, buf + len - pos, entry->pmkid,
					PMKID_LEN);
		ret = os_snprintf(pos, buf + len - pos, " %d %d\n",
				  (int) (entry->expiration - now.sec),
				  entry->opportunistic);
		if (ret < 0 || ret >= buf + len - pos)
			return pos - buf;
		pos += ret;
		entry = entry->next;
	}
	return pos - buf;
}


struct rsn_pmksa_cache *
pmksa_cache_init(void (*free_cb)(struct rsn_pmksa_cache_entry *entry,
				 void *ctx, int replace),
		 void *ctx, struct wpa_sm *sm)
{
	struct rsn_pmksa_cache *pmksa;

	pmksa = os_zalloc(sizeof(*pmksa));
	if (pmksa) {
		pmksa->free_cb = free_cb;
		pmksa->ctx = ctx;
		pmksa->sm = sm;
	}

	return pmksa;
}

#endif /* IEEE8021X_EAPOL and !CONFIG_NO_WPA2 */
