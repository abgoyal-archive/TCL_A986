
#include <net/cfg80211.h>
#include "ieee80211_i.h"

void mac80211_ev_michael_mic_failure(struct ieee80211_sub_if_data *sdata, int keyidx,
				     struct ieee80211_hdr *hdr, const u8 *tsc,
				     gfp_t gfp)
{
	cfg80211_michael_mic_failure(sdata->dev, hdr->addr2,
				     (hdr->addr1[0] & 0x01) ?
				     NL80211_KEYTYPE_GROUP :
				     NL80211_KEYTYPE_PAIRWISE,
				     keyidx, tsc, gfp);
}
