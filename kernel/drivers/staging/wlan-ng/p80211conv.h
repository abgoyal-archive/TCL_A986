

#ifndef _LINUX_P80211CONV_H
#define _LINUX_P80211CONV_H

#define WLAN_ETHADDR_LEN	6
#define WLAN_IEEE_OUI_LEN	3

#define WLAN_ETHCONV_ENCAP	1
#define WLAN_ETHCONV_8021h	3

#define WLAN_ETHHDR_LEN		14

#define P80211CAPTURE_VERSION	0x80211001

#define	P80211_FRMMETA_MAGIC		0x802110

#define P80211SKB_FRMMETA(s) \
	(((((p80211_frmmeta_t *)((s)->cb))->magic) == P80211_FRMMETA_MAGIC) ? \
		((p80211_frmmeta_t *)((s)->cb)) : \
		(NULL))

#define P80211SKB_RXMETA(s) \
	(P80211SKB_FRMMETA((s)) ?  P80211SKB_FRMMETA((s))->rx : ((p80211_rxmeta_t *)(NULL)))

typedef struct p80211_rxmeta {
	struct wlandevice *wlandev;

	u64 mactime;		/* Hi-rez MAC-supplied time value */
	u64 hosttime;		/* Best-rez host supplied time value */

	unsigned int rxrate;	/* Receive data rate in 100kbps */
	unsigned int priority;	/* 0-15, 0=contention, 6=CF */
	int signal;		/* An SSI, see p80211netdev.h */
	int noise;		/* An SSI, see p80211netdev.h */
	unsigned int channel;	/* Receive channel (mostly for snifs) */
	unsigned int preamble;	/* P80211ENUM_preambletype_* */
	unsigned int encoding;	/* P80211ENUM_encoding_* */

} p80211_rxmeta_t;

typedef struct p80211_frmmeta {
	unsigned int magic;
	p80211_rxmeta_t *rx;
} p80211_frmmeta_t;

void p80211skb_free(struct wlandevice *wlandev, struct sk_buff *skb);
int p80211skb_rxmeta_attach(struct wlandevice *wlandev, struct sk_buff *skb);
void p80211skb_rxmeta_detach(struct sk_buff *skb);

typedef struct p80211_caphdr {
	u32 version;
	u32 length;
	u64 mactime;
	u64 hosttime;
	u32 phytype;
	u32 channel;
	u32 datarate;
	u32 antenna;
	u32 priority;
	u32 ssi_type;
	s32 ssi_signal;
	s32 ssi_noise;
	u32 preamble;
	u32 encoding;
} p80211_caphdr_t;

/* buffer free method pointer type */
typedef void (*freebuf_method_t) (void *buf, int size);

typedef struct p80211_metawep {
	void *data;
	u8 iv[4];
	u8 icv[4];
} p80211_metawep_t;

/* local ether header type */
typedef struct wlan_ethhdr {
	u8 daddr[WLAN_ETHADDR_LEN];
	u8 saddr[WLAN_ETHADDR_LEN];
	u16 type;
} __attribute__ ((packed)) wlan_ethhdr_t;

/* local llc header type */
typedef struct wlan_llc {
	u8 dsap;
	u8 ssap;
	u8 ctl;
} __attribute__ ((packed)) wlan_llc_t;

/* local snap header type */
typedef struct wlan_snap {
	u8 oui[WLAN_IEEE_OUI_LEN];
	u16 type;
} __attribute__ ((packed)) wlan_snap_t;

/* Circular include trick */
struct wlandevice;

int skb_p80211_to_ether(struct wlandevice *wlandev, u32 ethconv,
			struct sk_buff *skb);
int skb_ether_to_p80211(struct wlandevice *wlandev, u32 ethconv,
			struct sk_buff *skb, p80211_hdr_t *p80211_hdr,
			p80211_metawep_t *p80211_wep);

int p80211_stt_findproto(u16 proto);

#endif
