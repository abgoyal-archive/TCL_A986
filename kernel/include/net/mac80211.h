

#ifndef MAC80211_H
#define MAC80211_H

#include <linux/kernel.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/wireless.h>
#include <linux/device.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>






enum ieee80211_max_queues {
	IEEE80211_MAX_QUEUES =		4,
};

struct ieee80211_tx_queue_params {
	u16 txop;
	u16 cw_min;
	u16 cw_max;
	u8 aifs;
	bool uapsd;
};

struct ieee80211_low_level_stats {
	unsigned int dot11ACKFailureCount;
	unsigned int dot11RTSFailureCount;
	unsigned int dot11FCSErrorCount;
	unsigned int dot11RTSSuccessCount;
};

enum ieee80211_bss_change {
	BSS_CHANGED_ASSOC		= 1<<0,
	BSS_CHANGED_ERP_CTS_PROT	= 1<<1,
	BSS_CHANGED_ERP_PREAMBLE	= 1<<2,
	BSS_CHANGED_ERP_SLOT		= 1<<3,
	BSS_CHANGED_HT                  = 1<<4,
	BSS_CHANGED_BASIC_RATES		= 1<<5,
	BSS_CHANGED_BEACON_INT		= 1<<6,
	BSS_CHANGED_BSSID		= 1<<7,
	BSS_CHANGED_BEACON		= 1<<8,
	BSS_CHANGED_BEACON_ENABLED	= 1<<9,
	BSS_CHANGED_CQM			= 1<<10,
	BSS_CHANGED_IBSS		= 1<<11,

	/* when adding here, make sure to change ieee80211_reconfig */
};

struct ieee80211_bss_conf {
	const u8 *bssid;
	/* association related data */
	bool assoc, ibss_joined;
	u16 aid;
	/* erp related data */
	bool use_cts_prot;
	bool use_short_preamble;
	bool use_short_slot;
	bool enable_beacon;
	u8 dtim_period;
	u16 beacon_int;
	u16 assoc_capability;
	u64 timestamp;
	u32 basic_rates;
	u16 ht_operation_mode;
	s32 cqm_rssi_thold;
	u32 cqm_rssi_hyst;
	enum nl80211_channel_type channel_type;
};

enum mac80211_tx_control_flags {
	IEEE80211_TX_CTL_REQ_TX_STATUS		= BIT(0),
	IEEE80211_TX_CTL_ASSIGN_SEQ		= BIT(1),
	IEEE80211_TX_CTL_NO_ACK			= BIT(2),
	IEEE80211_TX_CTL_CLEAR_PS_FILT		= BIT(3),
	IEEE80211_TX_CTL_FIRST_FRAGMENT		= BIT(4),
	IEEE80211_TX_CTL_SEND_AFTER_DTIM	= BIT(5),
	IEEE80211_TX_CTL_AMPDU			= BIT(6),
	IEEE80211_TX_CTL_INJECTED		= BIT(7),
	IEEE80211_TX_STAT_TX_FILTERED		= BIT(8),
	IEEE80211_TX_STAT_ACK			= BIT(9),
	IEEE80211_TX_STAT_AMPDU			= BIT(10),
	IEEE80211_TX_STAT_AMPDU_NO_BACK		= BIT(11),
	IEEE80211_TX_CTL_RATE_CTRL_PROBE	= BIT(12),
	IEEE80211_TX_INTFL_NEED_TXPROCESSING	= BIT(14),
	IEEE80211_TX_INTFL_RETRIED		= BIT(15),
	IEEE80211_TX_INTFL_DONT_ENCRYPT		= BIT(16),
	IEEE80211_TX_CTL_PSPOLL_RESPONSE	= BIT(17),
	IEEE80211_TX_CTL_MORE_FRAMES		= BIT(18),
	IEEE80211_TX_INTFL_RETRANSMISSION	= BIT(19),
	IEEE80211_TX_INTFL_HAS_RADIOTAP		= BIT(20),
	IEEE80211_TX_INTFL_NL80211_FRAME_TX	= BIT(21),
	IEEE80211_TX_CTL_LDPC			= BIT(22),
	IEEE80211_TX_CTL_STBC			= BIT(23) | BIT(24),
#define IEEE80211_TX_CTL_STBC_SHIFT		23
};

enum mac80211_rate_control_flags {
	IEEE80211_TX_RC_USE_RTS_CTS		= BIT(0),
	IEEE80211_TX_RC_USE_CTS_PROTECT		= BIT(1),
	IEEE80211_TX_RC_USE_SHORT_PREAMBLE	= BIT(2),

	/* rate index is an MCS rate number instead of an index */
	IEEE80211_TX_RC_MCS			= BIT(3),
	IEEE80211_TX_RC_GREEN_FIELD		= BIT(4),
	IEEE80211_TX_RC_40_MHZ_WIDTH		= BIT(5),
	IEEE80211_TX_RC_DUP_DATA		= BIT(6),
	IEEE80211_TX_RC_SHORT_GI		= BIT(7),
};


/* there are 40 bytes if you don't need the rateset to be kept */
#define IEEE80211_TX_INFO_DRIVER_DATA_SIZE 40

/* if you do need the rateset, then you have less space */
#define IEEE80211_TX_INFO_RATE_DRIVER_DATA_SIZE 24

/* maximum number of rate stages */
#define IEEE80211_TX_MAX_RATES	5

struct ieee80211_tx_rate {
	s8 idx;
	u8 count;
	u8 flags;
} __attribute__((packed));

struct ieee80211_tx_info {
	/* common information */
	u32 flags;
	u8 band;

	u8 antenna_sel_tx;

	/* 2 byte hole */
	u8 pad[2];

	union {
		struct {
			union {
				/* rate control */
				struct {
					struct ieee80211_tx_rate rates[
						IEEE80211_TX_MAX_RATES];
					s8 rts_cts_rate_idx;
				};
				/* only needed before rate control */
				unsigned long jiffies;
			};
			/* NB: vif can be NULL for injected frames */
			struct ieee80211_vif *vif;
			struct ieee80211_key_conf *hw_key;
			struct ieee80211_sta *sta;
		} control;
		struct {
			struct ieee80211_tx_rate rates[IEEE80211_TX_MAX_RATES];
			u8 ampdu_ack_len;
			u64 ampdu_ack_map;
			int ack_signal;
			u8 ampdu_len;
			/* 7 bytes free */
		} status;
		struct {
			struct ieee80211_tx_rate driver_rates[
				IEEE80211_TX_MAX_RATES];
			void *rate_driver_data[
				IEEE80211_TX_INFO_RATE_DRIVER_DATA_SIZE / sizeof(void *)];
		};
		void *driver_data[
			IEEE80211_TX_INFO_DRIVER_DATA_SIZE / sizeof(void *)];
	};
};

static inline struct ieee80211_tx_info *IEEE80211_SKB_CB(struct sk_buff *skb)
{
	return (struct ieee80211_tx_info *)skb->cb;
}

static inline struct ieee80211_rx_status *IEEE80211_SKB_RXCB(struct sk_buff *skb)
{
	return (struct ieee80211_rx_status *)skb->cb;
}

static inline void
ieee80211_tx_info_clear_status(struct ieee80211_tx_info *info)
{
	int i;

	BUILD_BUG_ON(offsetof(struct ieee80211_tx_info, status.rates) !=
		     offsetof(struct ieee80211_tx_info, control.rates));
	BUILD_BUG_ON(offsetof(struct ieee80211_tx_info, status.rates) !=
		     offsetof(struct ieee80211_tx_info, driver_rates));
	BUILD_BUG_ON(offsetof(struct ieee80211_tx_info, status.rates) != 8);
	/* clear the rate counts */
	for (i = 0; i < IEEE80211_TX_MAX_RATES; i++)
		info->status.rates[i].count = 0;

	BUILD_BUG_ON(
	    offsetof(struct ieee80211_tx_info, status.ampdu_ack_len) != 23);
	memset(&info->status.ampdu_ack_len, 0,
	       sizeof(struct ieee80211_tx_info) -
	       offsetof(struct ieee80211_tx_info, status.ampdu_ack_len));
}


enum mac80211_rx_flags {
	RX_FLAG_MMIC_ERROR	= 1<<0,
	RX_FLAG_DECRYPTED	= 1<<1,
	RX_FLAG_MMIC_STRIPPED	= 1<<3,
	RX_FLAG_IV_STRIPPED	= 1<<4,
	RX_FLAG_FAILED_FCS_CRC	= 1<<5,
	RX_FLAG_FAILED_PLCP_CRC = 1<<6,
	RX_FLAG_TSFT		= 1<<7,
	RX_FLAG_SHORTPRE	= 1<<8,
	RX_FLAG_HT		= 1<<9,
	RX_FLAG_40MHZ		= 1<<10,
	RX_FLAG_SHORT_GI	= 1<<11,
	RX_FLAG_INTERNAL_CMTR	= 1<<12,
};

struct ieee80211_rx_status {
	u64 mactime;
	enum ieee80211_band band;
	int freq;
	int signal;
	int antenna;
	int rate_idx;
	int flag;
};

enum ieee80211_conf_flags {
	IEEE80211_CONF_MONITOR		= (1<<0),
	IEEE80211_CONF_PS		= (1<<1),
	IEEE80211_CONF_IDLE		= (1<<2),
	IEEE80211_CONF_QOS		= (1<<3),
};


enum ieee80211_conf_changed {
	IEEE80211_CONF_CHANGE_SMPS		= BIT(1),
	IEEE80211_CONF_CHANGE_LISTEN_INTERVAL	= BIT(2),
	IEEE80211_CONF_CHANGE_MONITOR		= BIT(3),
	IEEE80211_CONF_CHANGE_PS		= BIT(4),
	IEEE80211_CONF_CHANGE_POWER		= BIT(5),
	IEEE80211_CONF_CHANGE_CHANNEL		= BIT(6),
	IEEE80211_CONF_CHANGE_RETRY_LIMITS	= BIT(7),
	IEEE80211_CONF_CHANGE_IDLE		= BIT(8),
	IEEE80211_CONF_CHANGE_QOS		= BIT(9),
};

enum ieee80211_smps_mode {
	IEEE80211_SMPS_AUTOMATIC,
	IEEE80211_SMPS_OFF,
	IEEE80211_SMPS_STATIC,
	IEEE80211_SMPS_DYNAMIC,

	/* keep last */
	IEEE80211_SMPS_NUM_MODES,
};

struct ieee80211_conf {
	u32 flags;
	int power_level, dynamic_ps_timeout, dynamic_ps_forced_timeout;
	int max_sleep_period;

	u16 listen_interval;
	u8 ps_dtim_period;

	u8 long_frame_max_tx_count, short_frame_max_tx_count;

	struct ieee80211_channel *channel;
	enum nl80211_channel_type channel_type;
	enum ieee80211_smps_mode smps_mode;
};

struct ieee80211_channel_switch {
	u64 timestamp;
	bool block_tx;
	struct ieee80211_channel *channel;
	u8 count;
};

struct ieee80211_vif {
	enum nl80211_iftype type;
	struct ieee80211_bss_conf bss_conf;
	u8 addr[ETH_ALEN];
	/* must be last */
	u8 drv_priv[0] __attribute__((__aligned__(sizeof(void *))));
};

static inline bool ieee80211_vif_is_mesh(struct ieee80211_vif *vif)
{
#ifdef CONFIG_MAC80211_MESH
	return vif->type == NL80211_IFTYPE_MESH_POINT;
#endif
	return false;
}

enum ieee80211_key_alg {
	ALG_WEP,
	ALG_TKIP,
	ALG_CCMP,
	ALG_AES_CMAC,
};

enum ieee80211_key_flags {
	IEEE80211_KEY_FLAG_WMM_STA	= 1<<0,
	IEEE80211_KEY_FLAG_GENERATE_IV	= 1<<1,
	IEEE80211_KEY_FLAG_GENERATE_MMIC= 1<<2,
	IEEE80211_KEY_FLAG_PAIRWISE	= 1<<3,
	IEEE80211_KEY_FLAG_SW_MGMT	= 1<<4,
};

struct ieee80211_key_conf {
	enum ieee80211_key_alg alg;
	u8 icv_len;
	u8 iv_len;
	u8 hw_key_idx;
	u8 flags;
	s8 keyidx;
	u8 keylen;
	u8 key[0];
};

enum set_key_cmd {
	SET_KEY, DISABLE_KEY,
};

struct ieee80211_sta {
	u32 supp_rates[IEEE80211_NUM_BANDS];
	u8 addr[ETH_ALEN];
	u16 aid;
	struct ieee80211_sta_ht_cap ht_cap;

	/* must be last */
	u8 drv_priv[0] __attribute__((__aligned__(sizeof(void *))));
};

enum sta_notify_cmd {
	STA_NOTIFY_ADD, STA_NOTIFY_REMOVE,
	STA_NOTIFY_SLEEP, STA_NOTIFY_AWAKE,
};

enum ieee80211_tkip_key_type {
	IEEE80211_TKIP_P1_KEY,
	IEEE80211_TKIP_P2_KEY,
};

enum ieee80211_hw_flags {
	IEEE80211_HW_HAS_RATE_CONTROL			= 1<<0,
	IEEE80211_HW_RX_INCLUDES_FCS			= 1<<1,
	IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING	= 1<<2,
	IEEE80211_HW_2GHZ_SHORT_SLOT_INCAPABLE		= 1<<3,
	IEEE80211_HW_2GHZ_SHORT_PREAMBLE_INCAPABLE	= 1<<4,
	IEEE80211_HW_SIGNAL_UNSPEC			= 1<<5,
	IEEE80211_HW_SIGNAL_DBM				= 1<<6,
	/* use this hole */
	IEEE80211_HW_SPECTRUM_MGMT			= 1<<8,
	IEEE80211_HW_AMPDU_AGGREGATION			= 1<<9,
	IEEE80211_HW_SUPPORTS_PS			= 1<<10,
	IEEE80211_HW_PS_NULLFUNC_STACK			= 1<<11,
	IEEE80211_HW_SUPPORTS_DYNAMIC_PS		= 1<<12,
	IEEE80211_HW_MFP_CAPABLE			= 1<<13,
	IEEE80211_HW_BEACON_FILTER			= 1<<14,
	IEEE80211_HW_SUPPORTS_STATIC_SMPS		= 1<<15,
	IEEE80211_HW_SUPPORTS_DYNAMIC_SMPS		= 1<<16,
	IEEE80211_HW_SUPPORTS_UAPSD			= 1<<17,
	IEEE80211_HW_REPORTS_TX_ACK_STATUS		= 1<<18,
	IEEE80211_HW_CONNECTION_MONITOR			= 1<<19,
	IEEE80211_HW_SUPPORTS_CQM_RSSI			= 1<<20,
};

struct ieee80211_hw {
	struct ieee80211_conf conf;
	struct wiphy *wiphy;
	const char *rate_control_algorithm;
	void *priv;
	u32 flags;
	unsigned int extra_tx_headroom;
	int channel_change_time;
	int vif_data_size;
	int sta_data_size;
	u16 queues;
	u16 max_listen_interval;
	s8 max_signal;
	u8 max_rates;
	u8 max_rate_tries;
};

struct ieee80211_hw *wiphy_to_ieee80211_hw(struct wiphy *wiphy);

static inline void SET_IEEE80211_DEV(struct ieee80211_hw *hw, struct device *dev)
{
	set_wiphy_dev(hw->wiphy, dev);
}

static inline void SET_IEEE80211_PERM_ADDR(struct ieee80211_hw *hw, u8 *addr)
{
	memcpy(hw->wiphy->perm_addr, addr, ETH_ALEN);
}

static inline struct ieee80211_rate *
ieee80211_get_tx_rate(const struct ieee80211_hw *hw,
		      const struct ieee80211_tx_info *c)
{
	if (WARN_ON(c->control.rates[0].idx < 0))
		return NULL;
	return &hw->wiphy->bands[c->band]->bitrates[c->control.rates[0].idx];
}

static inline struct ieee80211_rate *
ieee80211_get_rts_cts_rate(const struct ieee80211_hw *hw,
			   const struct ieee80211_tx_info *c)
{
	if (c->control.rts_cts_rate_idx < 0)
		return NULL;
	return &hw->wiphy->bands[c->band]->bitrates[c->control.rts_cts_rate_idx];
}

static inline struct ieee80211_rate *
ieee80211_get_alt_retry_rate(const struct ieee80211_hw *hw,
			     const struct ieee80211_tx_info *c, int idx)
{
	if (c->control.rates[idx + 1].idx < 0)
		return NULL;
	return &hw->wiphy->bands[c->band]->bitrates[c->control.rates[idx + 1].idx];
}






enum ieee80211_filter_flags {
	FIF_PROMISC_IN_BSS	= 1<<0,
	FIF_ALLMULTI		= 1<<1,
	FIF_FCSFAIL		= 1<<2,
	FIF_PLCPFAIL		= 1<<3,
	FIF_BCN_PRBRESP_PROMISC	= 1<<4,
	FIF_CONTROL		= 1<<5,
	FIF_OTHER_BSS		= 1<<6,
	FIF_PSPOLL		= 1<<7,
};

enum ieee80211_ampdu_mlme_action {
	IEEE80211_AMPDU_RX_START,
	IEEE80211_AMPDU_RX_STOP,
	IEEE80211_AMPDU_TX_START,
	IEEE80211_AMPDU_TX_STOP,
	IEEE80211_AMPDU_TX_OPERATIONAL,
};

struct ieee80211_ops {
	int (*tx)(struct ieee80211_hw *hw, struct sk_buff *skb);
	int (*start)(struct ieee80211_hw *hw);
	void (*stop)(struct ieee80211_hw *hw);
	int (*add_interface)(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif);
	void (*remove_interface)(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif);
	int (*config)(struct ieee80211_hw *hw, u32 changed);
	void (*bss_info_changed)(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_bss_conf *info,
				 u32 changed);
	u64 (*prepare_multicast)(struct ieee80211_hw *hw,
				 struct netdev_hw_addr_list *mc_list);
	void (*configure_filter)(struct ieee80211_hw *hw,
				 unsigned int changed_flags,
				 unsigned int *total_flags,
				 u64 multicast);
	int (*set_tim)(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
		       bool set);
	int (*set_key)(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		       struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		       struct ieee80211_key_conf *key);
	void (*update_tkip_key)(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_key_conf *conf,
				struct ieee80211_sta *sta,
				u32 iv32, u16 *phase1key);
	int (*hw_scan)(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		       struct cfg80211_scan_request *req);
	void (*sw_scan_start)(struct ieee80211_hw *hw);
	void (*sw_scan_complete)(struct ieee80211_hw *hw);
	int (*get_stats)(struct ieee80211_hw *hw,
			 struct ieee80211_low_level_stats *stats);
	void (*get_tkip_seq)(struct ieee80211_hw *hw, u8 hw_key_idx,
			     u32 *iv32, u16 *iv16);
	int (*set_rts_threshold)(struct ieee80211_hw *hw, u32 value);
	int (*sta_add)(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta);
	int (*sta_remove)(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta);
	void (*sta_notify)(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			enum sta_notify_cmd, struct ieee80211_sta *sta);
	int (*conf_tx)(struct ieee80211_hw *hw, u16 queue,
		       const struct ieee80211_tx_queue_params *params);
	u64 (*get_tsf)(struct ieee80211_hw *hw);
	void (*set_tsf)(struct ieee80211_hw *hw, u64 tsf);
	void (*reset_tsf)(struct ieee80211_hw *hw);
	int (*tx_last_beacon)(struct ieee80211_hw *hw);
	int (*ampdu_action)(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif,
			    enum ieee80211_ampdu_mlme_action action,
			    struct ieee80211_sta *sta, u16 tid, u16 *ssn);
	int (*get_survey)(struct ieee80211_hw *hw, int idx,
		struct survey_info *survey);
	void (*rfkill_poll)(struct ieee80211_hw *hw);
	void (*set_coverage_class)(struct ieee80211_hw *hw, u8 coverage_class);
#ifdef CONFIG_NL80211_TESTMODE
	int (*testmode_cmd)(struct ieee80211_hw *hw, void *data, int len);
#endif
	void (*flush)(struct ieee80211_hw *hw, bool drop);
	void (*channel_switch)(struct ieee80211_hw *hw,
			       struct ieee80211_channel_switch *ch_switch);
};

struct ieee80211_hw *ieee80211_alloc_hw(size_t priv_data_len,
					const struct ieee80211_ops *ops);

int ieee80211_register_hw(struct ieee80211_hw *hw);

#ifdef CONFIG_MAC80211_LEDS
extern char *__ieee80211_get_tx_led_name(struct ieee80211_hw *hw);
extern char *__ieee80211_get_rx_led_name(struct ieee80211_hw *hw);
extern char *__ieee80211_get_assoc_led_name(struct ieee80211_hw *hw);
extern char *__ieee80211_get_radio_led_name(struct ieee80211_hw *hw);
#endif
static inline char *ieee80211_get_tx_led_name(struct ieee80211_hw *hw)
{
#ifdef CONFIG_MAC80211_LEDS
	return __ieee80211_get_tx_led_name(hw);
#else
	return NULL;
#endif
}

static inline char *ieee80211_get_rx_led_name(struct ieee80211_hw *hw)
{
#ifdef CONFIG_MAC80211_LEDS
	return __ieee80211_get_rx_led_name(hw);
#else
	return NULL;
#endif
}

static inline char *ieee80211_get_assoc_led_name(struct ieee80211_hw *hw)
{
#ifdef CONFIG_MAC80211_LEDS
	return __ieee80211_get_assoc_led_name(hw);
#else
	return NULL;
#endif
}

static inline char *ieee80211_get_radio_led_name(struct ieee80211_hw *hw)
{
#ifdef CONFIG_MAC80211_LEDS
	return __ieee80211_get_radio_led_name(hw);
#else
	return NULL;
#endif
}

void ieee80211_unregister_hw(struct ieee80211_hw *hw);

void ieee80211_free_hw(struct ieee80211_hw *hw);

void ieee80211_restart_hw(struct ieee80211_hw *hw);

void ieee80211_rx(struct ieee80211_hw *hw, struct sk_buff *skb);

void ieee80211_rx_irqsafe(struct ieee80211_hw *hw, struct sk_buff *skb);

static inline void ieee80211_rx_ni(struct ieee80211_hw *hw,
				   struct sk_buff *skb)
{
	local_bh_disable();
	ieee80211_rx(hw, skb);
	local_bh_enable();
}

#define IEEE80211_TX_STATUS_HEADROOM	13

void ieee80211_tx_status(struct ieee80211_hw *hw,
			 struct sk_buff *skb);

void ieee80211_tx_status_irqsafe(struct ieee80211_hw *hw,
				 struct sk_buff *skb);

struct sk_buff *ieee80211_beacon_get_tim(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif,
					 u16 *tim_offset, u16 *tim_length);

static inline struct sk_buff *ieee80211_beacon_get(struct ieee80211_hw *hw,
						   struct ieee80211_vif *vif)
{
	return ieee80211_beacon_get_tim(hw, vif, NULL, NULL);
}

struct sk_buff *ieee80211_pspoll_get(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif);

struct sk_buff *ieee80211_nullfunc_get(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif);

struct sk_buff *ieee80211_probereq_get(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif,
				       const u8 *ssid, size_t ssid_len,
				       const u8 *ie, size_t ie_len);

void ieee80211_rts_get(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		       const void *frame, size_t frame_len,
		       const struct ieee80211_tx_info *frame_txctl,
		       struct ieee80211_rts *rts);

__le16 ieee80211_rts_duration(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif, size_t frame_len,
			      const struct ieee80211_tx_info *frame_txctl);

void ieee80211_ctstoself_get(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     const void *frame, size_t frame_len,
			     const struct ieee80211_tx_info *frame_txctl,
			     struct ieee80211_cts *cts);

__le16 ieee80211_ctstoself_duration(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    size_t frame_len,
				    const struct ieee80211_tx_info *frame_txctl);

__le16 ieee80211_generic_frame_duration(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif,
					size_t frame_len,
					struct ieee80211_rate *rate);

struct sk_buff *
ieee80211_get_buffered_bc(struct ieee80211_hw *hw, struct ieee80211_vif *vif);

void ieee80211_get_tkip_key(struct ieee80211_key_conf *keyconf,
				struct sk_buff *skb,
				enum ieee80211_tkip_key_type type, u8 *key);
void ieee80211_wake_queue(struct ieee80211_hw *hw, int queue);

void ieee80211_stop_queue(struct ieee80211_hw *hw, int queue);


int ieee80211_queue_stopped(struct ieee80211_hw *hw, int queue);

void ieee80211_stop_queues(struct ieee80211_hw *hw);

void ieee80211_wake_queues(struct ieee80211_hw *hw);

void ieee80211_scan_completed(struct ieee80211_hw *hw, bool aborted);

void ieee80211_iterate_active_interfaces(struct ieee80211_hw *hw,
					 void (*iterator)(void *data, u8 *mac,
						struct ieee80211_vif *vif),
					 void *data);

void ieee80211_iterate_active_interfaces_atomic(struct ieee80211_hw *hw,
						void (*iterator)(void *data,
						    u8 *mac,
						    struct ieee80211_vif *vif),
						void *data);

void ieee80211_queue_work(struct ieee80211_hw *hw, struct work_struct *work);

void ieee80211_queue_delayed_work(struct ieee80211_hw *hw,
				  struct delayed_work *dwork,
				  unsigned long delay);

int ieee80211_start_tx_ba_session(struct ieee80211_sta *sta, u16 tid);

void ieee80211_start_tx_ba_cb(struct ieee80211_vif *vif, u8 *ra, u16 tid);

void ieee80211_start_tx_ba_cb_irqsafe(struct ieee80211_vif *vif, const u8 *ra,
				      u16 tid);

int ieee80211_stop_tx_ba_session(struct ieee80211_sta *sta, u16 tid,
				 enum ieee80211_back_parties initiator);

void ieee80211_stop_tx_ba_cb(struct ieee80211_vif *vif, u8 *ra, u8 tid);

void ieee80211_stop_tx_ba_cb_irqsafe(struct ieee80211_vif *vif, const u8 *ra,
				     u16 tid);

struct ieee80211_sta *ieee80211_find_sta(struct ieee80211_vif *vif,
					 const u8 *addr);

struct ieee80211_sta *ieee80211_find_sta_by_hw(struct ieee80211_hw *hw,
					       const u8 *addr);

void ieee80211_sta_block_awake(struct ieee80211_hw *hw,
			       struct ieee80211_sta *pubsta, bool block);

void ieee80211_beacon_loss(struct ieee80211_vif *vif);

void ieee80211_connection_loss(struct ieee80211_vif *vif);

void ieee80211_cqm_rssi_notify(struct ieee80211_vif *vif,
			       enum nl80211_cqm_rssi_threshold_event rssi_event,
			       gfp_t gfp);

void ieee80211_chswitch_done(struct ieee80211_vif *vif, bool success);

/* Rate control API */

enum rate_control_changed {
	IEEE80211_RC_HT_CHANGED = BIT(0)
};

struct ieee80211_tx_rate_control {
	struct ieee80211_hw *hw;
	struct ieee80211_supported_band *sband;
	struct ieee80211_bss_conf *bss_conf;
	struct sk_buff *skb;
	struct ieee80211_tx_rate reported_rate;
	bool rts, short_preamble;
	u8 max_rate_idx;
	u32 rate_idx_mask;
	bool ap;
};

struct rate_control_ops {
	struct module *module;
	const char *name;
	void *(*alloc)(struct ieee80211_hw *hw, struct dentry *debugfsdir);
	void (*free)(void *priv);

	void *(*alloc_sta)(void *priv, struct ieee80211_sta *sta, gfp_t gfp);
	void (*rate_init)(void *priv, struct ieee80211_supported_band *sband,
			  struct ieee80211_sta *sta, void *priv_sta);
	void (*rate_update)(void *priv, struct ieee80211_supported_band *sband,
			    struct ieee80211_sta *sta,
			    void *priv_sta, u32 changed,
			    enum nl80211_channel_type oper_chan_type);
	void (*free_sta)(void *priv, struct ieee80211_sta *sta,
			 void *priv_sta);

	void (*tx_status)(void *priv, struct ieee80211_supported_band *sband,
			  struct ieee80211_sta *sta, void *priv_sta,
			  struct sk_buff *skb);
	void (*get_rate)(void *priv, struct ieee80211_sta *sta, void *priv_sta,
			 struct ieee80211_tx_rate_control *txrc);

	void (*add_sta_debugfs)(void *priv, void *priv_sta,
				struct dentry *dir);
	void (*remove_sta_debugfs)(void *priv, void *priv_sta);
};

static inline int rate_supported(struct ieee80211_sta *sta,
				 enum ieee80211_band band,
				 int index)
{
	return (sta == NULL || sta->supp_rates[band] & BIT(index));
}

bool rate_control_send_low(struct ieee80211_sta *sta,
			   void *priv_sta,
			   struct ieee80211_tx_rate_control *txrc);


static inline s8
rate_lowest_index(struct ieee80211_supported_band *sband,
		  struct ieee80211_sta *sta)
{
	int i;

	for (i = 0; i < sband->n_bitrates; i++)
		if (rate_supported(sta, sband->band, i))
			return i;

	/* warn when we cannot find a rate. */
	WARN_ON(1);

	return 0;
}

static inline
bool rate_usable_index_exists(struct ieee80211_supported_band *sband,
			      struct ieee80211_sta *sta)
{
	unsigned int i;

	for (i = 0; i < sband->n_bitrates; i++)
		if (rate_supported(sta, sband->band, i))
			return true;
	return false;
}

int ieee80211_rate_control_register(struct rate_control_ops *ops);
void ieee80211_rate_control_unregister(struct rate_control_ops *ops);

static inline bool
conf_is_ht20(struct ieee80211_conf *conf)
{
	return conf->channel_type == NL80211_CHAN_HT20;
}

static inline bool
conf_is_ht40_minus(struct ieee80211_conf *conf)
{
	return conf->channel_type == NL80211_CHAN_HT40MINUS;
}

static inline bool
conf_is_ht40_plus(struct ieee80211_conf *conf)
{
	return conf->channel_type == NL80211_CHAN_HT40PLUS;
}

static inline bool
conf_is_ht40(struct ieee80211_conf *conf)
{
	return conf_is_ht40_minus(conf) || conf_is_ht40_plus(conf);
}

static inline bool
conf_is_ht(struct ieee80211_conf *conf)
{
	return conf->channel_type != NL80211_CHAN_NO_HT;
}

#endif /* MAC80211_H */
