

#ifndef STA_INFO_H
#define STA_INFO_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/if_ether.h>
#include <linux/workqueue.h>
#include "key.h"

enum ieee80211_sta_info_flags {
	WLAN_STA_AUTH		= 1<<0,
	WLAN_STA_ASSOC		= 1<<1,
	WLAN_STA_PS_STA		= 1<<2,
	WLAN_STA_AUTHORIZED	= 1<<3,
	WLAN_STA_SHORT_PREAMBLE	= 1<<4,
	WLAN_STA_ASSOC_AP	= 1<<5,
	WLAN_STA_WME		= 1<<6,
	WLAN_STA_WDS		= 1<<7,
	WLAN_STA_CLEAR_PS_FILT	= 1<<9,
	WLAN_STA_MFP		= 1<<10,
	WLAN_STA_BLOCK_BA	= 1<<11,
	WLAN_STA_PS_DRIVER	= 1<<12,
	WLAN_STA_PSPOLL		= 1<<13,
	WLAN_STA_DISASSOC       = 1<<14,
};

#define STA_TID_NUM 16
#define ADDBA_RESP_INTERVAL HZ
#define HT_AGG_MAX_RETRIES		(0x3)

#define HT_AGG_STATE_INITIATOR_SHIFT	(4)

#define HT_ADDBA_REQUESTED_MSK		BIT(0)
#define HT_ADDBA_DRV_READY_MSK		BIT(1)
#define HT_ADDBA_RECEIVED_MSK		BIT(2)
#define HT_AGG_STATE_REQ_STOP_BA_MSK	BIT(3)
#define HT_AGG_STATE_INITIATOR_MSK      BIT(HT_AGG_STATE_INITIATOR_SHIFT)
#define HT_AGG_STATE_IDLE		(0x0)
#define HT_AGG_STATE_OPERATIONAL	(HT_ADDBA_REQUESTED_MSK |	\
					 HT_ADDBA_DRV_READY_MSK |	\
					 HT_ADDBA_RECEIVED_MSK)

struct tid_ampdu_tx {
	struct timer_list addba_resp_timer;
	struct sk_buff_head pending;
	u16 ssn;
	u8 dialog_token;
};

struct tid_ampdu_rx {
	struct sk_buff **reorder_buf;
	unsigned long *reorder_time;
	struct timer_list session_timer;
	u16 head_seq_num;
	u16 stored_mpdu_num;
	u16 ssn;
	u16 buf_size;
	u16 timeout;
	u8 dialog_token;
};

enum plink_state {
	PLINK_LISTEN,
	PLINK_OPN_SNT,
	PLINK_OPN_RCVD,
	PLINK_CNF_RCVD,
	PLINK_ESTAB,
	PLINK_HOLDING,
	PLINK_BLOCKED
};

struct sta_ampdu_mlme {
	/* rx */
	bool tid_active_rx[STA_TID_NUM];
	struct tid_ampdu_rx *tid_rx[STA_TID_NUM];
	/* tx */
	u8 tid_state_tx[STA_TID_NUM];
	struct tid_ampdu_tx *tid_tx[STA_TID_NUM];
	u8 addba_req_num[STA_TID_NUM];
	u8 dialog_token_allocator;
};


struct sta_info {
	/* General information, mostly static */
	struct list_head list;
	struct sta_info *hnext;
	struct ieee80211_local *local;
	struct ieee80211_sub_if_data *sdata;
	struct ieee80211_key *key;
	struct rate_control_ref *rate_ctrl;
	void *rate_ctrl_priv;
	spinlock_t lock;
	spinlock_t flaglock;

	struct work_struct drv_unblock_wk;

	u16 listen_interval;

	bool dead;

	bool uploaded;

	/*
	 * frequently updated, locked with own spinlock (flaglock),
	 * use the accessors defined below
	 */
	u32 flags;

	/*
	 * STA powersave frame queues, no more than the internal
	 * locking required.
	 */
	struct sk_buff_head ps_tx_buf;
	struct sk_buff_head tx_filtered;

	/* Updated from RX path only, no locking requirements */
	unsigned long rx_packets, rx_bytes;
	unsigned long wep_weak_iv_count;
	unsigned long last_rx;
	unsigned long num_duplicates;
	unsigned long rx_fragments;
	unsigned long rx_dropped;
	int last_signal;
	__le16 last_seq_ctrl[NUM_RX_DATA_QUEUES];

	/* Updated from TX status path only, no locking requirements */
	unsigned long tx_filtered_count;
	unsigned long tx_retry_failed, tx_retry_count;
	/* moving percentage of failed MSDUs */
	unsigned int fail_avg;

	/* Updated from TX path only, no locking requirements */
	unsigned long tx_packets;
	unsigned long tx_bytes;
	unsigned long tx_fragments;
	struct ieee80211_tx_rate last_tx_rate;
	u16 tid_seq[IEEE80211_QOS_CTL_TID_MASK + 1];

	/*
	 * Aggregation information, locked with lock.
	 */
	struct sta_ampdu_mlme ampdu_mlme;
	u8 timer_to_tid[STA_TID_NUM];

#ifdef CONFIG_MAC80211_MESH
	/*
	 * Mesh peer link attributes
	 * TODO: move to a sub-structure that is referenced with pointer?
	 */
	__le16 llid;
	__le16 plid;
	__le16 reason;
	u8 plink_retries;
	bool ignore_plink_timer;
	bool plink_timer_was_running;
	enum plink_state plink_state;
	u32 plink_timeout;
	struct timer_list plink_timer;
#endif

#ifdef CONFIG_MAC80211_DEBUGFS
	struct sta_info_debugfsdentries {
		struct dentry *dir;
		bool add_has_run;
	} debugfs;
#endif

	/* keep last! */
	struct ieee80211_sta sta;
};

static inline enum plink_state sta_plink_state(struct sta_info *sta)
{
#ifdef CONFIG_MAC80211_MESH
	return sta->plink_state;
#endif
	return PLINK_LISTEN;
}

static inline void set_sta_flags(struct sta_info *sta, const u32 flags)
{
	unsigned long irqfl;

	spin_lock_irqsave(&sta->flaglock, irqfl);
	sta->flags |= flags;
	spin_unlock_irqrestore(&sta->flaglock, irqfl);
}

static inline void clear_sta_flags(struct sta_info *sta, const u32 flags)
{
	unsigned long irqfl;

	spin_lock_irqsave(&sta->flaglock, irqfl);
	sta->flags &= ~flags;
	spin_unlock_irqrestore(&sta->flaglock, irqfl);
}

static inline u32 test_sta_flags(struct sta_info *sta, const u32 flags)
{
	u32 ret;
	unsigned long irqfl;

	spin_lock_irqsave(&sta->flaglock, irqfl);
	ret = sta->flags & flags;
	spin_unlock_irqrestore(&sta->flaglock, irqfl);

	return ret;
}

static inline u32 test_and_clear_sta_flags(struct sta_info *sta,
					   const u32 flags)
{
	u32 ret;
	unsigned long irqfl;

	spin_lock_irqsave(&sta->flaglock, irqfl);
	ret = sta->flags & flags;
	sta->flags &= ~flags;
	spin_unlock_irqrestore(&sta->flaglock, irqfl);

	return ret;
}

static inline u32 get_sta_flags(struct sta_info *sta)
{
	u32 ret;
	unsigned long irqfl;

	spin_lock_irqsave(&sta->flaglock, irqfl);
	ret = sta->flags;
	spin_unlock_irqrestore(&sta->flaglock, irqfl);

	return ret;
}



#define STA_HASH_SIZE 256
#define STA_HASH(sta) (sta[5])


/* Maximum number of frames to buffer per power saving station */
#define STA_MAX_TX_BUFFER 128

#define STA_TX_BUFFER_EXPIRE (10 * HZ)

#define STA_INFO_CLEANUP_INTERVAL (10 * HZ)

struct sta_info *sta_info_get(struct ieee80211_sub_if_data *sdata,
			      const u8 *addr);

struct sta_info *sta_info_get_bss(struct ieee80211_sub_if_data *sdata,
				  const u8 *addr);

static inline
void for_each_sta_info_type_check(struct ieee80211_local *local,
				  const u8 *addr,
				  struct sta_info *sta,
				  struct sta_info *nxt)
{
}

#define for_each_sta_info(local, _addr, sta, nxt) 			\
	for (	/* initialise loop */					\
		sta = rcu_dereference(local->sta_hash[STA_HASH(_addr)]),\
		nxt = sta ? rcu_dereference(sta->hnext) : NULL;		\
		/* typecheck */						\
		for_each_sta_info_type_check(local, (_addr), sta, nxt),	\
		/* continue condition */				\
		sta;							\
		/* advance loop */					\
		sta = nxt,						\
		nxt = sta ? rcu_dereference(sta->hnext) : NULL		\
	     )								\
	/* compare address and run code only if it matches */		\
	if (memcmp(sta->sta.addr, (_addr), ETH_ALEN) == 0)

struct sta_info *sta_info_get_by_idx(struct ieee80211_sub_if_data *sdata,
				     int idx);
struct sta_info *sta_info_alloc(struct ieee80211_sub_if_data *sdata,
				u8 *addr, gfp_t gfp);
int sta_info_insert(struct sta_info *sta);
int sta_info_insert_rcu(struct sta_info *sta) __acquires(RCU);
int sta_info_insert_atomic(struct sta_info *sta);

int sta_info_destroy_addr(struct ieee80211_sub_if_data *sdata,
			  const u8 *addr);
int sta_info_destroy_addr_bss(struct ieee80211_sub_if_data *sdata,
			      const u8 *addr);

void sta_info_set_tim_bit(struct sta_info *sta);
void sta_info_clear_tim_bit(struct sta_info *sta);

void sta_info_init(struct ieee80211_local *local);
int sta_info_start(struct ieee80211_local *local);
void sta_info_stop(struct ieee80211_local *local);
int sta_info_flush(struct ieee80211_local *local,
		   struct ieee80211_sub_if_data *sdata);
void ieee80211_sta_expire(struct ieee80211_sub_if_data *sdata,
			  unsigned long exp_time);

void ieee80211_sta_ps_deliver_wakeup(struct sta_info *sta);
void ieee80211_sta_ps_deliver_poll_response(struct sta_info *sta);

#endif /* STA_INFO_H */
