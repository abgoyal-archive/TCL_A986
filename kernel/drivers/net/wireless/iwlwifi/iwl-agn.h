

#ifndef __iwl_agn_h__
#define __iwl_agn_h__

#include "iwl-dev.h"

extern struct iwl_mod_params iwlagn_mod_params;
extern struct iwl_hcmd_ops iwlagn_hcmd;
extern struct iwl_hcmd_utils_ops iwlagn_hcmd_utils;

int iwl_reset_ict(struct iwl_priv *priv);
void iwl_disable_ict(struct iwl_priv *priv);
int iwl_alloc_isr_ict(struct iwl_priv *priv);
void iwl_free_isr_ict(struct iwl_priv *priv);
irqreturn_t iwl_isr_ict(int irq, void *data);
bool iwl_good_ack_health(struct iwl_priv *priv,
			 struct iwl_rx_packet *pkt);

/* tx queue */
void iwlagn_set_wr_ptrs(struct iwl_priv *priv,
		     int txq_id, u32 index);
void iwlagn_tx_queue_set_status(struct iwl_priv *priv,
			     struct iwl_tx_queue *txq,
			     int tx_fifo_id, int scd_retry);
void iwlagn_txq_update_byte_cnt_tbl(struct iwl_priv *priv,
				    struct iwl_tx_queue *txq,
				    u16 byte_cnt);
void iwlagn_txq_inval_byte_cnt_tbl(struct iwl_priv *priv,
				   struct iwl_tx_queue *txq);
int iwlagn_txq_agg_enable(struct iwl_priv *priv, int txq_id,
			  int tx_fifo, int sta_id, int tid, u16 ssn_idx);
int iwlagn_txq_agg_disable(struct iwl_priv *priv, u16 txq_id,
			   u16 ssn_idx, u8 tx_fifo);
void iwlagn_txq_set_sched(struct iwl_priv *priv, u32 mask);

/* uCode */
int iwlagn_load_ucode(struct iwl_priv *priv);
void iwlagn_rx_calib_result(struct iwl_priv *priv,
			 struct iwl_rx_mem_buffer *rxb);
void iwlagn_rx_calib_complete(struct iwl_priv *priv,
			   struct iwl_rx_mem_buffer *rxb);
void iwlagn_init_alive_start(struct iwl_priv *priv);
int iwlagn_alive_notify(struct iwl_priv *priv);

/* lib */
void iwl_check_abort_status(struct iwl_priv *priv,
			    u8 frame_count, u32 status);
void iwlagn_rx_handler_setup(struct iwl_priv *priv);
void iwlagn_setup_deferred_work(struct iwl_priv *priv);
int iwlagn_hw_valid_rtc_data_addr(u32 addr);
int iwlagn_send_tx_power(struct iwl_priv *priv);
void iwlagn_temperature(struct iwl_priv *priv);
u16 iwlagn_eeprom_calib_version(struct iwl_priv *priv);
const u8 *iwlagn_eeprom_query_addr(const struct iwl_priv *priv,
				   size_t offset);
void iwlagn_rx_queue_reset(struct iwl_priv *priv, struct iwl_rx_queue *rxq);
int iwlagn_rx_init(struct iwl_priv *priv, struct iwl_rx_queue *rxq);
int iwlagn_hw_nic_init(struct iwl_priv *priv);

/* rx */
void iwlagn_rx_queue_restock(struct iwl_priv *priv);
void iwlagn_rx_allocate(struct iwl_priv *priv, gfp_t priority);
void iwlagn_rx_replenish(struct iwl_priv *priv);
void iwlagn_rx_replenish_now(struct iwl_priv *priv);
void iwlagn_rx_queue_free(struct iwl_priv *priv, struct iwl_rx_queue *rxq);
int iwlagn_rxq_stop(struct iwl_priv *priv);
int iwlagn_hwrate_to_mac80211_idx(u32 rate_n_flags, enum ieee80211_band band);
void iwlagn_rx_reply_rx(struct iwl_priv *priv,
		     struct iwl_rx_mem_buffer *rxb);
void iwlagn_rx_reply_rx_phy(struct iwl_priv *priv,
			 struct iwl_rx_mem_buffer *rxb);

/* tx */
void iwlagn_hwrate_to_tx_control(struct iwl_priv *priv, u32 rate_n_flags,
			      struct ieee80211_tx_info *info);
int iwlagn_tx_skb(struct iwl_priv *priv, struct sk_buff *skb);
int iwlagn_tx_agg_start(struct iwl_priv *priv, struct ieee80211_vif *vif,
			struct ieee80211_sta *sta, u16 tid, u16 *ssn);
int iwlagn_tx_agg_stop(struct iwl_priv *priv, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta, u16 tid);
int iwlagn_txq_check_empty(struct iwl_priv *priv,
			   int sta_id, u8 tid, int txq_id);
void iwlagn_rx_reply_compressed_ba(struct iwl_priv *priv,
				struct iwl_rx_mem_buffer *rxb);
int iwlagn_tx_queue_reclaim(struct iwl_priv *priv, int txq_id, int index);
void iwlagn_hw_txq_ctx_free(struct iwl_priv *priv);
int iwlagn_txq_ctx_alloc(struct iwl_priv *priv);
void iwlagn_txq_ctx_reset(struct iwl_priv *priv);
void iwlagn_txq_ctx_stop(struct iwl_priv *priv);

static inline u32 iwl_tx_status_to_mac80211(u32 status)
{
	status &= TX_STATUS_MSK;

	switch (status) {
	case TX_STATUS_SUCCESS:
	case TX_STATUS_DIRECT_DONE:
		return IEEE80211_TX_STAT_ACK;
	case TX_STATUS_FAIL_DEST_PS:
		return IEEE80211_TX_STAT_TX_FILTERED;
	default:
		return 0;
	}
}

static inline bool iwl_is_tx_success(u32 status)
{
	status &= TX_STATUS_MSK;
	return (status == TX_STATUS_SUCCESS) ||
	       (status == TX_STATUS_DIRECT_DONE);
}

/* scan */
void iwlagn_request_scan(struct iwl_priv *priv, struct ieee80211_vif *vif);

/* station mgmt */
int iwlagn_manage_ibss_station(struct iwl_priv *priv,
			       struct ieee80211_vif *vif, bool add);

#endif /* __iwl_agn_h__ */
