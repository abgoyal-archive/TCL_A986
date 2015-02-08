

#include <linux/etherdevice.h>
#include <linux/slab.h>
#include <net/mac80211.h>
#include <asm/unaligned.h>
#include "iwl-eeprom.h"
#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-sta.h"
#include "iwl-io.h"
#include "iwl-calib.h"
#include "iwl-helpers.h"
/************************** RX-FUNCTIONS ****************************/

int iwl_rx_queue_space(const struct iwl_rx_queue *q)
{
	int s = q->read - q->write;
	if (s <= 0)
		s += RX_QUEUE_SIZE;
	/* keep some buffer to not confuse full and empty queue */
	s -= 2;
	if (s < 0)
		s = 0;
	return s;
}
EXPORT_SYMBOL(iwl_rx_queue_space);

void iwl_rx_queue_update_write_ptr(struct iwl_priv *priv, struct iwl_rx_queue *q)
{
	unsigned long flags;
	u32 rx_wrt_ptr_reg = priv->hw_params.rx_wrt_ptr_reg;
	u32 reg;

	spin_lock_irqsave(&q->lock, flags);

	if (q->need_update == 0)
		goto exit_unlock;

	/* If power-saving is in use, make sure device is awake */
	if (test_bit(STATUS_POWER_PMI, &priv->status)) {
		reg = iwl_read32(priv, CSR_UCODE_DRV_GP1);

		if (reg & CSR_UCODE_DRV_GP1_BIT_MAC_SLEEP) {
			IWL_DEBUG_INFO(priv, "Rx queue requesting wakeup, GP1 = 0x%x\n",
				      reg);
			iwl_set_bit(priv, CSR_GP_CNTRL,
				    CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
			goto exit_unlock;
		}

		q->write_actual = (q->write & ~0x7);
		iwl_write_direct32(priv, rx_wrt_ptr_reg, q->write_actual);

	/* Else device is assumed to be awake */
	} else {
		/* Device expects a multiple of 8 */
		q->write_actual = (q->write & ~0x7);
		iwl_write_direct32(priv, rx_wrt_ptr_reg, q->write_actual);
	}

	q->need_update = 0;

 exit_unlock:
	spin_unlock_irqrestore(&q->lock, flags);
}
EXPORT_SYMBOL(iwl_rx_queue_update_write_ptr);

int iwl_rx_queue_alloc(struct iwl_priv *priv)
{
	struct iwl_rx_queue *rxq = &priv->rxq;
	struct device *dev = &priv->pci_dev->dev;
	int i;

	spin_lock_init(&rxq->lock);
	INIT_LIST_HEAD(&rxq->rx_free);
	INIT_LIST_HEAD(&rxq->rx_used);

	/* Alloc the circular buffer of Read Buffer Descriptors (RBDs) */
	rxq->bd = dma_alloc_coherent(dev, 4 * RX_QUEUE_SIZE, &rxq->dma_addr,
				     GFP_KERNEL);
	if (!rxq->bd)
		goto err_bd;

	rxq->rb_stts = dma_alloc_coherent(dev, sizeof(struct iwl_rb_status),
					  &rxq->rb_stts_dma, GFP_KERNEL);
	if (!rxq->rb_stts)
		goto err_rb;

	/* Fill the rx_used queue with _all_ of the Rx buffers */
	for (i = 0; i < RX_FREE_BUFFERS + RX_QUEUE_SIZE; i++)
		list_add_tail(&rxq->pool[i].list, &rxq->rx_used);

	/* Set us so that we have processed and used all buffers, but have
	 * not restocked the Rx queue with fresh buffers */
	rxq->read = rxq->write = 0;
	rxq->write_actual = 0;
	rxq->free_count = 0;
	rxq->need_update = 0;
	return 0;

err_rb:
	dma_free_coherent(&priv->pci_dev->dev, 4 * RX_QUEUE_SIZE, rxq->bd,
			  rxq->dma_addr);
err_bd:
	return -ENOMEM;
}
EXPORT_SYMBOL(iwl_rx_queue_alloc);

void iwl_rx_missed_beacon_notif(struct iwl_priv *priv,
				struct iwl_rx_mem_buffer *rxb)

{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_missed_beacon_notif *missed_beacon;

	missed_beacon = &pkt->u.missed_beacon;
	if (le32_to_cpu(missed_beacon->consecutive_missed_beacons) >
	    priv->missed_beacon_threshold) {
		IWL_DEBUG_CALIB(priv, "missed bcn cnsq %d totl %d rcd %d expctd %d\n",
		    le32_to_cpu(missed_beacon->consecutive_missed_beacons),
		    le32_to_cpu(missed_beacon->total_missed_becons),
		    le32_to_cpu(missed_beacon->num_recvd_beacons),
		    le32_to_cpu(missed_beacon->num_expected_beacons));
		if (!test_bit(STATUS_SCANNING, &priv->status))
			iwl_init_sensitivity(priv);
	}
}
EXPORT_SYMBOL(iwl_rx_missed_beacon_notif);

void iwl_rx_spectrum_measure_notif(struct iwl_priv *priv,
					  struct iwl_rx_mem_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_spectrum_notification *report = &(pkt->u.spectrum_notif);

	if (!report->state) {
		IWL_DEBUG_11H(priv,
			"Spectrum Measure Notification: Start\n");
		return;
	}

	memcpy(&priv->measure_report, report, sizeof(*report));
	priv->measurement_status |= MEASUREMENT_READY;
}
EXPORT_SYMBOL(iwl_rx_spectrum_measure_notif);



static void iwl_rx_calc_noise(struct iwl_priv *priv)
{
	struct statistics_rx_non_phy *rx_info
				= &(priv->statistics.rx.general);
	int num_active_rx = 0;
	int total_silence = 0;
	int bcn_silence_a =
		le32_to_cpu(rx_info->beacon_silence_rssi_a) & IN_BAND_FILTER;
	int bcn_silence_b =
		le32_to_cpu(rx_info->beacon_silence_rssi_b) & IN_BAND_FILTER;
	int bcn_silence_c =
		le32_to_cpu(rx_info->beacon_silence_rssi_c) & IN_BAND_FILTER;
	int last_rx_noise;

	if (bcn_silence_a) {
		total_silence += bcn_silence_a;
		num_active_rx++;
	}
	if (bcn_silence_b) {
		total_silence += bcn_silence_b;
		num_active_rx++;
	}
	if (bcn_silence_c) {
		total_silence += bcn_silence_c;
		num_active_rx++;
	}

	/* Average among active antennas */
	if (num_active_rx)
		last_rx_noise = (total_silence / num_active_rx) - 107;
	else
		last_rx_noise = IWL_NOISE_MEAS_NOT_AVAILABLE;

	IWL_DEBUG_CALIB(priv, "inband silence a %u, b %u, c %u, dBm %d\n",
			bcn_silence_a, bcn_silence_b, bcn_silence_c,
			last_rx_noise);
}

#ifdef CONFIG_IWLWIFI_DEBUG
static void iwl_accumulative_statistics(struct iwl_priv *priv,
					__le32 *stats)
{
	int i;
	__le32 *prev_stats;
	u32 *accum_stats;
	u32 *delta, *max_delta;

	prev_stats = (__le32 *)&priv->statistics;
	accum_stats = (u32 *)&priv->accum_statistics;
	delta = (u32 *)&priv->delta_statistics;
	max_delta = (u32 *)&priv->max_delta;

	for (i = sizeof(__le32); i < sizeof(struct iwl_notif_statistics);
	     i += sizeof(__le32), stats++, prev_stats++, delta++,
	     max_delta++, accum_stats++) {
		if (le32_to_cpu(*stats) > le32_to_cpu(*prev_stats)) {
			*delta = (le32_to_cpu(*stats) -
				le32_to_cpu(*prev_stats));
			*accum_stats += *delta;
			if (*delta > *max_delta)
				*max_delta = *delta;
		}
	}

	/* reset accumulative statistics for "no-counter" type statistics */
	priv->accum_statistics.general.temperature =
		priv->statistics.general.temperature;
	priv->accum_statistics.general.temperature_m =
		priv->statistics.general.temperature_m;
	priv->accum_statistics.general.ttl_timestamp =
		priv->statistics.general.ttl_timestamp;
	priv->accum_statistics.tx.tx_power.ant_a =
		priv->statistics.tx.tx_power.ant_a;
	priv->accum_statistics.tx.tx_power.ant_b =
		priv->statistics.tx.tx_power.ant_b;
	priv->accum_statistics.tx.tx_power.ant_c =
		priv->statistics.tx.tx_power.ant_c;
}
#endif

#define REG_RECALIB_PERIOD (60)

bool iwl_good_plcp_health(struct iwl_priv *priv,
				struct iwl_rx_packet *pkt)
{
	bool rc = true;
	int combined_plcp_delta;
	unsigned int plcp_msec;
	unsigned long plcp_received_jiffies;

	/*
	 * check for plcp_err and trigger radio reset if it exceeds
	 * the plcp error threshold plcp_delta.
	 */
	plcp_received_jiffies = jiffies;
	plcp_msec = jiffies_to_msecs((long) plcp_received_jiffies -
					(long) priv->plcp_jiffies);
	priv->plcp_jiffies = plcp_received_jiffies;
	/*
	 * check to make sure plcp_msec is not 0 to prevent division
	 * by zero.
	 */
	if (plcp_msec) {
		combined_plcp_delta =
			(le32_to_cpu(pkt->u.stats.rx.ofdm.plcp_err) -
			le32_to_cpu(priv->statistics.rx.ofdm.plcp_err)) +
			(le32_to_cpu(pkt->u.stats.rx.ofdm_ht.plcp_err) -
			le32_to_cpu(priv->statistics.rx.ofdm_ht.plcp_err));

		if ((combined_plcp_delta > 0) &&
		    ((combined_plcp_delta * 100) / plcp_msec) >
			priv->cfg->plcp_delta_threshold) {
			/*
			 * if plcp_err exceed the threshold,
			 * the following data is printed in csv format:
			 *    Text: plcp_err exceeded %d,
			 *    Received ofdm.plcp_err,
			 *    Current ofdm.plcp_err,
			 *    Received ofdm_ht.plcp_err,
			 *    Current ofdm_ht.plcp_err,
			 *    combined_plcp_delta,
			 *    plcp_msec
			 */
			IWL_DEBUG_RADIO(priv, "plcp_err exceeded %u, "
				"%u, %u, %u, %u, %d, %u mSecs\n",
				priv->cfg->plcp_delta_threshold,
				le32_to_cpu(pkt->u.stats.rx.ofdm.plcp_err),
				le32_to_cpu(priv->statistics.rx.ofdm.plcp_err),
				le32_to_cpu(pkt->u.stats.rx.ofdm_ht.plcp_err),
				le32_to_cpu(
				  priv->statistics.rx.ofdm_ht.plcp_err),
				combined_plcp_delta, plcp_msec);
			rc = false;
		}
	}
	return rc;
}
EXPORT_SYMBOL(iwl_good_plcp_health);

void iwl_recover_from_statistics(struct iwl_priv *priv,
				struct iwl_rx_packet *pkt)
{
	if (test_bit(STATUS_EXIT_PENDING, &priv->status))
		return;
	if (iwl_is_associated(priv)) {
		if (priv->cfg->ops->lib->check_ack_health) {
			if (!priv->cfg->ops->lib->check_ack_health(
			    priv, pkt)) {
				/*
				 * low ack count detected
				 * restart Firmware
				 */
				IWL_ERR(priv, "low ack count detected, "
					"restart firmware\n");
				if (!iwl_force_reset(priv, IWL_FW_RESET))
					return;
			}
		}
		if (priv->cfg->ops->lib->check_plcp_health) {
			if (!priv->cfg->ops->lib->check_plcp_health(
			    priv, pkt)) {
				/*
				 * high plcp error detected
				 * reset Radio
				 */
				iwl_force_reset(priv, IWL_RF_RESET);
			}
		}
	}
}
EXPORT_SYMBOL(iwl_recover_from_statistics);

void iwl_rx_statistics(struct iwl_priv *priv,
			      struct iwl_rx_mem_buffer *rxb)
{
	int change;
	struct iwl_rx_packet *pkt = rxb_addr(rxb);


	IWL_DEBUG_RX(priv, "Statistics notification received (%d vs %d).\n",
		     (int)sizeof(priv->statistics),
		     le32_to_cpu(pkt->len_n_flags) & FH_RSCSR_FRAME_SIZE_MSK);

	change = ((priv->statistics.general.temperature !=
		   pkt->u.stats.general.temperature) ||
		  ((priv->statistics.flag &
		    STATISTICS_REPLY_FLG_HT40_MODE_MSK) !=
		   (pkt->u.stats.flag & STATISTICS_REPLY_FLG_HT40_MODE_MSK)));

#ifdef CONFIG_IWLWIFI_DEBUG
	iwl_accumulative_statistics(priv, (__le32 *)&pkt->u.stats);
#endif
	iwl_recover_from_statistics(priv, pkt);

	memcpy(&priv->statistics, &pkt->u.stats, sizeof(priv->statistics));

	set_bit(STATUS_STATISTICS, &priv->status);

	/* Reschedule the statistics timer to occur in
	 * REG_RECALIB_PERIOD seconds to ensure we get a
	 * thermal update even if the uCode doesn't give
	 * us one */
	mod_timer(&priv->statistics_periodic, jiffies +
		  msecs_to_jiffies(REG_RECALIB_PERIOD * 1000));

	if (unlikely(!test_bit(STATUS_SCANNING, &priv->status)) &&
	    (pkt->hdr.cmd == STATISTICS_NOTIFICATION)) {
		iwl_rx_calc_noise(priv);
		queue_work(priv->workqueue, &priv->run_time_calib_work);
	}
	if (priv->cfg->ops->lib->temp_ops.temperature && change)
		priv->cfg->ops->lib->temp_ops.temperature(priv);
}
EXPORT_SYMBOL(iwl_rx_statistics);

void iwl_reply_statistics(struct iwl_priv *priv,
			      struct iwl_rx_mem_buffer *rxb)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);

	if (le32_to_cpu(pkt->u.stats.flag) & UCODE_STATISTICS_CLEAR_MSK) {
#ifdef CONFIG_IWLWIFI_DEBUG
		memset(&priv->accum_statistics, 0,
			sizeof(struct iwl_notif_statistics));
		memset(&priv->delta_statistics, 0,
			sizeof(struct iwl_notif_statistics));
		memset(&priv->max_delta, 0,
			sizeof(struct iwl_notif_statistics));
#endif
		IWL_DEBUG_RX(priv, "Statistics have been cleared\n");
	}
	iwl_rx_statistics(priv, rxb);
}
EXPORT_SYMBOL(iwl_reply_statistics);

int iwl_set_decrypted_flag(struct iwl_priv *priv,
			   struct ieee80211_hdr *hdr,
			   u32 decrypt_res,
			   struct ieee80211_rx_status *stats)
{
	u16 fc = le16_to_cpu(hdr->frame_control);

	if (priv->active_rxon.filter_flags & RXON_FILTER_DIS_DECRYPT_MSK)
		return 0;

	if (!(fc & IEEE80211_FCTL_PROTECTED))
		return 0;

	IWL_DEBUG_RX(priv, "decrypt_res:0x%x\n", decrypt_res);
	switch (decrypt_res & RX_RES_STATUS_SEC_TYPE_MSK) {
	case RX_RES_STATUS_SEC_TYPE_TKIP:
		/* The uCode has got a bad phase 1 Key, pushes the packet.
		 * Decryption will be done in SW. */
		if ((decrypt_res & RX_RES_STATUS_DECRYPT_TYPE_MSK) ==
		    RX_RES_STATUS_BAD_KEY_TTAK)
			break;

	case RX_RES_STATUS_SEC_TYPE_WEP:
		if ((decrypt_res & RX_RES_STATUS_DECRYPT_TYPE_MSK) ==
		    RX_RES_STATUS_BAD_ICV_MIC) {
			/* bad ICV, the packet is destroyed since the
			 * decryption is inplace, drop it */
			IWL_DEBUG_RX(priv, "Packet destroyed\n");
			return -1;
		}
	case RX_RES_STATUS_SEC_TYPE_CCMP:
		if ((decrypt_res & RX_RES_STATUS_DECRYPT_TYPE_MSK) ==
		    RX_RES_STATUS_DECRYPT_OK) {
			IWL_DEBUG_RX(priv, "hw decrypt successfully!!!\n");
			stats->flag |= RX_FLAG_DECRYPTED;
		}
		break;

	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(iwl_set_decrypted_flag);
