


#ifndef RT2X00LIB_H
#define RT2X00LIB_H

#include "rt2x00dump.h"

#define LINK_TUNE_INTERVAL	round_jiffies_relative(HZ)

struct rt2x00_rate {
	unsigned short flags;
#define DEV_RATE_CCK			0x0001
#define DEV_RATE_OFDM			0x0002
#define DEV_RATE_SHORT_PREAMBLE		0x0004

	unsigned short bitrate; /* In 100kbit/s */
	unsigned short ratemask;

	unsigned short plcp;
	unsigned short mcs;
};

extern const struct rt2x00_rate rt2x00_supported_rates[12];

static inline const struct rt2x00_rate *rt2x00_get_rate(const u16 hw_value)
{
	return &rt2x00_supported_rates[hw_value & 0xff];
}

#define RATE_MCS(__mode, __mcs) \
	( (((__mode) & 0x00ff) << 8) | ((__mcs) & 0x00ff) )

static inline int rt2x00_get_rate_mcs(const u16 mcs_value)
{
	return (mcs_value & 0x00ff);
}

int rt2x00lib_enable_radio(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_disable_radio(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_toggle_rx(struct rt2x00_dev *rt2x00dev, enum dev_state state);

int rt2x00lib_start(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_stop(struct rt2x00_dev *rt2x00dev);

void rt2x00lib_config_intf(struct rt2x00_dev *rt2x00dev,
			   struct rt2x00_intf *intf,
			   enum nl80211_iftype type,
			   const u8 *mac, const u8 *bssid);
void rt2x00lib_config_erp(struct rt2x00_dev *rt2x00dev,
			  struct rt2x00_intf *intf,
			  struct ieee80211_bss_conf *conf);
void rt2x00lib_config_antenna(struct rt2x00_dev *rt2x00dev,
			      struct antenna_setup ant);
void rt2x00lib_config(struct rt2x00_dev *rt2x00dev,
		      struct ieee80211_conf *conf,
		      const unsigned int changed_flags);


struct sk_buff *rt2x00queue_alloc_rxskb(struct rt2x00_dev *rt2x00dev,
					struct queue_entry *entry);

void rt2x00queue_unmap_skb(struct rt2x00_dev *rt2x00dev, struct sk_buff *skb);

void rt2x00queue_free_skb(struct rt2x00_dev *rt2x00dev, struct sk_buff *skb);

void rt2x00queue_align_frame(struct sk_buff *skb);

void rt2x00queue_align_payload(struct sk_buff *skb, unsigned int header_length);

void rt2x00queue_insert_l2pad(struct sk_buff *skb, unsigned int header_length);

void rt2x00queue_remove_l2pad(struct sk_buff *skb, unsigned int header_length);

int rt2x00queue_write_tx_frame(struct data_queue *queue, struct sk_buff *skb,
			       bool local);

int rt2x00queue_update_beacon(struct rt2x00_dev *rt2x00dev,
			      struct ieee80211_vif *vif,
			      const bool enable_beacon);

void rt2x00queue_index_inc(struct data_queue *queue, enum queue_index index);

void rt2x00queue_stop_queues(struct rt2x00_dev *rt2x00dev);

void rt2x00queue_init_queues(struct rt2x00_dev *rt2x00dev);

int rt2x00queue_initialize(struct rt2x00_dev *rt2x00dev);
void rt2x00queue_uninitialize(struct rt2x00_dev *rt2x00dev);
int rt2x00queue_allocate(struct rt2x00_dev *rt2x00dev);
void rt2x00queue_free(struct rt2x00_dev *rt2x00dev);

void rt2x00link_update_stats(struct rt2x00_dev *rt2x00dev,
			     struct sk_buff *skb,
			     struct rxdone_entry_desc *rxdesc);

void rt2x00link_start_tuner(struct rt2x00_dev *rt2x00dev);

void rt2x00link_stop_tuner(struct rt2x00_dev *rt2x00dev);

void rt2x00link_reset_tuner(struct rt2x00_dev *rt2x00dev, bool antenna);

void rt2x00link_register(struct rt2x00_dev *rt2x00dev);

#ifdef CONFIG_RT2X00_LIB_FIRMWARE
int rt2x00lib_load_firmware(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_free_firmware(struct rt2x00_dev *rt2x00dev);
#else
static inline int rt2x00lib_load_firmware(struct rt2x00_dev *rt2x00dev)
{
	return 0;
}
static inline void rt2x00lib_free_firmware(struct rt2x00_dev *rt2x00dev)
{
}
#endif /* CONFIG_RT2X00_LIB_FIRMWARE */

#ifdef CONFIG_RT2X00_LIB_DEBUGFS
void rt2x00debug_register(struct rt2x00_dev *rt2x00dev);
void rt2x00debug_deregister(struct rt2x00_dev *rt2x00dev);
void rt2x00debug_dump_frame(struct rt2x00_dev *rt2x00dev,
			    enum rt2x00_dump_type type, struct sk_buff *skb);
void rt2x00debug_update_crypto(struct rt2x00_dev *rt2x00dev,
			       struct rxdone_entry_desc *rxdesc);
#else
static inline void rt2x00debug_register(struct rt2x00_dev *rt2x00dev)
{
}

static inline void rt2x00debug_deregister(struct rt2x00_dev *rt2x00dev)
{
}

static inline void rt2x00debug_dump_frame(struct rt2x00_dev *rt2x00dev,
					  enum rt2x00_dump_type type,
					  struct sk_buff *skb)
{
}

static inline void rt2x00debug_update_crypto(struct rt2x00_dev *rt2x00dev,
					     struct rxdone_entry_desc *rxdesc)
{
}
#endif /* CONFIG_RT2X00_LIB_DEBUGFS */

#ifdef CONFIG_RT2X00_LIB_CRYPTO
enum cipher rt2x00crypto_key_to_cipher(struct ieee80211_key_conf *key);
void rt2x00crypto_create_tx_descriptor(struct queue_entry *entry,
				       struct txentry_desc *txdesc);
unsigned int rt2x00crypto_tx_overhead(struct rt2x00_dev *rt2x00dev,
				      struct sk_buff *skb);
void rt2x00crypto_tx_copy_iv(struct sk_buff *skb,
			     struct txentry_desc *txdesc);
void rt2x00crypto_tx_remove_iv(struct sk_buff *skb,
			       struct txentry_desc *txdesc);
void rt2x00crypto_tx_insert_iv(struct sk_buff *skb, unsigned int header_length);
void rt2x00crypto_rx_insert_iv(struct sk_buff *skb,
			       unsigned int header_length,
			       struct rxdone_entry_desc *rxdesc);
#else
static inline enum cipher rt2x00crypto_key_to_cipher(struct ieee80211_key_conf *key)
{
	return CIPHER_NONE;
}

static inline void rt2x00crypto_create_tx_descriptor(struct queue_entry *entry,
						     struct txentry_desc *txdesc)
{
}

static inline unsigned int rt2x00crypto_tx_overhead(struct rt2x00_dev *rt2x00dev,
						    struct sk_buff *skb)
{
	return 0;
}

static inline void rt2x00crypto_tx_copy_iv(struct sk_buff *skb,
					   struct txentry_desc *txdesc)
{
}

static inline void rt2x00crypto_tx_remove_iv(struct sk_buff *skb,
					     struct txentry_desc *txdesc)
{
}

static inline void rt2x00crypto_tx_insert_iv(struct sk_buff *skb,
					     unsigned int header_length)
{
}

static inline void rt2x00crypto_rx_insert_iv(struct sk_buff *skb,
					     unsigned int header_length,
					     struct rxdone_entry_desc *rxdesc)
{
}
#endif /* CONFIG_RT2X00_LIB_CRYPTO */

#ifdef CONFIG_RT2X00_LIB_HT
void rt2x00ht_create_tx_descriptor(struct queue_entry *entry,
				   struct txentry_desc *txdesc,
				   const struct rt2x00_rate *hwrate);
#else
static inline void rt2x00ht_create_tx_descriptor(struct queue_entry *entry,
						 struct txentry_desc *txdesc,
						 const struct rt2x00_rate *hwrate)
{
}
#endif /* CONFIG_RT2X00_LIB_HT */

static inline void rt2x00rfkill_register(struct rt2x00_dev *rt2x00dev)
{
	if (test_bit(CONFIG_SUPPORT_HW_BUTTON, &rt2x00dev->flags))
		wiphy_rfkill_start_polling(rt2x00dev->hw->wiphy);
}

static inline void rt2x00rfkill_unregister(struct rt2x00_dev *rt2x00dev)
{
	if (test_bit(CONFIG_SUPPORT_HW_BUTTON, &rt2x00dev->flags))
		wiphy_rfkill_stop_polling(rt2x00dev->hw->wiphy);
}

#ifdef CONFIG_RT2X00_LIB_LEDS
void rt2x00leds_led_quality(struct rt2x00_dev *rt2x00dev, int rssi);
void rt2x00led_led_activity(struct rt2x00_dev *rt2x00dev, bool enabled);
void rt2x00leds_led_assoc(struct rt2x00_dev *rt2x00dev, bool enabled);
void rt2x00leds_led_radio(struct rt2x00_dev *rt2x00dev, bool enabled);
void rt2x00leds_register(struct rt2x00_dev *rt2x00dev);
void rt2x00leds_unregister(struct rt2x00_dev *rt2x00dev);
void rt2x00leds_suspend(struct rt2x00_dev *rt2x00dev);
void rt2x00leds_resume(struct rt2x00_dev *rt2x00dev);
#else
static inline void rt2x00leds_led_quality(struct rt2x00_dev *rt2x00dev,
					  int rssi)
{
}

static inline void rt2x00led_led_activity(struct rt2x00_dev *rt2x00dev,
					  bool enabled)
{
}

static inline void rt2x00leds_led_assoc(struct rt2x00_dev *rt2x00dev,
					bool enabled)
{
}

static inline void rt2x00leds_led_radio(struct rt2x00_dev *rt2x00dev,
					bool enabled)
{
}

static inline void rt2x00leds_register(struct rt2x00_dev *rt2x00dev)
{
}

static inline void rt2x00leds_unregister(struct rt2x00_dev *rt2x00dev)
{
}

static inline void rt2x00leds_suspend(struct rt2x00_dev *rt2x00dev)
{
}

static inline void rt2x00leds_resume(struct rt2x00_dev *rt2x00dev)
{
}
#endif /* CONFIG_RT2X00_LIB_LEDS */

#endif /* RT2X00LIB_H */
