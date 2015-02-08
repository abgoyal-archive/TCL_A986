

#ifndef ATH_H
#define ATH_H

#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <net/mac80211.h>

#define	ATH_KEYMAX	        128     /* max key cache size we handle */

static const u8 ath_bcast_mac[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct ath_ani {
	bool caldone;
	int16_t noise_floor;
	unsigned int longcal_timer;
	unsigned int shortcal_timer;
	unsigned int resetcal_timer;
	unsigned int checkani_timer;
	struct timer_list timer;
};

enum ath_device_state {
	ATH_HW_UNAVAILABLE,
	ATH_HW_INITIALIZED,
};

enum ath_bus_type {
	ATH_PCI,
	ATH_AHB,
	ATH_USB,
};

struct reg_dmn_pair_mapping {
	u16 regDmnEnum;
	u16 reg_5ghz_ctl;
	u16 reg_2ghz_ctl;
};

struct ath_regulatory {
	char alpha2[2];
	u16 country_code;
	u16 max_power_level;
	u32 tp_scale;
	u16 current_rd;
	u16 current_rd_ext;
	int16_t power_limit;
	struct reg_dmn_pair_mapping *regpair;
};

struct ath_ops {
	unsigned int (*read)(void *, u32 reg_offset);
	void (*write)(void *, u32 val, u32 reg_offset);
	void (*enable_write_buffer)(void *);
	void (*disable_write_buffer)(void *);
	void (*write_flush) (void *);
};

struct ath_common;

struct ath_bus_ops {
	enum ath_bus_type ath_bus_type;
	void (*read_cachesize)(struct ath_common *common, int *csz);
	bool (*eeprom_read)(struct ath_common *common, u32 off, u16 *data);
	void (*bt_coex_prep)(struct ath_common *common);
};

struct ath_common {
	void *ah;
	void *priv;
	struct ieee80211_hw *hw;
	int debug_mask;
	enum ath_device_state state;

	struct ath_ani ani;

	u16 cachelsz;
	u16 curaid;
	u8 macaddr[ETH_ALEN];
	u8 curbssid[ETH_ALEN];
	u8 bssidmask[ETH_ALEN];

	u8 tx_chainmask;
	u8 rx_chainmask;

	u32 rx_bufsize;

	u32 keymax;
	DECLARE_BITMAP(keymap, ATH_KEYMAX);
	u8 splitmic;

	struct ath_regulatory regulatory;
	const struct ath_ops *ops;
	const struct ath_bus_ops *bus_ops;
};

struct sk_buff *ath_rxbuf_alloc(struct ath_common *common,
				u32 len,
				gfp_t gfp_mask);

void ath_hw_setbssidmask(struct ath_common *common);

#endif /* ATH_H */
