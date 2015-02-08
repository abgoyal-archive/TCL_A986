


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <net/mac80211.h>
#include <linux/etherdevice.h>
#include <asm/unaligned.h>

#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-io.h"

/* default: IWL_LED_BLINK(0) using blinking index table */
static int led_mode;
module_param(led_mode, int, S_IRUGO);
MODULE_PARM_DESC(led_mode, "led mode: 0=blinking, 1=On(RF On)/Off(RF Off), "
			   "(default 0)");


static const struct {
	u16 tpt;	/* Mb/s */
	u8 on_time;
	u8 off_time;
} blink_tbl[] =
{
	{300, 25, 25},
	{200, 40, 40},
	{100, 55, 55},
	{70, 65, 65},
	{50, 75, 75},
	{20, 85, 85},
	{10, 95, 95},
	{5, 110, 110},
	{1, 130, 130},
	{0, 167, 167},
	/* SOLID_ON */
	{-1, IWL_LED_SOLID, 0}
};

#define IWL_1MB_RATE (128 * 1024)
#define IWL_LED_THRESHOLD (16)
#define IWL_MAX_BLINK_TBL (ARRAY_SIZE(blink_tbl) - 1) /* exclude SOLID_ON */
#define IWL_SOLID_BLINK_IDX (ARRAY_SIZE(blink_tbl) - 1)

static inline u8 iwl_blink_compensation(struct iwl_priv *priv,
				    u8 time, u16 compensation)
{
	if (!compensation) {
		IWL_ERR(priv, "undefined blink compensation: "
			"use pre-defined blinking time\n");
		return time;
	}

	return (u8)((time * compensation) >> 6);
}

/* Set led pattern command */
static int iwl_led_pattern(struct iwl_priv *priv, unsigned int idx)
{
	struct iwl_led_cmd led_cmd = {
		.id = IWL_LED_LINK,
		.interval = IWL_DEF_LED_INTRVL
	};

	BUG_ON(idx > IWL_MAX_BLINK_TBL);

	IWL_DEBUG_LED(priv, "Led blink time compensation= %u\n",
			priv->cfg->led_compensation);
	led_cmd.on =
		iwl_blink_compensation(priv, blink_tbl[idx].on_time,
					priv->cfg->led_compensation);
	led_cmd.off =
		iwl_blink_compensation(priv, blink_tbl[idx].off_time,
					priv->cfg->led_compensation);

	return priv->cfg->ops->led->cmd(priv, &led_cmd);
}

int iwl_led_start(struct iwl_priv *priv)
{
	return priv->cfg->ops->led->on(priv);
}
EXPORT_SYMBOL(iwl_led_start);

int iwl_led_associate(struct iwl_priv *priv)
{
	IWL_DEBUG_LED(priv, "Associated\n");
	if (led_mode == IWL_LED_BLINK)
		priv->allow_blinking = 1;
	priv->last_blink_time = jiffies;

	return 0;
}

int iwl_led_disassociate(struct iwl_priv *priv)
{
	priv->allow_blinking = 0;

	return 0;
}

static int iwl_get_blink_rate(struct iwl_priv *priv)
{
	int i;
	/* count both tx and rx traffic to be able to
	 * handle traffic in either direction
	 */
	u64 current_tpt = priv->tx_stats.data_bytes +
			  priv->rx_stats.data_bytes;
	s64 tpt = current_tpt - priv->led_tpt;

	if (tpt < 0) /* wraparound */
		tpt = -tpt;

	IWL_DEBUG_LED(priv, "tpt %lld current_tpt %llu\n",
		(long long)tpt,
		(unsigned long long)current_tpt);
	priv->led_tpt = current_tpt;

	if (!priv->allow_blinking)
		i = IWL_MAX_BLINK_TBL;
	else
		for (i = 0; i < IWL_MAX_BLINK_TBL; i++)
			if (tpt > (blink_tbl[i].tpt * IWL_1MB_RATE))
				break;

	IWL_DEBUG_LED(priv, "LED BLINK IDX=%d\n", i);
	return i;
}

void iwl_leds_background(struct iwl_priv *priv)
{
	u8 blink_idx;

	if (test_bit(STATUS_EXIT_PENDING, &priv->status)) {
		priv->last_blink_time = 0;
		return;
	}
	if (iwl_is_rfkill(priv)) {
		priv->last_blink_time = 0;
		return;
	}

	if (!priv->allow_blinking) {
		priv->last_blink_time = 0;
		if (priv->last_blink_rate != IWL_SOLID_BLINK_IDX) {
			priv->last_blink_rate = IWL_SOLID_BLINK_IDX;
			iwl_led_pattern(priv, IWL_SOLID_BLINK_IDX);
		}
		return;
	}
	if (!priv->last_blink_time ||
	    !time_after(jiffies, priv->last_blink_time +
			msecs_to_jiffies(1000)))
		return;

	blink_idx = iwl_get_blink_rate(priv);

	/* call only if blink rate change */
	if (blink_idx != priv->last_blink_rate)
		iwl_led_pattern(priv, blink_idx);

	priv->last_blink_time = jiffies;
	priv->last_blink_rate = blink_idx;
}
EXPORT_SYMBOL(iwl_leds_background);

void iwl_leds_init(struct iwl_priv *priv)
{
	priv->last_blink_rate = 0;
	priv->last_blink_time = 0;
	priv->allow_blinking = 0;
}
EXPORT_SYMBOL(iwl_leds_init);
