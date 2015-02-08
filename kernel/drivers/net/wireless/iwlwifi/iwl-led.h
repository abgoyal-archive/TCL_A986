

#ifndef __iwl_leds_h__
#define __iwl_leds_h__


struct iwl_priv;

#define IWL_LED_SOLID 11
#define IWL_LED_NAME_LEN 31
#define IWL_DEF_LED_INTRVL cpu_to_le32(1000)

#define IWL_LED_ACTIVITY       (0<<1)
#define IWL_LED_LINK           (1<<1)

enum led_type {
	IWL_LED_TRG_TX,
	IWL_LED_TRG_RX,
	IWL_LED_TRG_ASSOC,
	IWL_LED_TRG_RADIO,
	IWL_LED_TRG_MAX,
};

enum iwl_led_mode {
	IWL_LED_BLINK,
	IWL_LED_RF_STATE,
};

void iwl_leds_init(struct iwl_priv *priv);
void iwl_leds_background(struct iwl_priv *priv);
int iwl_led_start(struct iwl_priv *priv);
int iwl_led_associate(struct iwl_priv *priv);
int iwl_led_disassociate(struct iwl_priv *priv);

#endif /* __iwl_leds_h__ */
