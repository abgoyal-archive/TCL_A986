

#include "base.h"


static inline void ath5k_rfkill_disable(struct ath5k_softc *sc)
{
	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "rfkill disable (gpio:%d polarity:%d)\n",
		sc->rf_kill.gpio, sc->rf_kill.polarity);
	ath5k_hw_set_gpio_output(sc->ah, sc->rf_kill.gpio);
	ath5k_hw_set_gpio(sc->ah, sc->rf_kill.gpio, !sc->rf_kill.polarity);
}


static inline void ath5k_rfkill_enable(struct ath5k_softc *sc)
{
	ATH5K_DBG(sc, ATH5K_DEBUG_ANY, "rfkill enable (gpio:%d polarity:%d)\n",
		sc->rf_kill.gpio, sc->rf_kill.polarity);
	ath5k_hw_set_gpio_output(sc->ah, sc->rf_kill.gpio);
	ath5k_hw_set_gpio(sc->ah, sc->rf_kill.gpio, sc->rf_kill.polarity);
}

static inline void ath5k_rfkill_set_intr(struct ath5k_softc *sc, bool enable)
{
	struct ath5k_hw *ah = sc->ah;
	u32 curval;

	ath5k_hw_set_gpio_input(ah, sc->rf_kill.gpio);
	curval = ath5k_hw_get_gpio(ah, sc->rf_kill.gpio);
	ath5k_hw_set_gpio_intr(ah, sc->rf_kill.gpio, enable ?
					!!curval : !curval);
}

static bool
ath5k_is_rfkill_set(struct ath5k_softc *sc)
{
	/* configuring GPIO for input for some reason disables rfkill */
	/*ath5k_hw_set_gpio_input(sc->ah, sc->rf_kill.gpio);*/
	return ath5k_hw_get_gpio(sc->ah, sc->rf_kill.gpio) ==
							sc->rf_kill.polarity;
}

static void
ath5k_tasklet_rfkill_toggle(unsigned long data)
{
	struct ath5k_softc *sc = (void *)data;
	bool blocked;

	blocked = ath5k_is_rfkill_set(sc);
	wiphy_rfkill_set_hw_state(sc->hw->wiphy, blocked);
}


void
ath5k_rfkill_hw_start(struct ath5k_hw *ah)
{
	struct ath5k_softc *sc = ah->ah_sc;

	/* read rfkill GPIO configuration from EEPROM header */
	sc->rf_kill.gpio = ah->ah_capabilities.cap_eeprom.ee_rfkill_pin;
	sc->rf_kill.polarity = ah->ah_capabilities.cap_eeprom.ee_rfkill_pol;

	tasklet_init(&sc->rf_kill.toggleq, ath5k_tasklet_rfkill_toggle,
		(unsigned long)sc);

	ath5k_rfkill_disable(sc);

	/* enable interrupt for rfkill switch */
	if (AR5K_EEPROM_HDR_RFKILL(ah->ah_capabilities.cap_eeprom.ee_header))
		ath5k_rfkill_set_intr(sc, true);
}


void
ath5k_rfkill_hw_stop(struct ath5k_hw *ah)
{
	struct ath5k_softc *sc = ah->ah_sc;

	/* disable interrupt for rfkill switch */
	if (AR5K_EEPROM_HDR_RFKILL(ah->ah_capabilities.cap_eeprom.ee_header))
		ath5k_rfkill_set_intr(sc, false);

	tasklet_kill(&sc->rf_kill.toggleq);

	/* enable RFKILL when stopping HW so Wifi LED is turned off */
	ath5k_rfkill_enable(sc);
}

