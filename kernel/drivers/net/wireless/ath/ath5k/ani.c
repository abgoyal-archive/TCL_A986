

#include "ath5k.h"
#include "base.h"
#include "reg.h"
#include "debug.h"
#include "ani.h"



/*** ANI parameter control ***/

void
ath5k_ani_set_noise_immunity_level(struct ath5k_hw *ah, int level)
{
	/* TODO:
	 * ANI documents suggest the following five levels to use, but the HAL
	 * and ath9k use only use the last two levels, making this
	 * essentially an on/off option. There *may* be a reason for this (???),
	 * so i stick with the HAL version for now...
	 */
#if 0
	const s8 hi[] = { -18, -18, -16, -14, -12 };
	const s8 lo[] = { -52, -56, -60, -64, -70 };
	const s8 sz[] = { -34, -41, -48, -55, -62 };
	const s8 fr[] = { -70, -72, -75, -78, -80 };
#else
	const s8 sz[] = { -55, -62 };
	const s8 lo[] = { -64, -70 };
	const s8 hi[] = { -14, -12 };
	const s8 fr[] = { -78, -80 };
#endif
	if (level < 0 || level >= ARRAY_SIZE(sz)) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
			"level out of range %d", level);
		return;
	}

	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_DESIRED_SIZE,
				AR5K_PHY_DESIRED_SIZE_TOT, sz[level]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_AGCCOARSE,
				AR5K_PHY_AGCCOARSE_LO, lo[level]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_AGCCOARSE,
				AR5K_PHY_AGCCOARSE_HI, hi[level]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_SIG,
				AR5K_PHY_SIG_FIRPWR, fr[level]);

	ah->ah_sc->ani_state.noise_imm_level = level;
	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "new level %d", level);
}


void
ath5k_ani_set_spur_immunity_level(struct ath5k_hw *ah, int level)
{
	const int val[] = { 2, 4, 6, 8, 10, 12, 14, 16 };

	if (level < 0 || level >= ARRAY_SIZE(val) ||
	    level > ah->ah_sc->ani_state.max_spur_level) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
			"level out of range %d", level);
		return;
	}

	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_OFDM_SELFCORR,
		AR5K_PHY_OFDM_SELFCORR_CYPWR_THR1, val[level]);

	ah->ah_sc->ani_state.spur_level = level;
	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "new level %d", level);
}


void
ath5k_ani_set_firstep_level(struct ath5k_hw *ah, int level)
{
	const int val[] = { 0, 4, 8 };

	if (level < 0 || level >= ARRAY_SIZE(val)) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
			"level out of range %d", level);
		return;
	}

	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_SIG,
				AR5K_PHY_SIG_FIRSTEP, val[level]);

	ah->ah_sc->ani_state.firstep_level = level;
	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "new level %d", level);
}


void
ath5k_ani_set_ofdm_weak_signal_detection(struct ath5k_hw *ah, bool on)
{
	const int m1l[] = { 127, 50 };
	const int m2l[] = { 127, 40 };
	const int m1[] = { 127, 0x4d };
	const int m2[] = { 127, 0x40 };
	const int m2cnt[] = { 31, 16 };
	const int m2lcnt[] = { 63, 48 };

	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_WEAK_OFDM_LOW_THR,
				AR5K_PHY_WEAK_OFDM_LOW_THR_M1, m1l[on]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_WEAK_OFDM_LOW_THR,
				AR5K_PHY_WEAK_OFDM_LOW_THR_M2, m2l[on]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_WEAK_OFDM_HIGH_THR,
				AR5K_PHY_WEAK_OFDM_HIGH_THR_M1, m1[on]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_WEAK_OFDM_HIGH_THR,
				AR5K_PHY_WEAK_OFDM_HIGH_THR_M2, m2[on]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_WEAK_OFDM_HIGH_THR,
			AR5K_PHY_WEAK_OFDM_HIGH_THR_M2_COUNT, m2cnt[on]);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_WEAK_OFDM_LOW_THR,
			AR5K_PHY_WEAK_OFDM_LOW_THR_M2_COUNT, m2lcnt[on]);

	if (on)
		AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_WEAK_OFDM_LOW_THR,
				AR5K_PHY_WEAK_OFDM_LOW_THR_SELFCOR_EN);
	else
		AR5K_REG_DISABLE_BITS(ah, AR5K_PHY_WEAK_OFDM_LOW_THR,
				AR5K_PHY_WEAK_OFDM_LOW_THR_SELFCOR_EN);

	ah->ah_sc->ani_state.ofdm_weak_sig = on;
	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "turned %s",
			  on ? "on" : "off");
}


void
ath5k_ani_set_cck_weak_signal_detection(struct ath5k_hw *ah, bool on)
{
	const int val[] = { 8, 6 };
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_CCK_CROSSCORR,
				AR5K_PHY_CCK_CROSSCORR_WEAK_SIG_THR, val[on]);
	ah->ah_sc->ani_state.cck_weak_sig = on;
	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "turned %s",
			  on ? "on" : "off");
}


/*** ANI algorithm ***/

static void
ath5k_ani_raise_immunity(struct ath5k_hw *ah, struct ath5k_ani_state *as,
			 bool ofdm_trigger)
{
	int rssi = ah->ah_beacon_rssi_avg.avg;

	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "raise immunity (%s)",
		ofdm_trigger ? "ODFM" : "CCK");

	/* first: raise noise immunity */
	if (as->noise_imm_level < ATH5K_ANI_MAX_NOISE_IMM_LVL) {
		ath5k_ani_set_noise_immunity_level(ah, as->noise_imm_level + 1);
		return;
	}

	/* only OFDM: raise spur immunity level */
	if (ofdm_trigger &&
	    as->spur_level < ah->ah_sc->ani_state.max_spur_level) {
		ath5k_ani_set_spur_immunity_level(ah, as->spur_level + 1);
		return;
	}

	/* AP mode */
	if (ah->ah_sc->opmode == NL80211_IFTYPE_AP) {
		if (as->firstep_level < ATH5K_ANI_MAX_FIRSTEP_LVL)
			ath5k_ani_set_firstep_level(ah, as->firstep_level + 1);
		return;
	}

	/* STA and IBSS mode */

	/* TODO: for IBSS mode it would be better to keep a beacon RSSI average
	 * per each neighbour node and use the minimum of these, to make sure we
	 * don't shut out a remote node by raising immunity too high. */

	if (rssi > ATH5K_ANI_RSSI_THR_HIGH) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
				  "beacon RSSI high");
		/* only OFDM: beacon RSSI is high, we can disable ODFM weak
		 * signal detection */
		if (ofdm_trigger && as->ofdm_weak_sig == true) {
			ath5k_ani_set_ofdm_weak_signal_detection(ah, false);
			ath5k_ani_set_spur_immunity_level(ah, 0);
			return;
		}
		/* as a last resort or CCK: raise firstep level */
		if (as->firstep_level < ATH5K_ANI_MAX_FIRSTEP_LVL) {
			ath5k_ani_set_firstep_level(ah, as->firstep_level + 1);
			return;
		}
	} else if (rssi > ATH5K_ANI_RSSI_THR_LOW) {
		/* beacon RSSI in mid range, we need OFDM weak signal detect,
		 * but can raise firstep level */
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
				  "beacon RSSI mid");
		if (ofdm_trigger && as->ofdm_weak_sig == false)
			ath5k_ani_set_ofdm_weak_signal_detection(ah, true);
		if (as->firstep_level < ATH5K_ANI_MAX_FIRSTEP_LVL)
			ath5k_ani_set_firstep_level(ah, as->firstep_level + 1);
		return;
	} else if (ah->ah_current_channel->band == IEEE80211_BAND_2GHZ) {
		/* beacon RSSI is low. in B/G mode turn of OFDM weak signal
		 * detect and zero firstep level to maximize CCK sensitivity */
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
				  "beacon RSSI low, 2GHz");
		if (ofdm_trigger && as->ofdm_weak_sig == true)
			ath5k_ani_set_ofdm_weak_signal_detection(ah, false);
		if (as->firstep_level > 0)
			ath5k_ani_set_firstep_level(ah, 0);
		return;
	}

	/* TODO: why not?:
	if (as->cck_weak_sig == true) {
		ath5k_ani_set_cck_weak_signal_detection(ah, false);
	}
	*/
}


static void
ath5k_ani_lower_immunity(struct ath5k_hw *ah, struct ath5k_ani_state *as)
{
	int rssi = ah->ah_beacon_rssi_avg.avg;

	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "lower immunity");

	if (ah->ah_sc->opmode == NL80211_IFTYPE_AP) {
		/* AP mode */
		if (as->firstep_level > 0) {
			ath5k_ani_set_firstep_level(ah, as->firstep_level - 1);
			return;
		}
	} else {
		/* STA and IBSS mode (see TODO above) */
		if (rssi > ATH5K_ANI_RSSI_THR_HIGH) {
			/* beacon signal is high, leave OFDM weak signal
			 * detection off or it may oscillate
			 * TODO: who said it's off??? */
		} else if (rssi > ATH5K_ANI_RSSI_THR_LOW) {
			/* beacon RSSI is mid-range: turn on ODFM weak signal
			 * detection and next, lower firstep level */
			if (as->ofdm_weak_sig == false) {
				ath5k_ani_set_ofdm_weak_signal_detection(ah,
									 true);
				return;
			}
			if (as->firstep_level > 0) {
				ath5k_ani_set_firstep_level(ah,
							as->firstep_level - 1);
				return;
			}
		} else {
			/* beacon signal is low: only reduce firstep level */
			if (as->firstep_level > 0) {
				ath5k_ani_set_firstep_level(ah,
							as->firstep_level - 1);
				return;
			}
		}
	}

	/* all modes */
	if (as->spur_level > 0) {
		ath5k_ani_set_spur_immunity_level(ah, as->spur_level - 1);
		return;
	}

	/* finally, reduce noise immunity */
	if (as->noise_imm_level > 0) {
		ath5k_ani_set_noise_immunity_level(ah, as->noise_imm_level - 1);
		return;
	}
}


static int
ath5k_hw_ani_get_listen_time(struct ath5k_hw *ah, struct ath5k_ani_state *as)
{
	int listen;

	/* freeze */
	ath5k_hw_reg_write(ah, AR5K_MIBC_FMC, AR5K_MIBC);
	/* read */
	as->pfc_cycles = ath5k_hw_reg_read(ah, AR5K_PROFCNT_CYCLE);
	as->pfc_busy = ath5k_hw_reg_read(ah, AR5K_PROFCNT_RXCLR);
	as->pfc_tx = ath5k_hw_reg_read(ah, AR5K_PROFCNT_TX);
	as->pfc_rx = ath5k_hw_reg_read(ah, AR5K_PROFCNT_RX);
	/* clear */
	ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_TX);
	ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_RX);
	ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_RXCLR);
	ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_CYCLE);
	/* un-freeze */
	ath5k_hw_reg_write(ah, 0, AR5K_MIBC);

	/* TODO: where does 44000 come from? (11g clock rate?) */
	listen = (as->pfc_cycles - as->pfc_rx - as->pfc_tx) / 44000;

	if (as->pfc_cycles == 0 || listen < 0)
		return 0;
	return listen;
}


static int
ath5k_ani_save_and_clear_phy_errors(struct ath5k_hw *ah,
				    struct ath5k_ani_state *as)
{
	unsigned int ofdm_err, cck_err;

	if (!ah->ah_capabilities.cap_has_phyerr_counters)
		return 0;

	ofdm_err = ath5k_hw_reg_read(ah, AR5K_PHYERR_CNT1);
	cck_err = ath5k_hw_reg_read(ah, AR5K_PHYERR_CNT2);

	/* reset counters first, we might be in a hurry (interrupt) */
	ath5k_hw_reg_write(ah, ATH5K_PHYERR_CNT_MAX - ATH5K_ANI_OFDM_TRIG_HIGH,
			   AR5K_PHYERR_CNT1);
	ath5k_hw_reg_write(ah, ATH5K_PHYERR_CNT_MAX - ATH5K_ANI_CCK_TRIG_HIGH,
			   AR5K_PHYERR_CNT2);

	ofdm_err = ATH5K_ANI_OFDM_TRIG_HIGH - (ATH5K_PHYERR_CNT_MAX - ofdm_err);
	cck_err = ATH5K_ANI_CCK_TRIG_HIGH - (ATH5K_PHYERR_CNT_MAX - cck_err);

	/* sometimes both can be zero, especially when there is a superfluous
	 * second interrupt. detect that here and return an error. */
	if (ofdm_err <= 0 && cck_err <= 0)
		return 0;

	/* avoid negative values should one of the registers overflow */
	if (ofdm_err > 0) {
		as->ofdm_errors += ofdm_err;
		as->sum_ofdm_errors += ofdm_err;
	}
	if (cck_err > 0) {
		as->cck_errors += cck_err;
		as->sum_cck_errors += cck_err;
	}
	return 1;
}


static void
ath5k_ani_period_restart(struct ath5k_hw *ah, struct ath5k_ani_state *as)
{
	/* keep last values for debugging */
	as->last_ofdm_errors = as->ofdm_errors;
	as->last_cck_errors = as->cck_errors;
	as->last_listen = as->listen_time;

	as->ofdm_errors = 0;
	as->cck_errors = 0;
	as->listen_time = 0;
}


void
ath5k_ani_calibration(struct ath5k_hw *ah)
{
	struct ath5k_ani_state *as = &ah->ah_sc->ani_state;
	int listen, ofdm_high, ofdm_low, cck_high, cck_low;

	if (as->ani_mode != ATH5K_ANI_MODE_AUTO)
		return;

	/* get listen time since last call and add it to the counter because we
	 * might not have restarted the "ani period" last time */
	listen = ath5k_hw_ani_get_listen_time(ah, as);
	as->listen_time += listen;

	ath5k_ani_save_and_clear_phy_errors(ah, as);

	ofdm_high = as->listen_time * ATH5K_ANI_OFDM_TRIG_HIGH / 1000;
	cck_high = as->listen_time * ATH5K_ANI_CCK_TRIG_HIGH / 1000;
	ofdm_low = as->listen_time * ATH5K_ANI_OFDM_TRIG_LOW / 1000;
	cck_low = as->listen_time * ATH5K_ANI_CCK_TRIG_LOW / 1000;

	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
		"listen %d (now %d)", as->listen_time, listen);
	ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
		"check high ofdm %d/%d cck %d/%d",
		as->ofdm_errors, ofdm_high, as->cck_errors, cck_high);

	if (as->ofdm_errors > ofdm_high || as->cck_errors > cck_high) {
		/* too many PHY errors - we have to raise immunity */
		bool ofdm_flag = as->ofdm_errors > ofdm_high ? true : false;
		ath5k_ani_raise_immunity(ah, as, ofdm_flag);
		ath5k_ani_period_restart(ah, as);

	} else if (as->listen_time > 5 * ATH5K_ANI_LISTEN_PERIOD) {
		/* If more than 5 (TODO: why 5?) periods have passed and we got
		 * relatively little errors we can try to lower immunity */
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
			"check low ofdm %d/%d cck %d/%d",
			as->ofdm_errors, ofdm_low, as->cck_errors, cck_low);

		if (as->ofdm_errors <= ofdm_low && as->cck_errors <= cck_low)
			ath5k_ani_lower_immunity(ah, as);

		ath5k_ani_period_restart(ah, as);
	}
}


/*** INTERRUPT HANDLER ***/

void
ath5k_ani_mib_intr(struct ath5k_hw *ah)
{
	struct ath5k_ani_state *as = &ah->ah_sc->ani_state;

	/* nothing to do here if HW does not have PHY error counters - they
	 * can't be the reason for the MIB interrupt then */
	if (!ah->ah_capabilities.cap_has_phyerr_counters)
		return;

	/* not in use but clear anyways */
	ath5k_hw_reg_write(ah, 0, AR5K_OFDM_FIL_CNT);
	ath5k_hw_reg_write(ah, 0, AR5K_CCK_FIL_CNT);

	if (ah->ah_sc->ani_state.ani_mode != ATH5K_ANI_MODE_AUTO)
		return;

	/* if one of the errors triggered, we can get a superfluous second
	 * interrupt, even though we have already reset the register. the
	 * function detects that so we can return early */
	if (ath5k_ani_save_and_clear_phy_errors(ah, as) == 0)
		return;

	if (as->ofdm_errors > ATH5K_ANI_OFDM_TRIG_HIGH ||
	    as->cck_errors > ATH5K_ANI_CCK_TRIG_HIGH)
		tasklet_schedule(&ah->ah_sc->ani_tasklet);
}


void
ath5k_ani_phy_error_report(struct ath5k_hw *ah,
			   enum ath5k_phy_error_code phyerr)
{
	struct ath5k_ani_state *as = &ah->ah_sc->ani_state;

	if (phyerr == AR5K_RX_PHY_ERROR_OFDM_TIMING) {
		as->ofdm_errors++;
		if (as->ofdm_errors > ATH5K_ANI_OFDM_TRIG_HIGH)
			tasklet_schedule(&ah->ah_sc->ani_tasklet);
	} else if (phyerr == AR5K_RX_PHY_ERROR_CCK_TIMING) {
		as->cck_errors++;
		if (as->cck_errors > ATH5K_ANI_CCK_TRIG_HIGH)
			tasklet_schedule(&ah->ah_sc->ani_tasklet);
	}
}


/*** INIT ***/

static void
ath5k_enable_phy_err_counters(struct ath5k_hw *ah)
{
	ath5k_hw_reg_write(ah, ATH5K_PHYERR_CNT_MAX - ATH5K_ANI_OFDM_TRIG_HIGH,
			   AR5K_PHYERR_CNT1);
	ath5k_hw_reg_write(ah, ATH5K_PHYERR_CNT_MAX - ATH5K_ANI_CCK_TRIG_HIGH,
			   AR5K_PHYERR_CNT2);
	ath5k_hw_reg_write(ah, AR5K_PHY_ERR_FIL_OFDM, AR5K_PHYERR_CNT1_MASK);
	ath5k_hw_reg_write(ah, AR5K_PHY_ERR_FIL_CCK, AR5K_PHYERR_CNT2_MASK);

	/* not in use */
	ath5k_hw_reg_write(ah, 0, AR5K_OFDM_FIL_CNT);
	ath5k_hw_reg_write(ah, 0, AR5K_CCK_FIL_CNT);
}


static void
ath5k_disable_phy_err_counters(struct ath5k_hw *ah)
{
	ath5k_hw_reg_write(ah, 0, AR5K_PHYERR_CNT1);
	ath5k_hw_reg_write(ah, 0, AR5K_PHYERR_CNT2);
	ath5k_hw_reg_write(ah, 0, AR5K_PHYERR_CNT1_MASK);
	ath5k_hw_reg_write(ah, 0, AR5K_PHYERR_CNT2_MASK);

	/* not in use */
	ath5k_hw_reg_write(ah, 0, AR5K_OFDM_FIL_CNT);
	ath5k_hw_reg_write(ah, 0, AR5K_CCK_FIL_CNT);
}


void
ath5k_ani_init(struct ath5k_hw *ah, enum ath5k_ani_mode mode)
{
	/* ANI is only possible on 5212 and newer */
	if (ah->ah_version < AR5K_AR5212)
		return;

	/* clear old state information */
	memset(&ah->ah_sc->ani_state, 0, sizeof(ah->ah_sc->ani_state));

	/* older hardware has more spur levels than newer */
	if (ah->ah_mac_srev < AR5K_SREV_AR2414)
		ah->ah_sc->ani_state.max_spur_level = 7;
	else
		ah->ah_sc->ani_state.max_spur_level = 2;

	/* initial values for our ani parameters */
	if (mode == ATH5K_ANI_MODE_OFF) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "ANI off\n");
	} else if  (mode == ATH5K_ANI_MODE_MANUAL_LOW) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
			"ANI manual low -> high sensitivity\n");
		ath5k_ani_set_noise_immunity_level(ah, 0);
		ath5k_ani_set_spur_immunity_level(ah, 0);
		ath5k_ani_set_firstep_level(ah, 0);
		ath5k_ani_set_ofdm_weak_signal_detection(ah, true);
		ath5k_ani_set_cck_weak_signal_detection(ah, true);
	} else if (mode == ATH5K_ANI_MODE_MANUAL_HIGH) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI,
			"ANI manual high -> low sensitivity\n");
		ath5k_ani_set_noise_immunity_level(ah,
					ATH5K_ANI_MAX_NOISE_IMM_LVL);
		ath5k_ani_set_spur_immunity_level(ah,
					ah->ah_sc->ani_state.max_spur_level);
		ath5k_ani_set_firstep_level(ah, ATH5K_ANI_MAX_FIRSTEP_LVL);
		ath5k_ani_set_ofdm_weak_signal_detection(ah, false);
		ath5k_ani_set_cck_weak_signal_detection(ah, false);
	} else if (mode == ATH5K_ANI_MODE_AUTO) {
		ATH5K_DBG_UNLIMIT(ah->ah_sc, ATH5K_DEBUG_ANI, "ANI auto\n");
		ath5k_ani_set_noise_immunity_level(ah, 0);
		ath5k_ani_set_spur_immunity_level(ah, 0);
		ath5k_ani_set_firstep_level(ah, 0);
		ath5k_ani_set_ofdm_weak_signal_detection(ah, true);
		ath5k_ani_set_cck_weak_signal_detection(ah, false);
	}

	/* newer hardware has PHY error counter registers which we can use to
	 * get OFDM and CCK error counts. older hardware has to set rxfilter and
	 * report every single PHY error by calling ath5k_ani_phy_error_report()
	 */
	if (mode == ATH5K_ANI_MODE_AUTO) {
		if (ah->ah_capabilities.cap_has_phyerr_counters)
			ath5k_enable_phy_err_counters(ah);
		else
			ath5k_hw_set_rx_filter(ah, ath5k_hw_get_rx_filter(ah) |
						   AR5K_RX_FILTER_PHYERR);
	} else {
		if (ah->ah_capabilities.cap_has_phyerr_counters)
			ath5k_disable_phy_err_counters(ah);
		else
			ath5k_hw_set_rx_filter(ah, ath5k_hw_get_rx_filter(ah) &
						   ~AR5K_RX_FILTER_PHYERR);
	}

	ah->ah_sc->ani_state.ani_mode = mode;
}


/*** DEBUG ***/

#ifdef CONFIG_ATH5K_DEBUG

void
ath5k_ani_print_counters(struct ath5k_hw *ah)
{
	/* clears too */
	printk(KERN_NOTICE "ACK fail\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_ACK_FAIL));
	printk(KERN_NOTICE "RTS fail\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_RTS_FAIL));
	printk(KERN_NOTICE "RTS success\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_RTS_OK));
	printk(KERN_NOTICE "FCS error\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_FCS_FAIL));

	/* no clear */
	printk(KERN_NOTICE "tx\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_PROFCNT_TX));
	printk(KERN_NOTICE "rx\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_PROFCNT_RX));
	printk(KERN_NOTICE "busy\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_PROFCNT_RXCLR));
	printk(KERN_NOTICE "cycles\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_PROFCNT_CYCLE));

	printk(KERN_NOTICE "AR5K_PHYERR_CNT1\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_PHYERR_CNT1));
	printk(KERN_NOTICE "AR5K_PHYERR_CNT2\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_PHYERR_CNT2));
	printk(KERN_NOTICE "AR5K_OFDM_FIL_CNT\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_OFDM_FIL_CNT));
	printk(KERN_NOTICE "AR5K_CCK_FIL_CNT\t%d\n",
		ath5k_hw_reg_read(ah, AR5K_CCK_FIL_CNT));
}

#endif
