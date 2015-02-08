
#ifndef __iwl_power_setting_h__
#define __iwl_power_setting_h__

#include "iwl-commands.h"

#define IWL_ABSOLUTE_ZERO		0
#define IWL_ABSOLUTE_MAX		0xFFFFFFFF
#define IWL_TT_INCREASE_MARGIN	5
#define IWL_TT_CT_KILL_MARGIN	3

enum iwl_antenna_ok {
	IWL_ANT_OK_NONE,
	IWL_ANT_OK_SINGLE,
	IWL_ANT_OK_MULTI,
};

/* Thermal Throttling State Machine states */
enum  iwl_tt_state {
	IWL_TI_0,	/* normal temperature, system power state */
	IWL_TI_1,	/* high temperature detect, low power state */
	IWL_TI_2,	/* higher temperature detected, lower power state */
	IWL_TI_CT_KILL, /* critical temperature detected, lowest power state */
	IWL_TI_STATE_MAX
};

struct iwl_tt_restriction {
	enum iwl_antenna_ok tx_stream;
	enum iwl_antenna_ok rx_stream;
	bool is_ht;
};

struct iwl_tt_trans {
	enum iwl_tt_state next_state;
	u32 tt_low;
	u32 tt_high;
};

struct iwl_tt_mgmt {
	enum iwl_tt_state state;
	bool advanced_tt;
	u8 tt_power_mode;
	bool ct_kill_toggle;
#ifdef CONFIG_IWLWIFI_DEBUG
	s32 tt_previous_temp;
#endif
	struct iwl_tt_restriction *restriction;
	struct iwl_tt_trans *transaction;
	struct timer_list ct_kill_exit_tm;
	struct timer_list ct_kill_waiting_tm;
};

enum iwl_power_level {
	IWL_POWER_INDEX_1,
	IWL_POWER_INDEX_2,
	IWL_POWER_INDEX_3,
	IWL_POWER_INDEX_4,
	IWL_POWER_INDEX_5,
	IWL_POWER_NUM
};

struct iwl_power_mgr {
	struct iwl_powertable_cmd sleep_cmd;
	int debug_sleep_level_override;
	bool pci_pm;
};

int iwl_power_update_mode(struct iwl_priv *priv, bool force);
bool iwl_ht_enabled(struct iwl_priv *priv);
bool iwl_within_ct_kill_margin(struct iwl_priv *priv);
enum iwl_antenna_ok iwl_tx_ant_restriction(struct iwl_priv *priv);
enum iwl_antenna_ok iwl_rx_ant_restriction(struct iwl_priv *priv);
void iwl_tt_enter_ct_kill(struct iwl_priv *priv);
void iwl_tt_exit_ct_kill(struct iwl_priv *priv);
void iwl_tt_handler(struct iwl_priv *priv);
void iwl_tt_initialize(struct iwl_priv *priv);
void iwl_tt_exit(struct iwl_priv *priv);
void iwl_power_initialize(struct iwl_priv *priv);

extern bool no_sleep_autoadjust;

#endif  /* __iwl_power_setting_h__ */
