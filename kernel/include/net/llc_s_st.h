
#ifndef LLC_S_ST_H
#define LLC_S_ST_H

#define LLC_NR_SAP_STATES	2       /* size of state table */

/* structures and types */
/* SAP state table structure */
struct llc_sap_state_trans {
	llc_sap_ev_t	  ev;
	u8		  next_state;
	llc_sap_action_t *ev_actions;
};

struct llc_sap_state {
	u8			   curr_state;
	struct llc_sap_state_trans **transitions;
};

/* only access to SAP state table */
extern struct llc_sap_state llc_sap_state_table[LLC_NR_SAP_STATES];
#endif /* LLC_S_ST_H */
