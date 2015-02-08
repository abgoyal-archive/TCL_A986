


#include "ixgbe.h"
#include "ixgbe_type.h"
#include "ixgbe_dcb.h"
#include "ixgbe_dcb_82598.h"
#include "ixgbe_dcb_82599.h"

s32 ixgbe_dcb_check_config(struct ixgbe_dcb_config *dcb_config)
{
	struct tc_bw_alloc *p;
	s32 ret_val = 0;
	u8 i, j, bw = 0, bw_id;
	u8 bw_sum[2][MAX_BW_GROUP];
	bool link_strict[2][MAX_BW_GROUP];

	memset(bw_sum, 0, sizeof(bw_sum));
	memset(link_strict, 0, sizeof(link_strict));

	/* First Tx, then Rx */
	for (i = 0; i < 2; i++) {
		/* Check each traffic class for rule violation */
		for (j = 0; j < MAX_TRAFFIC_CLASS; j++) {
			p = &dcb_config->tc_config[j].path[i];

			bw = p->bwg_percent;
			bw_id = p->bwg_id;

			if (bw_id >= MAX_BW_GROUP) {
				ret_val = DCB_ERR_CONFIG;
				goto err_config;
			}
			if (p->prio_type == prio_link) {
				link_strict[i][bw_id] = true;
				/* Link strict should have zero bandwidth */
				if (bw) {
					ret_val = DCB_ERR_LS_BW_NONZERO;
					goto err_config;
				}
			} else if (!bw) {
				/*
				 * Traffic classes without link strict
				 * should have non-zero bandwidth.
				 */
				ret_val = DCB_ERR_TC_BW_ZERO;
				goto err_config;
			}
			bw_sum[i][bw_id] += bw;
		}

		bw = 0;

		/* Check each bandwidth group for rule violation */
		for (j = 0; j < MAX_BW_GROUP; j++) {
			bw += dcb_config->bw_percentage[i][j];
			/*
			 * Sum of bandwidth percentages of all traffic classes
			 * within a Bandwidth Group must total 100 except for
			 * link strict group (zero bandwidth).
			 */
			if (link_strict[i][j]) {
				if (bw_sum[i][j]) {
					/*
					 * Link strict group should have zero
					 * bandwidth.
					 */
					ret_val = DCB_ERR_LS_BWG_NONZERO;
					goto err_config;
				}
			} else if (bw_sum[i][j] != BW_PERCENT &&
				   bw_sum[i][j] != 0) {
				ret_val = DCB_ERR_TC_BW;
				goto err_config;
			}
		}

		if (bw != BW_PERCENT) {
			ret_val = DCB_ERR_BW_GROUP;
			goto err_config;
		}
	}

err_config:
	return ret_val;
}

s32 ixgbe_dcb_calculate_tc_credits(struct ixgbe_dcb_config *dcb_config,
                                   u8 direction)
{
	struct tc_bw_alloc *p;
	s32 ret_val = 0;
	/* Initialization values default for Tx settings */
	u32 credit_refill       = 0;
	u32 credit_max          = 0;
	u16 link_percentage     = 0;
	u8  bw_percent          = 0;
	u8  i;

	if (dcb_config == NULL) {
		ret_val = DCB_ERR_CONFIG;
		goto out;
	}

	/* Find out the link percentage for each TC first */
	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		p = &dcb_config->tc_config[i].path[direction];
		bw_percent = dcb_config->bw_percentage[direction][p->bwg_id];

		link_percentage = p->bwg_percent;
		/* Must be careful of integer division for very small nums */
		link_percentage = (link_percentage * bw_percent) / 100;
		if (p->bwg_percent > 0 && link_percentage == 0)
			link_percentage = 1;

		/* Save link_percentage for reference */
		p->link_percent = (u8)link_percentage;

		/* Calculate credit refill and save it */
		credit_refill = link_percentage * MINIMUM_CREDIT_REFILL;
		p->data_credits_refill = (u16)credit_refill;

		/* Calculate maximum credit for the TC */
		credit_max = (link_percentage * MAX_CREDIT) / 100;

		/*
		 * Adjustment based on rule checking, if the percentage
		 * of a TC is too small, the maximum credit may not be
		 * enough to send out a jumbo frame in data plane arbitration.
		 */
		if (credit_max && (credit_max < MINIMUM_CREDIT_FOR_JUMBO))
			credit_max = MINIMUM_CREDIT_FOR_JUMBO;

		if (direction == DCB_TX_CONFIG) {
			/*
			 * Adjustment based on rule checking, if the
			 * percentage of a TC is too small, the maximum
			 * credit may not be enough to send out a TSO
			 * packet in descriptor plane arbitration.
			 */
			if (credit_max &&
			    (credit_max < MINIMUM_CREDIT_FOR_TSO))
				credit_max = MINIMUM_CREDIT_FOR_TSO;

			dcb_config->tc_config[i].desc_credits_max =
				(u16)credit_max;
		}

		p->data_credits_max = (u16)credit_max;
	}

out:
	return ret_val;
}

s32 ixgbe_dcb_get_tc_stats(struct ixgbe_hw *hw, struct ixgbe_hw_stats *stats,
                           u8 tc_count)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_get_tc_stats_82598(hw, stats, tc_count);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_get_tc_stats_82599(hw, stats, tc_count);
	return ret;
}

s32 ixgbe_dcb_get_pfc_stats(struct ixgbe_hw *hw, struct ixgbe_hw_stats *stats,
                            u8 tc_count)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_get_pfc_stats_82598(hw, stats, tc_count);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_get_pfc_stats_82599(hw, stats, tc_count);
	return ret;
}

s32 ixgbe_dcb_config_rx_arbiter(struct ixgbe_hw *hw,
                                struct ixgbe_dcb_config *dcb_config)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_config_rx_arbiter_82598(hw, dcb_config);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_config_rx_arbiter_82599(hw, dcb_config);
	return ret;
}

s32 ixgbe_dcb_config_tx_desc_arbiter(struct ixgbe_hw *hw,
                                     struct ixgbe_dcb_config *dcb_config)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_config_tx_desc_arbiter_82598(hw, dcb_config);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_config_tx_desc_arbiter_82599(hw, dcb_config);
	return ret;
}

s32 ixgbe_dcb_config_tx_data_arbiter(struct ixgbe_hw *hw,
                                     struct ixgbe_dcb_config *dcb_config)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_config_tx_data_arbiter_82598(hw, dcb_config);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_config_tx_data_arbiter_82599(hw, dcb_config);
	return ret;
}

s32 ixgbe_dcb_config_pfc(struct ixgbe_hw *hw,
                         struct ixgbe_dcb_config *dcb_config)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_config_pfc_82598(hw, dcb_config);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_config_pfc_82599(hw, dcb_config);
	return ret;
}

s32 ixgbe_dcb_config_tc_stats(struct ixgbe_hw *hw)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_config_tc_stats_82598(hw);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_config_tc_stats_82599(hw);
	return ret;
}

s32 ixgbe_dcb_hw_config(struct ixgbe_hw *hw,
                        struct ixgbe_dcb_config *dcb_config)
{
	s32 ret = 0;
	if (hw->mac.type == ixgbe_mac_82598EB)
		ret = ixgbe_dcb_hw_config_82598(hw, dcb_config);
	else if (hw->mac.type == ixgbe_mac_82599EB)
		ret = ixgbe_dcb_hw_config_82599(hw, dcb_config);
	return ret;
}

