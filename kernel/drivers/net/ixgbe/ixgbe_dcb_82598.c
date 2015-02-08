

#include "ixgbe.h"
#include "ixgbe_type.h"
#include "ixgbe_dcb.h"
#include "ixgbe_dcb_82598.h"

s32 ixgbe_dcb_get_tc_stats_82598(struct ixgbe_hw *hw,
                                 struct ixgbe_hw_stats *stats,
                                 u8 tc_count)
{
	int tc;

	if (tc_count > MAX_TRAFFIC_CLASS)
		return DCB_ERR_PARAM;

	/* Statistics pertaining to each traffic class */
	for (tc = 0; tc < tc_count; tc++) {
		/* Transmitted Packets */
		stats->qptc[tc] += IXGBE_READ_REG(hw, IXGBE_QPTC(tc));
		/* Transmitted Bytes */
		stats->qbtc[tc] += IXGBE_READ_REG(hw, IXGBE_QBTC(tc));
		/* Received Packets */
		stats->qprc[tc] += IXGBE_READ_REG(hw, IXGBE_QPRC(tc));
		/* Received Bytes */
		stats->qbrc[tc] += IXGBE_READ_REG(hw, IXGBE_QBRC(tc));
	}

	return 0;
}

s32 ixgbe_dcb_get_pfc_stats_82598(struct ixgbe_hw *hw,
                                  struct ixgbe_hw_stats *stats,
                                  u8 tc_count)
{
	int tc;

	if (tc_count > MAX_TRAFFIC_CLASS)
		return DCB_ERR_PARAM;

	for (tc = 0; tc < tc_count; tc++) {
		/* Priority XOFF Transmitted */
		stats->pxofftxc[tc] += IXGBE_READ_REG(hw, IXGBE_PXOFFTXC(tc));
		/* Priority XOFF Received */
		stats->pxoffrxc[tc] += IXGBE_READ_REG(hw, IXGBE_PXOFFRXC(tc));
	}

	return 0;
}

static s32 ixgbe_dcb_config_packet_buffers_82598(struct ixgbe_hw *hw,
						 struct ixgbe_dcb_config *dcb_config)
{
	s32 ret_val = 0;
	u32 value = IXGBE_RXPBSIZE_64KB;
	u8  i = 0;

	/* Setup Rx packet buffer sizes */
	switch (dcb_config->rx_pba_cfg) {
	case pba_80_48:
		/* Setup the first four at 80KB */
		value = IXGBE_RXPBSIZE_80KB;
		for (; i < 4; i++)
			IXGBE_WRITE_REG(hw, IXGBE_RXPBSIZE(i), value);
		/* Setup the last four at 48KB...don't re-init i */
		value = IXGBE_RXPBSIZE_48KB;
		/* Fall Through */
	case pba_equal:
	default:
		for (; i < IXGBE_MAX_PACKET_BUFFERS; i++)
			IXGBE_WRITE_REG(hw, IXGBE_RXPBSIZE(i), value);

		/* Setup Tx packet buffer sizes */
		for (i = 0; i < IXGBE_MAX_PACKET_BUFFERS; i++) {
			IXGBE_WRITE_REG(hw, IXGBE_TXPBSIZE(i),
					IXGBE_TXPBSIZE_40KB);
		}
		break;
	}

	return ret_val;
}

s32 ixgbe_dcb_config_rx_arbiter_82598(struct ixgbe_hw *hw,
                                      struct ixgbe_dcb_config *dcb_config)
{
	struct tc_bw_alloc    *p;
	u32    reg           = 0;
	u32    credit_refill = 0;
	u32    credit_max    = 0;
	u8     i             = 0;

	reg = IXGBE_READ_REG(hw, IXGBE_RUPPBMR) | IXGBE_RUPPBMR_MQA;
	IXGBE_WRITE_REG(hw, IXGBE_RUPPBMR, reg);

	reg = IXGBE_READ_REG(hw, IXGBE_RMCS);
	/* Enable Arbiter */
	reg &= ~IXGBE_RMCS_ARBDIS;
	/* Enable Receive Recycle within the BWG */
	reg |= IXGBE_RMCS_RRM;
	/* Enable Deficit Fixed Priority arbitration*/
	reg |= IXGBE_RMCS_DFP;

	IXGBE_WRITE_REG(hw, IXGBE_RMCS, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		p = &dcb_config->tc_config[i].path[DCB_RX_CONFIG];
		credit_refill = p->data_credits_refill;
		credit_max    = p->data_credits_max;

		reg = credit_refill | (credit_max << IXGBE_RT2CR_MCL_SHIFT);

		if (p->prio_type == prio_link)
			reg |= IXGBE_RT2CR_LSP;

		IXGBE_WRITE_REG(hw, IXGBE_RT2CR(i), reg);
	}

	reg = IXGBE_READ_REG(hw, IXGBE_RDRXCTL);
	reg |= IXGBE_RDRXCTL_RDMTS_1_2;
	reg |= IXGBE_RDRXCTL_MPBEN;
	reg |= IXGBE_RDRXCTL_MCEN;
	IXGBE_WRITE_REG(hw, IXGBE_RDRXCTL, reg);

	reg = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	/* Make sure there is enough descriptors before arbitration */
	reg &= ~IXGBE_RXCTRL_DMBYPS;
	IXGBE_WRITE_REG(hw, IXGBE_RXCTRL, reg);

	return 0;
}

s32 ixgbe_dcb_config_tx_desc_arbiter_82598(struct ixgbe_hw *hw,
                                           struct ixgbe_dcb_config *dcb_config)
{
	struct tc_bw_alloc *p;
	u32    reg, max_credits;
	u8     i;

	reg = IXGBE_READ_REG(hw, IXGBE_DPMCS);

	/* Enable arbiter */
	reg &= ~IXGBE_DPMCS_ARBDIS;
	if (!(dcb_config->round_robin_enable)) {
		/* Enable DFP and Recycle mode */
		reg |= (IXGBE_DPMCS_TDPAC | IXGBE_DPMCS_TRM);
	}
	reg |= IXGBE_DPMCS_TSOEF;
	/* Configure Max TSO packet size 34KB including payload and headers */
	reg |= (0x4 << IXGBE_DPMCS_MTSOS_SHIFT);

	IXGBE_WRITE_REG(hw, IXGBE_DPMCS, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		p = &dcb_config->tc_config[i].path[DCB_TX_CONFIG];
		max_credits = dcb_config->tc_config[i].desc_credits_max;
		reg = max_credits << IXGBE_TDTQ2TCCR_MCL_SHIFT;
		reg |= p->data_credits_refill;
		reg |= (u32)(p->bwg_id) << IXGBE_TDTQ2TCCR_BWG_SHIFT;

		if (p->prio_type == prio_group)
			reg |= IXGBE_TDTQ2TCCR_GSP;

		if (p->prio_type == prio_link)
			reg |= IXGBE_TDTQ2TCCR_LSP;

		IXGBE_WRITE_REG(hw, IXGBE_TDTQ2TCCR(i), reg);
	}

	return 0;
}

s32 ixgbe_dcb_config_tx_data_arbiter_82598(struct ixgbe_hw *hw,
                                           struct ixgbe_dcb_config *dcb_config)
{
	struct tc_bw_alloc *p;
	u32 reg;
	u8 i;

	reg = IXGBE_READ_REG(hw, IXGBE_PDPMCS);
	/* Enable Data Plane Arbiter */
	reg &= ~IXGBE_PDPMCS_ARBDIS;
	/* Enable DFP and Transmit Recycle Mode */
	reg |= (IXGBE_PDPMCS_TPPAC | IXGBE_PDPMCS_TRM);

	IXGBE_WRITE_REG(hw, IXGBE_PDPMCS, reg);

	/* Configure traffic class credits and priority */
	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		p = &dcb_config->tc_config[i].path[DCB_TX_CONFIG];
		reg = p->data_credits_refill;
		reg |= (u32)(p->data_credits_max) << IXGBE_TDPT2TCCR_MCL_SHIFT;
		reg |= (u32)(p->bwg_id) << IXGBE_TDPT2TCCR_BWG_SHIFT;

		if (p->prio_type == prio_group)
			reg |= IXGBE_TDPT2TCCR_GSP;

		if (p->prio_type == prio_link)
			reg |= IXGBE_TDPT2TCCR_LSP;

		IXGBE_WRITE_REG(hw, IXGBE_TDPT2TCCR(i), reg);
	}

	/* Enable Tx packet buffer division */
	reg = IXGBE_READ_REG(hw, IXGBE_DTXCTL);
	reg |= IXGBE_DTXCTL_ENDBUBD;
	IXGBE_WRITE_REG(hw, IXGBE_DTXCTL, reg);

	return 0;
}

s32 ixgbe_dcb_config_pfc_82598(struct ixgbe_hw *hw,
                               struct ixgbe_dcb_config *dcb_config)
{
	u32 reg, rx_pba_size;
	u8  i;

	if (!dcb_config->pfc_mode_enable)
		goto out;

	/* Enable Transmit Priority Flow Control */
	reg = IXGBE_READ_REG(hw, IXGBE_RMCS);
	reg &= ~IXGBE_RMCS_TFCE_802_3X;
	/* correct the reporting of our flow control status */
	reg |= IXGBE_RMCS_TFCE_PRIORITY;
	IXGBE_WRITE_REG(hw, IXGBE_RMCS, reg);

	/* Enable Receive Priority Flow Control */
	reg = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	reg &= ~IXGBE_FCTRL_RFCE;
	reg |= IXGBE_FCTRL_RPFCE;
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, reg);

	/*
	 * Configure flow control thresholds and enable priority flow control
	 * for each traffic class.
	 */
	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		if (dcb_config->rx_pba_cfg == pba_equal) {
			rx_pba_size = IXGBE_RXPBSIZE_64KB;
		} else {
			rx_pba_size = (i < 4) ? IXGBE_RXPBSIZE_80KB
					      : IXGBE_RXPBSIZE_48KB;
		}

		reg = ((rx_pba_size >> 5) &  0xFFF0);
		if (dcb_config->tc_config[i].dcb_pfc == pfc_enabled_tx ||
		    dcb_config->tc_config[i].dcb_pfc == pfc_enabled_full)
			reg |= IXGBE_FCRTL_XONE;

		IXGBE_WRITE_REG(hw, IXGBE_FCRTL(i), reg);

		reg = ((rx_pba_size >> 2) & 0xFFF0);
		if (dcb_config->tc_config[i].dcb_pfc == pfc_enabled_tx ||
		    dcb_config->tc_config[i].dcb_pfc == pfc_enabled_full)
			reg |= IXGBE_FCRTH_FCEN;

		IXGBE_WRITE_REG(hw, IXGBE_FCRTH(i), reg);
	}

	/* Configure pause time */
	for (i = 0; i < (MAX_TRAFFIC_CLASS >> 1); i++)
		IXGBE_WRITE_REG(hw, IXGBE_FCTTV(i), 0x68006800);

	/* Configure flow control refresh threshold value */
	IXGBE_WRITE_REG(hw, IXGBE_FCRTV, 0x3400);

out:
	return 0;
}

s32 ixgbe_dcb_config_tc_stats_82598(struct ixgbe_hw *hw)
{
	u32 reg = 0;
	u8  i   = 0;
	u8  j   = 0;

	/* Receive Queues stats setting -  8 queues per statistics reg */
	for (i = 0, j = 0; i < 15 && j < 8; i = i + 2, j++) {
		reg = IXGBE_READ_REG(hw, IXGBE_RQSMR(i));
		reg |= ((0x1010101) * j);
		IXGBE_WRITE_REG(hw, IXGBE_RQSMR(i), reg);
		reg = IXGBE_READ_REG(hw, IXGBE_RQSMR(i + 1));
		reg |= ((0x1010101) * j);
		IXGBE_WRITE_REG(hw, IXGBE_RQSMR(i + 1), reg);
	}
	/* Transmit Queues stats setting -  4 queues per statistics reg */
	for (i = 0; i < 8; i++) {
		reg = IXGBE_READ_REG(hw, IXGBE_TQSMR(i));
		reg |= ((0x1010101) * i);
		IXGBE_WRITE_REG(hw, IXGBE_TQSMR(i), reg);
	}

	return 0;
}

s32 ixgbe_dcb_hw_config_82598(struct ixgbe_hw *hw,
                              struct ixgbe_dcb_config *dcb_config)
{
	ixgbe_dcb_config_packet_buffers_82598(hw, dcb_config);
	ixgbe_dcb_config_rx_arbiter_82598(hw, dcb_config);
	ixgbe_dcb_config_tx_desc_arbiter_82598(hw, dcb_config);
	ixgbe_dcb_config_tx_data_arbiter_82598(hw, dcb_config);
	ixgbe_dcb_config_pfc_82598(hw, dcb_config);
	ixgbe_dcb_config_tc_stats_82598(hw);

	return 0;
}
