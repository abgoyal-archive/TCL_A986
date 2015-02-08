

#ifndef __iwl_4965_hw_h__
#define __iwl_4965_hw_h__

#include "iwl-fh.h"

/* EEPROM */
#define IWL4965_EEPROM_IMG_SIZE			1024

#define IWL49_FIRST_AMPDU_QUEUE	7

#define IWL49_RTC_INST_LOWER_BOUND		(0x000000)
#define IWL49_RTC_INST_UPPER_BOUND		(0x018000)

#define IWL49_RTC_DATA_LOWER_BOUND		(0x800000)
#define IWL49_RTC_DATA_UPPER_BOUND		(0x80A000)

#define IWL49_RTC_INST_SIZE  (IWL49_RTC_INST_UPPER_BOUND - \
				IWL49_RTC_INST_LOWER_BOUND)
#define IWL49_RTC_DATA_SIZE  (IWL49_RTC_DATA_UPPER_BOUND - \
				IWL49_RTC_DATA_LOWER_BOUND)

#define IWL49_MAX_INST_SIZE IWL49_RTC_INST_SIZE
#define IWL49_MAX_DATA_SIZE IWL49_RTC_DATA_SIZE

/* Size of uCode instruction memory in bootstrap state machine */
#define IWL49_MAX_BSM_SIZE BSM_SRAM_SIZE

static inline int iwl4965_hw_valid_rtc_data_addr(u32 addr)
{
	return (addr >= IWL49_RTC_DATA_LOWER_BOUND) &&
	       (addr < IWL49_RTC_DATA_UPPER_BOUND);
}

/********************* START TEMPERATURE *************************************/

#define TEMPERATURE_CALIB_KELVIN_OFFSET 8
#define TEMPERATURE_CALIB_A_VAL 259

/* Limit range of calculated temperature to be between these Kelvin values */
#define IWL_TX_POWER_TEMPERATURE_MIN  (263)
#define IWL_TX_POWER_TEMPERATURE_MAX  (410)

#define IWL_TX_POWER_TEMPERATURE_OUT_OF_RANGE(t) \
	(((t) < IWL_TX_POWER_TEMPERATURE_MIN) || \
	 ((t) > IWL_TX_POWER_TEMPERATURE_MAX))

/********************* END TEMPERATURE ***************************************/

/********************* START TXPOWER *****************************************/



#define IWL_TX_POWER_MIMO_REGULATORY_COMPENSATION (6)

#define IWL_TX_POWER_CCK_COMPENSATION_B_STEP (9)
#define IWL_TX_POWER_CCK_COMPENSATION_C_STEP (5)

#define TX_POWER_IWL_VOLTAGE_CODES_PER_03V   (7)

#define MIN_TX_GAIN_INDEX		(0)  /* highest gain, lowest idx, 2.4 */
#define MIN_TX_GAIN_INDEX_52GHZ_EXT	(-9) /* highest gain, lowest idx, 5 */




#define IWL_TX_POWER_DEFAULT_REGULATORY_24   (34)
#define IWL_TX_POWER_DEFAULT_REGULATORY_52   (34)
#define IWL_TX_POWER_REGULATORY_MIN          (0)
#define IWL_TX_POWER_REGULATORY_MAX          (34)

#define IWL_TX_POWER_DEFAULT_SATURATION_24   (38)
#define IWL_TX_POWER_DEFAULT_SATURATION_52   (38)
#define IWL_TX_POWER_SATURATION_MIN          (20)
#define IWL_TX_POWER_SATURATION_MAX          (50)

/* Group 0, 5.2 GHz ch 34-43:  4.5 degrees per 1/2 dB. */
#define CALIB_IWL_TX_ATTEN_GR1_FCH 34
#define CALIB_IWL_TX_ATTEN_GR1_LCH 43

/* Group 1, 5.3 GHz ch 44-70:  4.0 degrees per 1/2 dB. */
#define CALIB_IWL_TX_ATTEN_GR2_FCH 44
#define CALIB_IWL_TX_ATTEN_GR2_LCH 70

/* Group 2, 5.5 GHz ch 71-124:  4.0 degrees per 1/2 dB. */
#define CALIB_IWL_TX_ATTEN_GR3_FCH 71
#define CALIB_IWL_TX_ATTEN_GR3_LCH 124

/* Group 3, 5.7 GHz ch 125-200:  4.0 degrees per 1/2 dB. */
#define CALIB_IWL_TX_ATTEN_GR4_FCH 125
#define CALIB_IWL_TX_ATTEN_GR4_LCH 200

/* Group 4, 2.4 GHz all channels:  3.5 degrees per 1/2 dB. */
#define CALIB_IWL_TX_ATTEN_GR5_FCH 1
#define CALIB_IWL_TX_ATTEN_GR5_LCH 20

enum {
	CALIB_CH_GROUP_1 = 0,
	CALIB_CH_GROUP_2 = 1,
	CALIB_CH_GROUP_3 = 2,
	CALIB_CH_GROUP_4 = 3,
	CALIB_CH_GROUP_5 = 4,
	CALIB_CH_GROUP_MAX
};

/********************* END TXPOWER *****************************************/


#define IWL49_NUM_FIFOS 	7
#define IWL49_CMD_FIFO_NUM	4
#define IWL49_NUM_QUEUES	16
#define IWL49_NUM_AMPDU_QUEUES	8


struct iwl4965_scd_bc_tbl {
	__le16 tfd_offset[TFD_QUEUE_BC_SIZE];
	u8 pad[1024 - (TFD_QUEUE_BC_SIZE) * sizeof(__le16)];
} __attribute__ ((packed));

#endif /* !__iwl_4965_hw_h__ */
