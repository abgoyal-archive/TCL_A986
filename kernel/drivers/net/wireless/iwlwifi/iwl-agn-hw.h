

#ifndef __iwl_agn_hw_h__
#define __iwl_agn_hw_h__

#define IWLAGN_RTC_INST_LOWER_BOUND		(0x000000)
#define IWLAGN_RTC_INST_UPPER_BOUND		(0x020000)

#define IWLAGN_RTC_DATA_LOWER_BOUND		(0x800000)
#define IWLAGN_RTC_DATA_UPPER_BOUND		(0x80C000)

#define IWLAGN_RTC_INST_SIZE (IWLAGN_RTC_INST_UPPER_BOUND - \
				IWLAGN_RTC_INST_LOWER_BOUND)
#define IWLAGN_RTC_DATA_SIZE (IWLAGN_RTC_DATA_UPPER_BOUND - \
				IWLAGN_RTC_DATA_LOWER_BOUND)

/* RSSI to dBm */
#define IWLAGN_RSSI_OFFSET	44

/* PCI registers */
#define PCI_CFG_RETRY_TIMEOUT	0x041

/* PCI register values */
#define PCI_CFG_LINK_CTRL_VAL_L0S_EN	0x01
#define PCI_CFG_LINK_CTRL_VAL_L1_EN	0x02

#define IWLAGN_DEFAULT_TX_RETRY  15

/* Limit range of txpower output target to be between these values */
#define IWLAGN_TX_POWER_TARGET_POWER_MIN	(0)	/* 0 dBm: 1 milliwatt */
#define IWLAGN_TX_POWER_TARGET_POWER_MAX	(16)	/* 16 dBm */

/* EEPROM */
#define IWLAGN_EEPROM_IMG_SIZE		2048

#define IWLAGN_CMD_FIFO_NUM		7
#define IWLAGN_NUM_QUEUES		20
#define IWLAGN_NUM_AMPDU_QUEUES		10
#define IWLAGN_FIRST_AMPDU_QUEUE	10

/* Fixed (non-configurable) rx data from phy */

struct iwlagn_scd_bc_tbl {
	__le16 tfd_offset[TFD_QUEUE_BC_SIZE];
} __attribute__ ((packed));


#endif /* __iwl_agn_hw_h__ */
