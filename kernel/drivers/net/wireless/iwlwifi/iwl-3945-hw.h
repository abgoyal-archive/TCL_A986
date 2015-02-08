

#ifndef __iwl_3945_hw__
#define __iwl_3945_hw__

#include "iwl-eeprom.h"

/* RSSI to dBm */
#define IWL39_RSSI_OFFSET	95

#define IWL_DEFAULT_TX_POWER	0x0F

#define EEPROM_SKU_CAP_OP_MODE_MRC                      (1 << 7)

struct iwl3945_eeprom_txpower_sample {
	u8 gain_index;		/* index into power (gain) setup table ... */
	s8 power;		/* ... for this pwr level for this chnl group */
	u16 v_det;		/* PA output voltage */
} __attribute__ ((packed));

struct iwl3945_eeprom_txpower_group {
	struct iwl3945_eeprom_txpower_sample samples[5];  /* 5 power levels */
	s32 a, b, c, d, e;	/* coefficients for voltage->power
				 * formula (signed) */
	s32 Fa, Fb, Fc, Fd, Fe;	/* these modify coeffs based on
				 * frequency (signed) */
	s8 saturation_power;	/* highest power possible by h/w in this
				 * band */
	u8 group_channel;	/* "representative" channel # in this band */
	s16 temperature;	/* h/w temperature at factory calib this band
				 * (signed) */
} __attribute__ ((packed));

struct iwl3945_eeprom_temperature_corr {
	u32 Ta;
	u32 Tb;
	u32 Tc;
	u32 Td;
	u32 Te;
} __attribute__ ((packed));

struct iwl3945_eeprom {
	u8 reserved0[16];
	u16 device_id;	/* abs.ofs: 16 */
	u8 reserved1[2];
	u16 pmc;		/* abs.ofs: 20 */
	u8 reserved2[20];
	u8 mac_address[6];	/* abs.ofs: 42 */
	u8 reserved3[58];
	u16 board_revision;	/* abs.ofs: 106 */
	u8 reserved4[11];
	u8 board_pba_number[9];	/* abs.ofs: 119 */
	u8 reserved5[8];
	u16 version;		/* abs.ofs: 136 */
	u8 sku_cap;		/* abs.ofs: 138 */
	u8 leds_mode;		/* abs.ofs: 139 */
	u16 oem_mode;
	u16 wowlan_mode;	/* abs.ofs: 142 */
	u16 leds_time_interval;	/* abs.ofs: 144 */
	u8 leds_off_time;	/* abs.ofs: 146 */
	u8 leds_on_time;	/* abs.ofs: 147 */
	u8 almgor_m_version;	/* abs.ofs: 148 */
	u8 antenna_switch_type;	/* abs.ofs: 149 */
	u8 reserved6[42];
	u8 sku_id[4];		/* abs.ofs: 192 */

	u16 band_1_count;	/* abs.ofs: 196 */
	struct iwl_eeprom_channel band_1_channels[14];  /* abs.ofs: 198 */

	u16 band_2_count;	/* abs.ofs: 226 */
	struct iwl_eeprom_channel band_2_channels[13];  /* abs.ofs: 228 */

	u16 band_3_count;	/* abs.ofs: 254 */
	struct iwl_eeprom_channel band_3_channels[12];  /* abs.ofs: 256 */

	u16 band_4_count;	/* abs.ofs: 280 */
	struct iwl_eeprom_channel band_4_channels[11];  /* abs.ofs: 282 */

	u16 band_5_count;	/* abs.ofs: 304 */
	struct iwl_eeprom_channel band_5_channels[6];  /* abs.ofs: 306 */

	u8 reserved9[194];

#define IWL_NUM_TX_CALIB_GROUPS 5
	struct iwl3945_eeprom_txpower_group groups[IWL_NUM_TX_CALIB_GROUPS];
/* abs.ofs: 512 */
	struct iwl3945_eeprom_temperature_corr corrections;  /* abs.ofs: 832 */
	u8 reserved16[172];	/* fill out to full 1024 byte block */
} __attribute__ ((packed));

#define IWL3945_EEPROM_IMG_SIZE 1024

/* End of EEPROM */

#define PCI_CFG_REV_ID_BIT_BASIC_SKU                (0x40)	/* bit 6    */
#define PCI_CFG_REV_ID_BIT_RTP                      (0x80)	/* bit 7    */

/* 4 DATA + 1 CMD. There are 2 HCCA queues that are not used. */
#define IWL39_NUM_QUEUES        5

#define IWL_DEFAULT_TX_RETRY  15

/*********************************************/

#define RFD_SIZE                              4
#define NUM_TFD_CHUNKS                        4

#define RX_QUEUE_SIZE                         256
#define RX_QUEUE_MASK                         255
#define RX_QUEUE_SIZE_LOG                     8

#define U32_PAD(n)		((4-(n))&0x3)

#define TFD_CTL_COUNT_SET(n)       (n << 24)
#define TFD_CTL_COUNT_GET(ctl)     ((ctl >> 24) & 7)
#define TFD_CTL_PAD_SET(n)         (n << 28)
#define TFD_CTL_PAD_GET(ctl)       (ctl >> 28)

#define IWL39_RTC_INST_LOWER_BOUND		(0x000000)
#define IWL39_RTC_INST_UPPER_BOUND		(0x014000)

#define IWL39_RTC_DATA_LOWER_BOUND		(0x800000)
#define IWL39_RTC_DATA_UPPER_BOUND		(0x808000)

#define IWL39_RTC_INST_SIZE (IWL39_RTC_INST_UPPER_BOUND - \
				IWL39_RTC_INST_LOWER_BOUND)
#define IWL39_RTC_DATA_SIZE (IWL39_RTC_DATA_UPPER_BOUND - \
				IWL39_RTC_DATA_LOWER_BOUND)

#define IWL39_MAX_INST_SIZE IWL39_RTC_INST_SIZE
#define IWL39_MAX_DATA_SIZE IWL39_RTC_DATA_SIZE

/* Size of uCode instruction memory in bootstrap state machine */
#define IWL39_MAX_BSM_SIZE IWL39_RTC_INST_SIZE

static inline int iwl3945_hw_valid_rtc_data_addr(u32 addr)
{
	return (addr >= IWL39_RTC_DATA_LOWER_BOUND) &&
	       (addr < IWL39_RTC_DATA_UPPER_BOUND);
}

struct iwl3945_shared {
	__le32 tx_base_ptr[8];
} __attribute__ ((packed));

static inline u8 iwl3945_hw_get_rate(__le16 rate_n_flags)
{
	return le16_to_cpu(rate_n_flags) & 0xFF;
}

static inline u16 iwl3945_hw_get_rate_n_flags(__le16 rate_n_flags)
{
	return le16_to_cpu(rate_n_flags);
}

static inline __le16 iwl3945_hw_set_rate_n_flags(u8 rate, u16 flags)
{
	return cpu_to_le16((u16)rate|flags);
}
#endif
