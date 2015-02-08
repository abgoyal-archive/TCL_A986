

#ifndef __IWM_EEPROM_H__
#define __IWM_EEPROM_H__

enum {
	IWM_EEPROM_SIG = 0,
	IWM_EEPROM_FIRST = IWM_EEPROM_SIG,
	IWM_EEPROM_VERSION,
	IWM_EEPROM_OEM_HW_VERSION,
	IWM_EEPROM_MAC_VERSION,
	IWM_EEPROM_CARD_ID,
	IWM_EEPROM_RADIO_CONF,
	IWM_EEPROM_SKU_CAP,
	IWM_EEPROM_FAT_CHANNELS_CAP,

	IWM_EEPROM_INDIRECT_OFFSET,
	IWM_EEPROM_CALIB_RXIQ_OFFSET = IWM_EEPROM_INDIRECT_OFFSET,

	IWM_EEPROM_INDIRECT_DATA,
	IWM_EEPROM_CALIB_RXIQ = IWM_EEPROM_INDIRECT_DATA,

	IWM_EEPROM_LAST,
};

#define IWM_EEPROM_SIG_OFF                 0x00
#define IWM_EEPROM_VERSION_OFF            (0x54 << 1)
#define IWM_EEPROM_OEM_HW_VERSION_OFF     (0x56 << 1)
#define IWM_EEPROM_MAC_VERSION_OFF        (0x30 << 1)
#define IWM_EEPROM_CARD_ID_OFF            (0x5d << 1)
#define IWM_EEPROM_RADIO_CONF_OFF         (0x58 << 1)
#define IWM_EEPROM_SKU_CAP_OFF            (0x55 << 1)
#define IWM_EEPROM_CALIB_CONFIG_OFF       (0x7c << 1)
#define IWM_EEPROM_FAT_CHANNELS_CAP_OFF   (0xde << 1)

#define IWM_EEPROM_SIG_LEN              4
#define IWM_EEPROM_VERSION_LEN          2
#define IWM_EEPROM_OEM_HW_VERSION_LEN   2
#define IWM_EEPROM_MAC_VERSION_LEN      1
#define IWM_EEPROM_CARD_ID_LEN          2
#define IWM_EEPROM_RADIO_CONF_LEN       2
#define IWM_EEPROM_SKU_CAP_LEN          2
#define IWM_EEPROM_FAT_CHANNELS_CAP_LEN 40
#define IWM_EEPROM_INDIRECT_LEN		2

#define IWM_MAX_EEPROM_DATA_LEN         240
#define IWM_EEPROM_LEN                  0x800

#define IWM_EEPROM_MIN_ALLOWED_VERSION          0x0610
#define IWM_EEPROM_MAX_ALLOWED_VERSION          0x0700
#define IWM_EEPROM_CURRENT_VERSION              0x0612

#define IWM_EEPROM_SKU_CAP_BAND_24GHZ           (1 << 4)
#define IWM_EEPROM_SKU_CAP_BAND_52GHZ           (1 << 5)
#define IWM_EEPROM_SKU_CAP_11N_ENABLE           (1 << 6)

#define IWM_EEPROM_FAT_CHANNELS 20
/* 2.4 gHz FAT primary channels: 1, 2, 3, 4, 5, 6, 7, 8, 9 */
#define IWM_EEPROM_FAT_CHANNELS_24 9
/* 5.2 gHz FAT primary channels: 36,44,52,60,100,108,116,124,132,149,157 */
#define IWM_EEPROM_FAT_CHANNELS_52 11

#define IWM_EEPROM_FAT_CHANNEL_ENABLED (1 << 0)

enum {
	IWM_EEPROM_CALIB_CAL_HDR,
	IWM_EEPROM_CALIB_TX_POWER,
	IWM_EEPROM_CALIB_XTAL,
	IWM_EEPROM_CALIB_TEMPERATURE,
	IWM_EEPROM_CALIB_RX_BB_FILTER,
	IWM_EEPROM_CALIB_RX_IQ,
	IWM_EEPROM_CALIB_MAX,
};

#define IWM_EEPROM_CALIB_RXIQ_OFF	(IWM_EEPROM_CALIB_CONFIG_OFF + \
					 (IWM_EEPROM_CALIB_RX_IQ << 1))
#define IWM_EEPROM_CALIB_RXIQ_LEN	sizeof(struct iwm_lmac_calib_rxiq)

struct iwm_eeprom_entry {
	char *name;
	u32 offset;
	u32 length;
};

int iwm_eeprom_init(struct iwm_priv *iwm);
void iwm_eeprom_exit(struct iwm_priv *iwm);
u8 *iwm_eeprom_access(struct iwm_priv *iwm, u8 eeprom_id);
int iwm_eeprom_fat_channels(struct iwm_priv *iwm);
u32 iwm_eeprom_wireless_mode(struct iwm_priv *iwm);

#endif
