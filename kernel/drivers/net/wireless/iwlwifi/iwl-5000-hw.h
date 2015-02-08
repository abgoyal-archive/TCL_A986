

#ifndef __iwl_5000_hw_h__
#define __iwl_5000_hw_h__

/* 5150 only */
#define IWL_5150_VOLTAGE_TO_TEMPERATURE_COEFF	(-5)

static inline s32 iwl_temp_calib_to_offset(struct iwl_priv *priv)
{
	u16 temperature, voltage;
	__le16 *temp_calib =
		(__le16 *)iwl_eeprom_query_addr(priv, EEPROM_5000_TEMPERATURE);

	temperature = le16_to_cpu(temp_calib[0]);
	voltage = le16_to_cpu(temp_calib[1]);

	/* offset = temp - volt / coeff */
	return (s32)(temperature - voltage / IWL_5150_VOLTAGE_TO_TEMPERATURE_COEFF);
}

#endif /* __iwl_5000_hw_h__ */

