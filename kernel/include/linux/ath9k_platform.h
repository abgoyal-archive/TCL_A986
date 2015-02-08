

#ifndef _LINUX_ATH9K_PLATFORM_H
#define _LINUX_ATH9K_PLATFORM_H

#define ATH9K_PLAT_EEP_MAX_WORDS	2048

struct ath9k_platform_data {
	u16 eeprom_data[ATH9K_PLAT_EEP_MAX_WORDS];
};

#endif /* _LINUX_ATH9K_PLATFORM_H */
