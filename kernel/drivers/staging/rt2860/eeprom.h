
#ifndef __EEPROM_H__
#define __EEPROM_H__

#ifdef RTMP_PCI_SUPPORT
int rtmp_ee_prom_read16(struct rt_rtmp_adapter *pAd,
			u16 Offset, u16 * pValue);
#endif /* RTMP_PCI_SUPPORT // */
#ifdef RTMP_USB_SUPPORT
int RTUSBReadEEPROM16(struct rt_rtmp_adapter *pAd,
			   u16 offset, u16 *pData);
#endif /* RTMP_USB_SUPPORT // */

#ifdef RT30xx
#ifdef RTMP_EFUSE_SUPPORT
int rtmp_ee_efuse_read16(struct rt_rtmp_adapter *pAd,
			 u16 Offset, u16 * pValue);
#endif /* RTMP_EFUSE_SUPPORT // */
#endif /* RT30xx // */

int RtmpChipOpsEepromHook(struct rt_rtmp_adapter *pAd, int infType);

#endif /* __EEPROM_H__ // */
