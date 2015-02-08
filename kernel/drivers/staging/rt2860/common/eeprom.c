
#include "../rt_config.h"

int RtmpChipOpsEepromHook(struct rt_rtmp_adapter *pAd, int infType)
{
	struct rt_rtmp_chip_op *pChipOps = &pAd->chipOps;
#ifdef RT30xx
#ifdef RTMP_EFUSE_SUPPORT
	u32 eFuseCtrl, MacCsr0;
	int index;

	index = 0;
	do {
		RTMP_IO_READ32(pAd, MAC_CSR0, &MacCsr0);
		pAd->MACVersion = MacCsr0;

		if ((pAd->MACVersion != 0x00)
		    && (pAd->MACVersion != 0xFFFFFFFF))
			break;

		RTMPusecDelay(10);
	} while (index++ < 100);

	pAd->bUseEfuse = FALSE;
	RTMP_IO_READ32(pAd, EFUSE_CTRL, &eFuseCtrl);
	pAd->bUseEfuse = ((eFuseCtrl & 0x80000000) == 0x80000000) ? 1 : 0;
	if (pAd->bUseEfuse) {
		pChipOps->eeinit = eFuse_init;
		pChipOps->eeread = rtmp_ee_efuse_read16;
		return 0;
	} else
		DBGPRINT(RT_DEBUG_TRACE, ("NVM is EEPROM\n"));
#endif /* RTMP_EFUSE_SUPPORT // */
#endif /* RT30xx // */

	switch (infType) {
#ifdef RTMP_PCI_SUPPORT
	case RTMP_DEV_INF_PCI:
		pChipOps->eeinit = NULL;
		pChipOps->eeread = rtmp_ee_prom_read16;
		break;
#endif /* RTMP_PCI_SUPPORT // */
#ifdef RTMP_USB_SUPPORT
	case RTMP_DEV_INF_USB:
		pChipOps->eeinit = NULL;
		pChipOps->eeread = RTUSBReadEEPROM16;
		break;
#endif /* RTMP_USB_SUPPORT // */

	default:
		DBGPRINT(RT_DEBUG_ERROR, ("RtmpChipOpsEepromHook() failed!\n"));
		break;
	}

	return 0;
}
