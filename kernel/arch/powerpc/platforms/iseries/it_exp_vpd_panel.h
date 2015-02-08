
#ifndef _PLATFORMS_ISERIES_IT_EXT_VPD_PANEL_H
#define _PLATFORMS_ISERIES_IT_EXT_VPD_PANEL_H


#include <asm/types.h>

struct ItExtVpdPanel {
	/* Definition of the Extended Vpd On Panel Data Area */
	char	systemSerial[8];
	char	mfgID[4];
	char	reserved1[24];
	char	machineType[4];
	char	systemID[6];
	char	somUniqueCnt[4];
	char	serialNumberCount;
	char	reserved2[7];
	u16	bbu3;
	u16	bbu2;
	u16	bbu1;
	char	xLocationLabel[8];
	u8	xRsvd1[6];
	u16	xFrameId;
	u8	xRsvd2[48];
};

extern struct ItExtVpdPanel	xItExtVpdPanel;

#endif /* _PLATFORMS_ISERIES_IT_EXT_VPD_PANEL_H */
