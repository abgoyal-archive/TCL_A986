
#ifndef _ISERIES_IPL_PARMS_H
#define _ISERIES_IPL_PARMS_H


#include <asm/types.h>

struct ItIplParmsReal {
	u8	xFormat;		// Defines format of IplParms	x00-x00
	u8	xRsvd01:6;		// Reserved			x01-x01
	u8	xAlternateSearch:1;	// Alternate search indicator	...
	u8	xUaSupplied:1;		// UA Supplied on programmed IPL...
	u8	xLsUaFormat;		// Format byte for UA		x02-x02
	u8	xRsvd02;		// Reserved			x03-x03
	u32	xLsUa;			// LS UA			x04-x07
	u32	xUnusedLsLid;		// First OS LID to load		x08-x0B
	u16	xLsBusNumber;		// LS Bus Number		x0C-x0D
	u8	xLsCardAdr;		// LS Card Address		x0E-x0E
	u8	xLsBoardAdr;		// LS Board Address		x0F-x0F
	u32	xRsvd03;		// Reserved			x10-x13
	u8	xSpcnPresent:1;		// SPCN present			x14-x14
	u8	xCpmPresent:1;		// CPM present			...
	u8	xRsvd04:6;		// Reserved			...
	u8	xRsvd05:4;		// Reserved			x15-x15
	u8	xKeyLock:4;		// Keylock setting		...
	u8	xRsvd06:6;		// Reserved			x16-x16
	u8	xIplMode:2;		// Ipl mode (A|B|C|D)		...
	u8	xHwIplType;		// Fast v slow v slow EC HW IPL	x17-x17
	u16	xCpmEnabledIpl:1;	// CPM in effect when IPL initiatedx18-x19
	u16	xPowerOnResetIpl:1;	// Indicate POR condition	...
	u16	xMainStorePreserved:1;	// Main Storage is preserved	...
	u16	xRsvd07:13;		// Reserved			...
	u16	xIplSource:16;		// Ipl source			x1A-x1B
	u8	xIplReason:8;		// Reason for this IPL		x1C-x1C
	u8	xRsvd08;		// Reserved			x1D-x1D
	u16	xRsvd09;		// Reserved			x1E-x1F
	u16	xSysBoxType;		// System Box Type		x20-x21
	u16	xSysProcType;		// System Processor Type	x22-x23
	u32	xRsvd10;		// Reserved			x24-x27
	u64	xRsvd11;		// Reserved			x28-x2F
	u64	xRsvd12;		// Reserved			x30-x37
	u64	xRsvd13;		// Reserved			x38-x3F
};

#endif /* _ISERIES_IPL_PARMS_H */
