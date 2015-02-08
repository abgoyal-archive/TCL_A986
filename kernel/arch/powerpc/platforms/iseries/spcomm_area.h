

#ifndef _ISERIES_SPCOMM_AREA_H
#define _ISERIES_SPCOMM_AREA_H


struct SpCommArea {
	u32	xDesc;			// Descriptor (only in new formats)	000-003
	u8	xFormat;		// Format (only in new formats)		004-004
	u8	xRsvd1[11];		// Reserved				005-00F
	u64	xRawTbAtIplStart;	// Raw HW TB value when IPL is started	010-017
	u64	xRawTodAtIplStart;	// Raw HW TOD value when IPL is started	018-01F
	u64	xBcdTimeAtIplStart;	// BCD time when IPL is started		020-027
	u64	xBcdTimeAtOsStart;	// BCD time when OS passed control	028-02F
	u8	xRsvd2[80];		// Reserved				030-07F
};

#endif /* _ISERIES_SPCOMM_AREA_H */
