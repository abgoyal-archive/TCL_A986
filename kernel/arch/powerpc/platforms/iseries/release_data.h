
#ifndef _ISERIES_RELEASE_DATA_H
#define _ISERIES_RELEASE_DATA_H

#include <asm/types.h>
#include "naca.h"


#define	HVREL_TAGSINACTIVE	0x8000
#define HVREL_32BIT		0x4000
#define HVREL_NOSHAREDPROCS	0x2000
#define HVREL_NOHMT		0x1000

struct HvReleaseData {
	u32	xDesc;		/* Descriptor "HvRD" ebcdic	x00-x03 */
	u16	xSize;		/* Size of this control block	x04-x05 */
	u16	xVpdAreasPtrOffset; /* Offset in NACA of ItVpdAreas x06-x07 */
	struct  naca_struct	*xSlicNacaAddr; /* Virt addr of SLIC NACA x08-x0F */
	u32	xMsNucDataOffset; /* Offset of Linux Mapping Data x10-x13 */
	u32	xRsvd1;		/* Reserved			x14-x17 */
	u16	xFlags;
	u16	xVrmIndex;	/* VRM Index of OS image	x1A-x1B */
	u16	xMinSupportedPlicVrmIndex; /* Min PLIC level  (soft) x1C-x1D */
	u16	xMinCompatablePlicVrmIndex; /* Min PLIC levelP (hard) x1E-x1F */
	char	xVrmName[12];	/* Displayable name		x20-x2B */
	char	xRsvd3[20];	/* Reserved			x2C-x3F */
};

extern const struct HvReleaseData	hvReleaseData;

#endif /* _ISERIES_RELEASE_DATA_H */
