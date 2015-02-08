
#ifndef _ASM_POWERPC_ISERIES_LPAR_MAP_H
#define _ASM_POWERPC_ISERIES_LPAR_MAP_H

#ifndef __ASSEMBLY__

#include <asm/types.h>

#endif



#define HvEsidsToMap	2
#define HvRangesToMap	1

/* Hypervisor initially maps 32MB of the load area */
#define HvPagesToMap	8192

#ifndef __ASSEMBLY__
struct LparMap {
	u64	xNumberEsids;	// Number of ESID/VSID pairs
	u64	xNumberRanges;	// Number of VA ranges to map
	u64	xSegmentTableOffs; // Page number within load area of seg table
	u64	xRsvd[5];
	struct {
		u64	xKernelEsid;	// Esid used to map kernel load
		u64	xKernelVsid;	// Vsid used to map kernel load
	} xEsids[HvEsidsToMap];
	struct {
		u64	xPages;		// Number of pages to be mapped
		u64	xOffset;	// Offset from start of load area
		u64	xVPN;		// Virtual Page Number
	} xRanges[HvRangesToMap];
};

extern const struct LparMap	xLparMap;

#endif /* __ASSEMBLY__ */

/* the fixed address where the LparMap exists */
#define LPARMAP_PHYS		0x7000

#endif /* _ASM_POWERPC_ISERIES_LPAR_MAP_H */
