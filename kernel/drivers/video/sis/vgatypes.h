
/* $XFree86$ */
/* $XdotOrg$ */

#ifndef _VGATYPES_H_
#define _VGATYPES_H_

#define SISIOMEMTYPE

#ifdef SIS_LINUX_KERNEL
typedef unsigned long SISIOADDRESS;
#include <linux/types.h>  /* Need __iomem */
#undef SISIOMEMTYPE
#define SISIOMEMTYPE __iomem
#endif

#ifdef SIS_XORG_XF86
#if XF86_VERSION_CURRENT < XF86_VERSION_NUMERIC(4,2,0,0,0)
typedef unsigned long IOADDRESS;
typedef unsigned long SISIOADDRESS;
#else
typedef IOADDRESS SISIOADDRESS;
#endif
#endif

typedef enum _SIS_CHIP_TYPE {
    SIS_VGALegacy = 0,
    SIS_530,
    SIS_OLD,
    SIS_300,
    SIS_630,
    SIS_730,
    SIS_540,
    SIS_315H,   /* SiS 310 */
    SIS_315,
    SIS_315PRO, /* SiS 325 */
    SIS_550,
    SIS_650,
    SIS_740,
    SIS_330,
    SIS_661,
    SIS_741,
    SIS_670,
    SIS_660 = 35,
    SIS_760,
    SIS_761,
    SIS_762,
    SIS_770,
    SIS_340 = 55,
    SIS_341,
    SIS_342,
    XGI_20  = 75,
    XGI_40,
    MAX_SIS_CHIP
} SIS_CHIP_TYPE;


#endif

