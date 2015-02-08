

#ifndef __RT3070_H__
#define __RT3070_H__

#ifdef RT3070

#ifndef RTMP_USB_SUPPORT
#error "For RT3070, you should define the compile flag -DRTMP_USB_SUPPORT"
#endif

#ifndef RTMP_MAC_USB
#error "For RT3070, you should define the compile flag -DRTMP_MAC_USB"
#endif

#ifndef RTMP_RF_RW_SUPPORT
#error "For RT3070, you should define the compile flag -DRTMP_RF_RW_SUPPORT"
#endif

#ifndef RT30xx
#error "For RT3070, you should define the compile flag -DRT30xx"
#endif

#include "mac_usb.h"
#include "rt30xx.h"

/* */
/* Device ID & Vendor ID, these values should match EEPROM value */
/* */

#endif /* RT3070 // */

#endif /*__RT3070_H__ // */
