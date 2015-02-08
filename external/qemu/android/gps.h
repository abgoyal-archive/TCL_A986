
#ifndef _android_gps_h
#define _android_gps_h

#include "qemu-common.h"

extern CharDriverState*  android_gps_cs;

extern void  android_gps_send_nmea( const char*  sentence );

#endif /* _android_gps_h */
