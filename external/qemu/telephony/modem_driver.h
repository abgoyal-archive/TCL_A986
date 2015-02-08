
#ifndef _modem_driver_h
#define _modem_driver_h

#include "android_modem.h"
#include "qemu-common.h"

/** in telephony/modem_driver.c */
extern CharDriverState*  android_modem_cs;

/* the emulated GSM modem itself */
extern AModem  android_modem;

/* must be called before the VM runs if there is a modem to emulate */
extern void   android_modem_init( int  base_port );

#endif /* _modem_driver_h */
