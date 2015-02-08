
/*                                                                      */
/*  Module Name : zdusb.h                                               */
/*                                                                      */
/*  Abstract                                                            */
/*     This module contains definitions for USB device driver           */
/*                                                                      */
/*  NOTES                                                               */
/*     Platform dependent.                                              */
/*                                                                      */
/************************************************************************/

#ifndef _ZDUSB_H
#define _ZDUSB_H

#ifndef DRIVER_NAME
#define DRIVER_NAME             "arusb"
#endif

#define VERSIONID               "0.0.0.999"

/* Define these values to match your device */
#define VENDOR_ATHR             0x0CF3  /* Atheros */
#define PRODUCT_AR9170          0x9170

#define VENDOR_DLINK            0x07D1  /* Dlink */
#define PRODUCT_DWA160A         0x3C10

#define	VENDOR_NETGEAR		0x0846	/* NetGear */
#define	PRODUCT_WNDA3100	0x9010
#define	PRODUCT_WN111v2		0x9001

#endif
