
#ifndef HOSTAP_CONFIG_H
#define HOSTAP_CONFIG_H

/* #define PRISM2_NO_KERNEL_IEEE80211_MGMT */

/* Maximum number of events handler per one interrupt */
#define PRISM2_MAX_INTERRUPT_EVENTS 20

/* Include code for downloading firmware images into volatile RAM. */
#define PRISM2_DOWNLOAD_SUPPORT

/* Allow kernel configuration to enable download support. */
#if !defined(PRISM2_DOWNLOAD_SUPPORT) && defined(CONFIG_HOSTAP_FIRMWARE)
#define PRISM2_DOWNLOAD_SUPPORT
#endif

/* Allow kernel configuration to enable non-volatile download support. */
#ifdef CONFIG_HOSTAP_FIRMWARE_NVRAM
#define PRISM2_NON_VOLATILE_DOWNLOAD
#endif

/* #define PRISM2_IO_DEBUG */


/* Do not include debug messages into the driver */
/* #define PRISM2_NO_DEBUG */

/* Do not include /proc/net/prism2/wlan#/{registers,debug} */
/* #define PRISM2_NO_PROCFS_DEBUG */

/* #define PRISM2_NO_STATION_MODES */

#endif /* HOSTAP_CONFIG_H */
