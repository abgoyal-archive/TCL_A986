
#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

//#include <linux/config.h>
#include <linux/types.h>

#include "ttype.h"

typedef
struct _version {
    unsigned char   major;
    unsigned char   minor;
    unsigned char   build;
} version_t, *pversion_t;

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (!(FALSE))
#endif

#define VID_TABLE_SIZE      64
#define MCAST_TABLE_SIZE    64
#define MCAM_SIZE           32
#define VCAM_SIZE           32
#define TX_QUEUE_NO         8

#define DEVICE_NAME         "vt6655"
#define DEVICE_FULL_DRV_NAM "VIA Networking Solomon-A/B/G Wireless LAN Adapter Driver"

#ifndef MAJOR_VERSION
#define MAJOR_VERSION       1
#endif

#ifndef MINOR_VERSION
#define MINOR_VERSION       17
#endif

#ifndef DEVICE_VERSION
#define DEVICE_VERSION       "1.19.12"
#endif

//config file
#include <linux/fs.h>
#include <linux/fcntl.h>
#ifndef CONFIG_PATH
#define CONFIG_PATH            "/etc/vntconfiguration.dat"
#endif

//Max: 2378=2312Payload + 30HD +4CRC + 2Padding + 4Len + 8TSF + 4RSR
#define PKT_BUF_SZ          2390


#define MAX_UINTS           8
#define OPTION_DEFAULT      { [0 ... MAX_UINTS-1] = -1}



typedef enum  _chip_type{
    VT3253=1
} CHIP_TYPE, *PCHIP_TYPE;



#ifdef VIAWET_DEBUG
#define ASSERT(x) { \
    if (!(x)) { \
        printk(KERN_ERR "assertion %s failed: file %s line %d\n", #x,\
        __FUNCTION__, __LINE__);\
        *(int*) 0=0;\
    }\
}
#define DBG_PORT80(value)                   outb(value, 0x80)
#else
#define ASSERT(x)
#define DBG_PORT80(value)
#endif


#endif
