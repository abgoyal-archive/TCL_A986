
#ifndef MTK_USB_CUSTOM_H
#define MTK_USB_CUSTOM_H

/* Force full-speed is not guaranteed with adb function, use it with care! */
//#define USB_FORCE_FULL_SPEED

#define USB_MS_PRODUCT_ID           0x0001
//#define USB_MS_ADB_PRODUCT_ID       0x0c03
#define USB_MS_ADB_PRODUCT_ID       0x00f2
#define USB_RNDIS_PRODUCT_ID        0x0003
#define USB_RNDIS_ADB_PRODUCT_ID    0x0004
#define USB_MS_ADB_ACM_PRODUCT_ID   0x0005
#define USB_ACM_PRODUCT_ID          0x0006

//#define VENDOR_ID      0x0bb4                /* USB vendor id  */
#define VENDOR_ID      0x1bbb                /* USB vendor id  */
#define PRODUCT_ID     USB_MS_PRODUCT_ID     /* USB default product id */

#define MANUFACTURER_STRING "Alcatel"
#define PRODUCT_STRING      "Alcatel Android phone"

#define USB_ETH_VENDORID     0
#define USB_ETH_VENDORDESCR  "Alcatel"

#define USB_MS_VENDOR        "Alcatel"
#define USB_MS_PRODUCT       "Alcatel MS"
#define USB_MS_RELEASE       0x0100

#define CHR_TYPE_DETECT_DEB  400  /* debounce time for charger type detection, in ms */

#endif /* MTK_USB_CUSTOM_H */
