

#ifndef __BFI_BOOT_H__
#define __BFI_BOOT_H__

#define BFI_BOOT_TYPE_OFF		8
#define BFI_BOOT_PARAM_OFF		12

#define BFI_BOOT_TYPE_NORMAL 		0	/* param is device id */
#define	BFI_BOOT_TYPE_FLASH		1
#define	BFI_BOOT_TYPE_MEMTEST		2

#define BFI_BOOT_MEMTEST_RES_ADDR   0x900
#define BFI_BOOT_MEMTEST_RES_SIG    0xA0A1A2A3

#endif
