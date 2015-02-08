

#ifndef __NVRAM_H
#define __NVRAM_H

#include <linux/types.h>

struct nvram_header {
	u32 magic;
	u32 len;
	u32 crc_ver_init;	/* 0:7 crc, 8:15 ver, 16:31 sdram_init */
	u32 config_refresh;	/* 0:15 sdram_config, 16:31 sdram_refresh */
	u32 config_ncdl;	/* ncdl values for memc */
};

#define NVRAM_HEADER		0x48534C46	/* 'FLSH' */
#define NVRAM_VERSION		1
#define NVRAM_HEADER_SIZE	20
#define NVRAM_SPACE		0x8000

#define FLASH_MIN		0x00020000	/* Minimum flash size */

#define NVRAM_MAX_VALUE_LEN 255
#define NVRAM_MAX_PARAM_LEN 64

extern int nvram_getenv(char *name, char *val, size_t val_len);

#endif
