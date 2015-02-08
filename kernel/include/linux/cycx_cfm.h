
#ifndef	_CYCX_CFM_H
#define	_CYCX_CFM_H

/* Defines */

#define	CFM_VERSION	2
#define	CFM_SIGNATURE	"CFM - Cyclades CYCX Firmware Module"

/* min/max */
#define	CFM_IMAGE_SIZE	0x20000	/* max size of CYCX code image file */
#define	CFM_DESCR_LEN	256	/* max length of description string */
#define	CFM_MAX_CYCX	1	/* max number of compatible adapters */
#define	CFM_LOAD_BUFSZ	0x400	/* buffer size for reset code (buffer_load) */

/* Firmware Commands */
#define GEN_POWER_ON	0x1280

#define GEN_SET_SEG	0x1401	/* boot segment setting. */
#define GEN_BOOT_DAT	0x1402	/* boot data. */
#define GEN_START	0x1403	/* board start. */
#define GEN_DEFPAR	0x1404	/* buffer length for boot. */

/* Adapter Types */
#define CYCX_2X		2
/* for now only the 2X is supported, no plans to support 8X or 16X */
#define CYCX_8X		8
#define CYCX_16X	16

#define	CFID_X25_2X	5200

struct cycx_fw_info {
	unsigned short	codeid;
	unsigned short	version;
	unsigned short	adapter[CFM_MAX_CYCX];
	unsigned long	memsize;
	unsigned short	reserved[2];
	unsigned short	startoffs;
	unsigned short	winoffs;
	unsigned short	codeoffs;
	unsigned long	codesize;
	unsigned short	dataoffs;
	unsigned long	datasize;
};

struct cycx_firmware {
	char		    signature[80];
	unsigned short	    version;
	unsigned short	    checksum;
	unsigned short	    reserved[6];
	char		    descr[CFM_DESCR_LEN];
	struct cycx_fw_info info;
	unsigned char	    image[0];
};

struct cycx_fw_header {
	unsigned long  reset_size;
	unsigned long  data_size;
	unsigned long  code_size;
};
#endif	/* _CYCX_CFM_H */
