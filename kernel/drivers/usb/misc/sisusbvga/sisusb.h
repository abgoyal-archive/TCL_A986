

#ifndef _SISUSB_H_
#define _SISUSB_H_

#ifdef CONFIG_COMPAT
#define SISUSB_NEW_CONFIG_COMPAT
#endif

#include <linux/mutex.h>

/* #define CONFIG_USB_SISUSBVGA_CON */

/* Version Information */

#define SISUSB_VERSION		0
#define SISUSB_REVISION		0
#define SISUSB_PATCHLEVEL	8

/* Include console and mode switching code? */

#ifdef CONFIG_USB_SISUSBVGA_CON
#define INCL_SISUSB_CON		1
#endif

#include <linux/console.h>
#include <linux/vt_kern.h>
#include "sisusb_struct.h"

/* USB related */

#define SISUSB_MINOR		133	/* official */

/* Size of the sisusb input/output buffers */
#define SISUSB_IBUF_SIZE  0x01000
#define SISUSB_OBUF_SIZE  0x10000	/* fixed */

#define NUMOBUFS 8		/* max number of output buffers/output URBs */


#ifdef __BIG_ENDIAN
#define SISUSB_CORRECT_ENDIANNESS_PACKET(p)		\
	do {						\
		p->header  = cpu_to_le16(p->header);	\
		p->address = cpu_to_le32(p->address);	\
		p->data    = cpu_to_le32(p->data);	\
	} while(0)
#else
#define SISUSB_CORRECT_ENDIANNESS_PACKET(p)
#endif

struct sisusb_usb_data;

struct sisusb_urb_context {	/* urb->context for outbound bulk URBs */
	struct sisusb_usb_data *sisusb;
	int urbindex;
	int *actual_length;
};

struct sisusb_usb_data {
	struct usb_device *sisusb_dev;
	struct usb_interface *interface;
	struct kref kref;
	wait_queue_head_t wait_q;	/* for syncind and timeouts */
	struct mutex lock;	/* general race avoidance */
	unsigned int ifnum;	/* interface number of the USB device */
	int minor;		/* minor (for logging clarity) */
	int isopen;		/* !=0 if open */
	int present;		/* !=0 if device is present on the bus */
	int ready;		/* !=0 if device is ready for userland */
	int numobufs;		/* number of obufs = number of out urbs */
	char *obuf[NUMOBUFS], *ibuf;	/* transfer buffers */
	int obufsize, ibufsize;
	struct urb *sisurbout[NUMOBUFS];
	struct urb *sisurbin;
	unsigned char urbstatus[NUMOBUFS];
	unsigned char completein;
	struct sisusb_urb_context urbout_context[NUMOBUFS];
	unsigned long flagb0;
	unsigned long vrambase;	/* framebuffer base */
	unsigned int vramsize;	/* framebuffer size (bytes) */
	unsigned long mmiobase;
	unsigned int mmiosize;
	unsigned long ioportbase;
	unsigned char devinit;	/* device initialized? */
	unsigned char gfxinit;	/* graphics core initialized? */
	unsigned short chipid, chipvendor;
	unsigned short chiprevision;
#ifdef INCL_SISUSB_CON
	struct SiS_Private *SiS_Pr;
	unsigned long scrbuf;
	unsigned int scrbuf_size;
	int haveconsole, con_first, con_last;
	int havethisconsole[MAX_NR_CONSOLES];
	int textmodedestroyed;
	unsigned int sisusb_num_columns;	/* real number, not vt's idea */
	int cur_start_addr, con_rolled_over;
	int sisusb_cursor_loc, bad_cursor_pos;
	int sisusb_cursor_size_from;
	int sisusb_cursor_size_to;
	int current_font_height, current_font_512;
	int font_backup_size, font_backup_height, font_backup_512;
	char *font_backup;
	int font_slot;
	struct vc_data *sisusb_display_fg;
	int is_gfx;
	int con_blanked;
#endif
};

#define to_sisusb_dev(d) container_of(d, struct sisusb_usb_data, kref)

/* USB transport related */

/* urbstatus */
#define SU_URB_BUSY   1
#define SU_URB_ALLOC  2

/* Endpoints */

#define SISUSB_EP_GFX_IN	0x0e	/* gfx std packet out(0e)/in(8e) */
#define SISUSB_EP_GFX_OUT	0x0e

#define SISUSB_EP_GFX_BULK_OUT	0x01	/* gfx mem bulk out/in */
#define SISUSB_EP_GFX_BULK_IN	0x02	/* ? 2 is "OUT" ? */

#define SISUSB_EP_GFX_LBULK_OUT	0x03	/* gfx large mem bulk out */

#define SISUSB_EP_UNKNOWN_04	0x04	/* ? 4 is "OUT" ? - unused */

#define SISUSB_EP_BRIDGE_IN	0x0d	/* Net2280 out(0d)/in(8d) */
#define SISUSB_EP_BRIDGE_OUT	0x0d

#define SISUSB_TYPE_MEM		0
#define SISUSB_TYPE_IO		1

struct sisusb_packet {
	unsigned short header;
	u32 address;
	u32 data;
} __attribute__ ((__packed__));

#define CLEARPACKET(packet) memset(packet, 0, 10)

/* PCI bridge related */

#define SISUSB_PCI_MEMBASE	0xd0000000
#define SISUSB_PCI_MMIOBASE	0xe4000000
#define SISUSB_PCI_IOPORTBASE	0x0000d000

#define SISUSB_PCI_PSEUDO_MEMBASE	0x10000000
#define SISUSB_PCI_PSEUDO_MMIOBASE	0x20000000
#define SISUSB_PCI_PSEUDO_IOPORTBASE	0x0000d000
#define SISUSB_PCI_PSEUDO_PCIBASE	0x00010000

#define SISUSB_PCI_MMIOSIZE	(128*1024)
#define SISUSB_PCI_PCONFSIZE	0x5c

/* graphics core related */

#define AROFFSET	0x40
#define ARROFFSET	0x41
#define GROFFSET	0x4e
#define SROFFSET	0x44
#define CROFFSET	0x54
#define MISCROFFSET	0x4c
#define MISCWOFFSET	0x42
#define INPUTSTATOFFSET 0x5A
#define PART1OFFSET	0x04
#define PART2OFFSET	0x10
#define PART3OFFSET	0x12
#define PART4OFFSET	0x14
#define PART5OFFSET	0x16
#define CAPTUREOFFSET	0x00
#define VIDEOOFFSET	0x02
#define COLREGOFFSET	0x48
#define PELMASKOFFSET	0x46
#define VGAENABLE	0x43

#define SISAR		SISUSB_PCI_IOPORTBASE + AROFFSET
#define SISARR		SISUSB_PCI_IOPORTBASE + ARROFFSET
#define SISGR		SISUSB_PCI_IOPORTBASE + GROFFSET
#define SISSR		SISUSB_PCI_IOPORTBASE + SROFFSET
#define SISCR		SISUSB_PCI_IOPORTBASE + CROFFSET
#define SISMISCR	SISUSB_PCI_IOPORTBASE + MISCROFFSET
#define SISMISCW	SISUSB_PCI_IOPORTBASE + MISCWOFFSET
#define SISINPSTAT	SISUSB_PCI_IOPORTBASE + INPUTSTATOFFSET
#define SISPART1	SISUSB_PCI_IOPORTBASE + PART1OFFSET
#define SISPART2	SISUSB_PCI_IOPORTBASE + PART2OFFSET
#define SISPART3	SISUSB_PCI_IOPORTBASE + PART3OFFSET
#define SISPART4	SISUSB_PCI_IOPORTBASE + PART4OFFSET
#define SISPART5	SISUSB_PCI_IOPORTBASE + PART5OFFSET
#define SISCAP		SISUSB_PCI_IOPORTBASE + CAPTUREOFFSET
#define SISVID		SISUSB_PCI_IOPORTBASE + VIDEOOFFSET
#define SISCOLIDXR	SISUSB_PCI_IOPORTBASE + COLREGOFFSET - 1
#define SISCOLIDX	SISUSB_PCI_IOPORTBASE + COLREGOFFSET
#define SISCOLDATA	SISUSB_PCI_IOPORTBASE + COLREGOFFSET + 1
#define SISCOL2IDX	SISPART5
#define SISCOL2DATA	SISPART5 + 1
#define SISPEL		SISUSB_PCI_IOPORTBASE + PELMASKOFFSET
#define SISVGAEN	SISUSB_PCI_IOPORTBASE + VGAENABLE
#define SISDACA		SISCOLIDX
#define SISDACD		SISCOLDATA

/* ioctl related */

/* Structure argument for SISUSB_GET_INFO ioctl  */
struct sisusb_info {
	__u32 sisusb_id;	/* for identifying sisusb */
#define SISUSB_ID  0x53495355	/* Identify myself with 'SISU' */
	__u8 sisusb_version;
	__u8 sisusb_revision;
	__u8 sisusb_patchlevel;
	__u8 sisusb_gfxinit;	/* graphics core initialized? */

	__u32 sisusb_vrambase;
	__u32 sisusb_mmiobase;
	__u32 sisusb_iobase;
	__u32 sisusb_pcibase;

	__u32 sisusb_vramsize;	/* framebuffer size in bytes */

	__u32 sisusb_minor;

	__u32 sisusb_fbdevactive;	/* != 0 if framebuffer device active */

	__u32 sisusb_conactive;	/* != 0 if console driver active */

	__u8 sisusb_reserved[28];	/* for future use */
};

struct sisusb_command {
	__u8 operation;		/* see below */
	__u8 data0;		/* operation dependent */
	__u8 data1;		/* operation dependent */
	__u8 data2;		/* operation dependent */
	__u32 data3;		/* operation dependent */
	__u32 data4;		/* for future use */
};

#define SUCMD_GET	0x01	/* for all: data0 = index, data3 = port */
#define SUCMD_SET	0x02	/* data1 = value */
#define SUCMD_SETOR	0x03	/* data1 = or */
#define SUCMD_SETAND	0x04	/* data1 = and */
#define SUCMD_SETANDOR	0x05	/* data1 = and, data2 = or */
#define SUCMD_SETMASK	0x06	/* data1 = data, data2 = mask */

#define SUCMD_CLRSCR	0x07	/* data0:1:2 = length, data3 = address */

#define SUCMD_HANDLETEXTMODE 0x08	/* Reset/destroy text mode */

#define SUCMD_SETMODE	0x09	/* Set a display mode (data3 = SiS mode) */
#define SUCMD_SETVESAMODE 0x0a	/* Set a display mode (data3 = VESA mode) */

#define SISUSB_COMMAND		_IOWR(0xF3,0x3D,struct sisusb_command)
#define SISUSB_GET_CONFIG_SIZE	_IOR(0xF3,0x3E,__u32)
#define SISUSB_GET_CONFIG	_IOR(0xF3,0x3F,struct sisusb_info)

#endif /* SISUSB_H */
