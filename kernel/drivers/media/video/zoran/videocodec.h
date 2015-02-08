

/* =================== */
/* general description */
/* =================== */



/* ========================================== */
/* description of the videocodec_io structure */
/* ========================================== */


/* ============== */
/* user interface */
/* ============== */



/* =============================================== */
/* special defines for the videocodec_io structure */
/* =============================================== */

#ifndef __LINUX_VIDEOCODEC_H
#define __LINUX_VIDEOCODEC_H

#include <linux/videodev2.h>

#define CODEC_DO_COMPRESSION 0
#define CODEC_DO_EXPANSION   1

/* this are the current codec flags I think they are needed */
/*  -> type value in structure */
#define CODEC_FLAG_JPEG      0x00000001L	// JPEG codec
#define CODEC_FLAG_MPEG      0x00000002L	// MPEG1/2/4 codec
#define CODEC_FLAG_DIVX      0x00000004L	// DIVX codec
#define CODEC_FLAG_WAVELET   0x00000008L	// WAVELET codec
					  // room for other types

#define CODEC_FLAG_MAGIC     0x00000800L	// magic key must match
#define CODEC_FLAG_HARDWARE  0x00001000L	// is a hardware codec
#define CODEC_FLAG_VFE       0x00002000L	// has direct video frontend
#define CODEC_FLAG_ENCODER   0x00004000L	// compression capability
#define CODEC_FLAG_DECODER   0x00008000L	// decompression capability
#define CODEC_FLAG_NEEDIRQ   0x00010000L	// needs irq handling
#define CODEC_FLAG_RDWRPIC   0x00020000L	// handles picture I/O

/* a list of modes, some are just examples (is there any HW?) */
#define CODEC_MODE_BJPG      0x0001	// Baseline JPEG
#define CODEC_MODE_LJPG      0x0002	// Lossless JPEG
#define CODEC_MODE_MPEG1     0x0003	// MPEG 1
#define CODEC_MODE_MPEG2     0x0004	// MPEG 2
#define CODEC_MODE_MPEG4     0x0005	// MPEG 4
#define CODEC_MODE_MSDIVX    0x0006	// MS DivX
#define CODEC_MODE_ODIVX     0x0007	// Open DivX
#define CODEC_MODE_WAVELET   0x0008	// Wavelet

/* this are the current codec types I want to implement */
/*  -> type value in structure */
#define CODEC_TYPE_NONE    0
#define CODEC_TYPE_L64702  1
#define CODEC_TYPE_ZR36050 2
#define CODEC_TYPE_ZR36016 3
#define CODEC_TYPE_ZR36060 4

/* the type of data may be enhanced by future implementations (data-fn.'s) */
/*  -> used in command                                                     */
#define CODEC_G_STATUS         0x0000	/* codec status (query only) */
#define CODEC_S_CODEC_MODE     0x0001	/* codec mode (baseline JPEG, MPEG1,... */
#define CODEC_G_CODEC_MODE     0x8001
#define CODEC_S_VFE            0x0002	/* additional video frontend setup */
#define CODEC_G_VFE            0x8002
#define CODEC_S_MMAP           0x0003	/* MMAP setup (if available) */

#define CODEC_S_JPEG_TDS_BYTE  0x0010	/* target data size in bytes */
#define CODEC_G_JPEG_TDS_BYTE  0x8010
#define CODEC_S_JPEG_SCALE     0x0011	/* scaling factor for quant. tables */
#define CODEC_G_JPEG_SCALE     0x8011
#define CODEC_S_JPEG_HDT_DATA  0x0018	/* huffman-tables */
#define CODEC_G_JPEG_HDT_DATA  0x8018
#define CODEC_S_JPEG_QDT_DATA  0x0019	/* quantizing-tables */
#define CODEC_G_JPEG_QDT_DATA  0x8019
#define CODEC_S_JPEG_APP_DATA  0x001A	/* APP marker */
#define CODEC_G_JPEG_APP_DATA  0x801A
#define CODEC_S_JPEG_COM_DATA  0x001B	/* COM marker */
#define CODEC_G_JPEG_COM_DATA  0x801B

#define CODEC_S_PRIVATE        0x1000	/* "private" commands start here */
#define CODEC_G_PRIVATE        0x9000

#define CODEC_G_FLAG           0x8000	/* this is how 'get' is detected */

/* types of transfer, directly user space or a kernel buffer (image-fn.'s) */
/*  -> used in get_image, put_image                                        */
#define CODEC_TRANSFER_KERNEL 0	/* use "memcopy" */
#define CODEC_TRANSFER_USER   1	/* use "to/from_user" */


/* ========================= */
/* the structures itself ... */
/* ========================= */

struct vfe_polarity {
	unsigned int vsync_pol:1;
	unsigned int hsync_pol:1;
	unsigned int field_pol:1;
	unsigned int blank_pol:1;
	unsigned int subimg_pol:1;
	unsigned int poe_pol:1;
	unsigned int pvalid_pol:1;
	unsigned int vclk_pol:1;
};

struct vfe_settings {
	__u32 x, y;		/* Offsets into image */
	__u32 width, height;	/* Area to capture */
	__u16 decimation;	/* Decimation divider */
	__u16 flags;		/* Flags for capture */
	__u16 quality;		/* quality of the video */
};

struct tvnorm {
	u16 Wt, Wa, HStart, HSyncStart, Ht, Ha, VStart;
};

struct jpeg_com_marker {
	int len; /* number of usable bytes in data */
	char data[60];
};

struct jpeg_app_marker {
	int appn; /* number app segment */
	int len; /* number of usable bytes in data */
	char data[60];
};

struct videocodec {
	struct module *owner;
	/* -- filled in by slave device during register -- */
	char name[32];
	unsigned long magic;	/* may be used for client<->master attaching */
	unsigned long flags;	/* functionality flags */
	unsigned int type;	/* codec type */

	/* -- these is filled in later during master device attach -- */

	struct videocodec_master *master_data;

	/* -- these are filled in by the slave device during register -- */

	void *data;		/* private slave data */

	/* attach/detach client functions (indirect call) */
	int (*setup) (struct videocodec * codec);
	int (*unset) (struct videocodec * codec);

	/* main functions, every client needs them for sure! */
	// set compression or decompression (or freeze, stop, standby, etc)
	int (*set_mode) (struct videocodec * codec,
			 int mode);
	// setup picture size and norm (for the codec's video frontend)
	int (*set_video) (struct videocodec * codec,
			  struct tvnorm * norm,
			  struct vfe_settings * cap,
			  struct vfe_polarity * pol);
	// other control commands, also mmap setup etc.
	int (*control) (struct videocodec * codec,
			int type,
			int size,
			void *data);

	/* additional setup/query/processing (may be NULL pointer) */
	// interrupt setup / handling (for irq's delivered by master)
	int (*setup_interrupt) (struct videocodec * codec,
				long mode);
	int (*handle_interrupt) (struct videocodec * codec,
				 int source,
				 long flag);
	// picture interface (if any)
	long (*put_image) (struct videocodec * codec,
			   int tr_type,
			   int block,
			   long *fr_num,
			   long *flag,
			   long size,
			   void *buf);
	long (*get_image) (struct videocodec * codec,
			   int tr_type,
			   int block,
			   long *fr_num,
			   long *flag,
			   long size,
			   void *buf);
};

struct videocodec_master {
	/* -- filled in by master device for registration -- */
	char name[32];
	unsigned long magic;	/* may be used for client<->master attaching */
	unsigned long flags;	/* functionality flags */
	unsigned int type;	/* master type */

	void *data;		/* private master data */

	 __u32(*readreg) (struct videocodec * codec,
			  __u16 reg);
	void (*writereg) (struct videocodec * codec,
			  __u16 reg,
			  __u32 value);
};


/* ================================================= */
/* function prototypes of the master/slave interface */
/* ================================================= */

/* attach and detach commands for the master */
// * master structure needs to be kmalloc'ed before calling attach
//   and free'd after calling detach
// * returns pointer on success, NULL on failure
extern struct videocodec *videocodec_attach(struct videocodec_master *);
// * 0 on success, <0 (errno) on failure
extern int videocodec_detach(struct videocodec *);

/* register and unregister commands for the slaves */
// * 0 on success, <0 (errno) on failure
extern int videocodec_register(const struct videocodec *);
// * 0 on success, <0 (errno) on failure
extern int videocodec_unregister(const struct videocodec *);

/* the other calls are directly done via the videocodec structure! */

#endif				/*ifndef __LINUX_VIDEOCODEC_H */
