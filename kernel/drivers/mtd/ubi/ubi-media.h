


#ifndef __UBI_MEDIA_H__
#define __UBI_MEDIA_H__

#include <asm/byteorder.h>

/* The version of UBI images supported by this implementation */
#define UBI_VERSION 1

/* The highest erase counter value supported by this implementation */
#define UBI_MAX_ERASECOUNTER 0x7FFFFFFF

/* The initial CRC32 value used when calculating CRC checksums */
#define UBI_CRC32_INIT 0xFFFFFFFFU

/* Erase counter header magic number (ASCII "UBI#") */
#define UBI_EC_HDR_MAGIC  0x55424923
/* Volume identifier header magic number (ASCII "UBI!") */
#define UBI_VID_HDR_MAGIC 0x55424921

enum {
	UBI_VID_DYNAMIC = 1,
	UBI_VID_STATIC  = 2
};

enum {
	UBI_VTBL_AUTORESIZE_FLG = 0x01,
};

enum {
	UBI_COMPAT_DELETE   = 1,
	UBI_COMPAT_RO       = 2,
	UBI_COMPAT_PRESERVE = 4,
	UBI_COMPAT_REJECT   = 5
};

/* Sizes of UBI headers */
#define UBI_EC_HDR_SIZE  sizeof(struct ubi_ec_hdr)
#define UBI_VID_HDR_SIZE sizeof(struct ubi_vid_hdr)

/* Sizes of UBI headers without the ending CRC */
#define UBI_EC_HDR_SIZE_CRC  (UBI_EC_HDR_SIZE  - sizeof(__be32))
#define UBI_VID_HDR_SIZE_CRC (UBI_VID_HDR_SIZE - sizeof(__be32))

struct ubi_ec_hdr {
	__be32  magic;
	__u8    version;
	__u8    padding1[3];
	__be64  ec; /* Warning: the current limit is 31-bit anyway! */
	__be32  vid_hdr_offset;
	__be32  data_offset;
	__be32  image_seq;
	__u8    padding2[32];
	__be32  hdr_crc;
} __attribute__ ((packed));

struct ubi_vid_hdr {
	__be32  magic;
	__u8    version;
	__u8    vol_type;
	__u8    copy_flag;
	__u8    compat;
	__be32  vol_id;
	__be32  lnum;
	__u8    padding1[4];
	__be32  data_size;
	__be32  used_ebs;
	__be32  data_pad;
	__be32  data_crc;
	__u8    padding2[4];
	__be64  sqnum;
	__u8    padding3[12];
	__be32  hdr_crc;
} __attribute__ ((packed));

/* Internal UBI volumes count */
#define UBI_INT_VOL_COUNT 1

#define UBI_INTERNAL_VOL_START (0x7FFFFFFF - 4096)

/* The layout volume contains the volume table */

#define UBI_LAYOUT_VOLUME_ID     UBI_INTERNAL_VOL_START
#define UBI_LAYOUT_VOLUME_TYPE   UBI_VID_DYNAMIC
#define UBI_LAYOUT_VOLUME_ALIGN  1
#define UBI_LAYOUT_VOLUME_EBS    2
#define UBI_LAYOUT_VOLUME_NAME   "layout volume"
#define UBI_LAYOUT_VOLUME_COMPAT UBI_COMPAT_REJECT

/* The maximum number of volumes per one UBI device */
#define UBI_MAX_VOLUMES 128

/* The maximum volume name length */
#define UBI_VOL_NAME_MAX 127

/* Size of the volume table record */
#define UBI_VTBL_RECORD_SIZE sizeof(struct ubi_vtbl_record)

/* Size of the volume table record without the ending CRC */
#define UBI_VTBL_RECORD_SIZE_CRC (UBI_VTBL_RECORD_SIZE - sizeof(__be32))

struct ubi_vtbl_record {
	__be32  reserved_pebs;
	__be32  alignment;
	__be32  data_pad;
	__u8    vol_type;
	__u8    upd_marker;
	__be16  name_len;
	__u8    name[UBI_VOL_NAME_MAX+1];
	__u8    flags;
	__u8    padding[23];
	__be32  crc;
} __attribute__ ((packed));

#endif /* !__UBI_MEDIA_H__ */
