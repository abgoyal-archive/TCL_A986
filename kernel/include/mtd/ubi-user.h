

#ifndef __UBI_USER_H__
#define __UBI_USER_H__

#include <linux/types.h>


#define UBI_VOL_NUM_AUTO (-1)
#define UBI_DEV_NUM_AUTO (-1)

/* Maximum volume name length */
#define UBI_MAX_VOLUME_NAME 127

/* ioctl commands of UBI character devices */

#define UBI_IOC_MAGIC 'o'

/* Create an UBI volume */
#define UBI_IOCMKVOL _IOW(UBI_IOC_MAGIC, 0, struct ubi_mkvol_req)
/* Remove an UBI volume */
#define UBI_IOCRMVOL _IOW(UBI_IOC_MAGIC, 1, __s32)
/* Re-size an UBI volume */
#define UBI_IOCRSVOL _IOW(UBI_IOC_MAGIC, 2, struct ubi_rsvol_req)
/* Re-name volumes */
#define UBI_IOCRNVOL _IOW(UBI_IOC_MAGIC, 3, struct ubi_rnvol_req)

/* ioctl commands of the UBI control character device */

#define UBI_CTRL_IOC_MAGIC 'o'

/* Attach an MTD device */
#define UBI_IOCATT _IOW(UBI_CTRL_IOC_MAGIC, 64, struct ubi_attach_req)
/* Detach an MTD device */
#define UBI_IOCDET _IOW(UBI_CTRL_IOC_MAGIC, 65, __s32)

/* ioctl commands of UBI volume character devices */

#define UBI_VOL_IOC_MAGIC 'O'

/* Start UBI volume update */
#define UBI_IOCVOLUP _IOW(UBI_VOL_IOC_MAGIC, 0, __s64)
/* LEB erasure command, used for debugging, disabled by default */
#define UBI_IOCEBER _IOW(UBI_VOL_IOC_MAGIC, 1, __s32)
/* Atomic LEB change command */
#define UBI_IOCEBCH _IOW(UBI_VOL_IOC_MAGIC, 2, __s32)
/* Map LEB command */
#define UBI_IOCEBMAP _IOW(UBI_VOL_IOC_MAGIC, 3, struct ubi_map_req)
/* Unmap LEB command */
#define UBI_IOCEBUNMAP _IOW(UBI_VOL_IOC_MAGIC, 4, __s32)
/* Check if LEB is mapped command */
#define UBI_IOCEBISMAP _IOR(UBI_VOL_IOC_MAGIC, 5, __s32)
/* Set an UBI volume property */
#define UBI_IOCSETPROP _IOW(UBI_VOL_IOC_MAGIC, 6, struct ubi_set_prop_req)

/* Maximum MTD device name length supported by UBI */
#define MAX_UBI_MTD_NAME_LEN 127

/* Maximum amount of UBI volumes that can be re-named at one go */
#define UBI_MAX_RNVOL 32

enum {
	UBI_LONGTERM  = 1,
	UBI_SHORTTERM = 2,
	UBI_UNKNOWN   = 3,
};

enum {
	UBI_DYNAMIC_VOLUME = 3,
	UBI_STATIC_VOLUME  = 4,
};

enum {
       UBI_PROP_DIRECT_WRITE = 1,
};

struct ubi_attach_req {
	__s32 ubi_num;
	__s32 mtd_num;
	__s32 vid_hdr_offset;
	__s8 padding[12];
};

struct ubi_mkvol_req {
	__s32 vol_id;
	__s32 alignment;
	__s64 bytes;
	__s8 vol_type;
	__s8 padding1;
	__s16 name_len;
	__s8 padding2[4];
	char name[UBI_MAX_VOLUME_NAME + 1];
} __attribute__ ((packed));

struct ubi_rsvol_req {
	__s64 bytes;
	__s32 vol_id;
} __attribute__ ((packed));

struct ubi_rnvol_req {
	__s32 count;
	__s8 padding1[12];
	struct {
		__s32 vol_id;
		__s16 name_len;
		__s8  padding2[2];
		char    name[UBI_MAX_VOLUME_NAME + 1];
	} ents[UBI_MAX_RNVOL];
} __attribute__ ((packed));

struct ubi_leb_change_req {
	__s32 lnum;
	__s32 bytes;
	__s8  dtype;
	__s8  padding[7];
} __attribute__ ((packed));

struct ubi_map_req {
	__s32 lnum;
	__s8  dtype;
	__s8  padding[3];
} __attribute__ ((packed));


struct ubi_set_prop_req {
       __u8  property;
       __u8  padding[7];
       __u64 value;
}  __attribute__ ((packed));

#endif /* __UBI_USER_H__ */
