
#ifndef __OSD_TYPES_H__
#define __OSD_TYPES_H__

struct osd_systemid {
	u8 data[OSD_SYSTEMID_LEN];
};

typedef u64 __bitwise osd_id;

struct osd_obj_id {
	osd_id partition;
	osd_id id;
};

static const struct __weak osd_obj_id osd_root_object = {0, 0};

struct osd_attr {
	u32 attr_page;
	u32 attr_id;
	u16 len;		/* byte count of operand */
	void *val_ptr;		/* in network order */
};

#endif /* ndef __OSD_TYPES_H__ */
