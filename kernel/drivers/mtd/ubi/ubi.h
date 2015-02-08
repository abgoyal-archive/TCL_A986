

#ifndef __UBI_UBI_H__
#define __UBI_UBI_H__

#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/notifier.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/ubi.h>

#include "ubi-media.h"
#include "scan.h"
#include "debug.h"

/* Maximum number of supported UBI devices */
#define UBI_MAX_DEVICES 32

/* UBI name used for character devices, sysfs, etc */
#define UBI_NAME_STR "ubi"

/* Normal UBI messages */
#define ubi_msg(fmt, ...) printk(KERN_NOTICE "UBI: " fmt "\n", ##__VA_ARGS__)
/* UBI warning messages */
#define ubi_warn(fmt, ...) printk(KERN_WARNING "UBI warning: %s: " fmt "\n", \
				  __func__, ##__VA_ARGS__)
/* UBI error messages */
#define ubi_err(fmt, ...) printk(KERN_ERR "UBI error: %s: " fmt "\n", \
				 __func__, ##__VA_ARGS__)

/* Lowest number PEBs reserved for bad PEB handling */
#define MIN_RESEVED_PEBS 2

/* Background thread name pattern */
#define UBI_BGT_NAME_PATTERN "ubi_bgt%dd"

/* This marker in the EBA table means that the LEB is um-mapped */
#define UBI_LEB_UNMAPPED -1

#define UBI_IO_RETRIES 3

#define UBI_PROT_QUEUE_LEN 10

enum {
	UBI_IO_PEB_EMPTY = 1,
	UBI_IO_PEB_FREE,
	UBI_IO_BAD_EC_HDR,
	UBI_IO_BAD_VID_HDR,
	UBI_IO_BITFLIPS
};

enum {
	MOVE_CANCEL_RACE = 1,
	MOVE_SOURCE_RD_ERR,
	MOVE_TARGET_RD_ERR,
	MOVE_TARGET_WR_ERR,
	MOVE_CANCEL_BITFLIPS,
};

struct ubi_wl_entry {
	union {
		struct rb_node rb;
		struct list_head list;
	} u;
	int ec;
	int pnum;
};

struct ubi_ltree_entry {
	struct rb_node rb;
	int vol_id;
	int lnum;
	int users;
	struct rw_semaphore mutex;
};

struct ubi_rename_entry {
	int new_name_len;
	char new_name[UBI_VOL_NAME_MAX + 1];
	int remove;
	struct ubi_volume_desc *desc;
	struct list_head list;
};

struct ubi_volume_desc;

struct ubi_volume {
	struct device dev;
	struct cdev cdev;
	struct ubi_device *ubi;
	int vol_id;
	int ref_count;
	int readers;
	int writers;
	int exclusive;

	int reserved_pebs;
	int vol_type;
	int usable_leb_size;
	int used_ebs;
	int last_eb_bytes;
	long long used_bytes;
	int alignment;
	int data_pad;
	int name_len;
	char name[UBI_VOL_NAME_MAX + 1];

	int upd_ebs;
	int ch_lnum;
	int ch_dtype;
	long long upd_bytes;
	long long upd_received;
	void *upd_buf;

	int *eba_tbl;
	unsigned int checked:1;
	unsigned int corrupted:1;
	unsigned int upd_marker:1;
	unsigned int updating:1;
	unsigned int changing_leb:1;
	unsigned int direct_writes:1;
};

struct ubi_volume_desc {
	struct ubi_volume *vol;
	int mode;
};

struct ubi_wl_entry;

struct ubi_device {
	struct cdev cdev;
	struct device dev;
	int ubi_num;
	char ubi_name[sizeof(UBI_NAME_STR)+5];
	int vol_count;
	struct ubi_volume *volumes[UBI_MAX_VOLUMES+UBI_INT_VOL_COUNT];
	spinlock_t volumes_lock;
	int ref_count;
	int image_seq;

	int rsvd_pebs;
	int avail_pebs;
	int beb_rsvd_pebs;
	int beb_rsvd_level;

	int autoresize_vol_id;
	int vtbl_slots;
	int vtbl_size;
	struct ubi_vtbl_record *vtbl;
	struct mutex device_mutex;

	int max_ec;
	/* Note, mean_ec is not updated run-time - should be fixed */
	int mean_ec;

	/* EBA sub-system's stuff */
	unsigned long long global_sqnum;
	spinlock_t ltree_lock;
	struct rb_root ltree;
	struct mutex alc_mutex;

	/* Wear-leveling sub-system's stuff */
	struct rb_root used;
	struct rb_root erroneous;
	struct rb_root free;
	struct rb_root scrub;
	struct list_head pq[UBI_PROT_QUEUE_LEN];
	int pq_head;
	spinlock_t wl_lock;
	struct mutex move_mutex;
	struct rw_semaphore work_sem;
	int wl_scheduled;
	struct ubi_wl_entry **lookuptbl;
	struct ubi_wl_entry *move_from;
	struct ubi_wl_entry *move_to;
	int move_to_put;
	struct list_head works;
	int works_count;
	struct task_struct *bgt_thread;
	int thread_enabled;
	char bgt_name[sizeof(UBI_BGT_NAME_PATTERN)+2];

	/* I/O sub-system's stuff */
	long long flash_size;
	int peb_count;
	int peb_size;
	int bad_peb_count;
	int good_peb_count;
	int erroneous_peb_count;
	int max_erroneous;
	int min_io_size;
	int hdrs_min_io_size;
	int ro_mode;
	int leb_size;
	int leb_start;
	int ec_hdr_alsize;
	int vid_hdr_alsize;
	int vid_hdr_offset;
	int vid_hdr_aloffset;
	int vid_hdr_shift;
	unsigned int bad_allowed:1;
	unsigned int nor_flash:1;
	struct mtd_info *mtd;

	void *peb_buf1;
	void *peb_buf2;
	struct mutex buf_mutex;
	struct mutex ckvol_mutex;
#ifdef CONFIG_MTD_UBI_DEBUG_PARANOID
	void *dbg_peb_buf;
	struct mutex dbg_buf_mutex;
#endif
};

extern struct kmem_cache *ubi_wl_entry_slab;
extern const struct file_operations ubi_ctrl_cdev_operations;
extern const struct file_operations ubi_cdev_operations;
extern const struct file_operations ubi_vol_cdev_operations;
extern struct class *ubi_class;
extern struct mutex ubi_devices_mutex;
extern struct blocking_notifier_head ubi_notifiers;

/* vtbl.c */
int ubi_change_vtbl_record(struct ubi_device *ubi, int idx,
			   struct ubi_vtbl_record *vtbl_rec);
int ubi_vtbl_rename_volumes(struct ubi_device *ubi,
			    struct list_head *rename_list);
int ubi_read_volume_table(struct ubi_device *ubi, struct ubi_scan_info *si);

/* vmt.c */
int ubi_create_volume(struct ubi_device *ubi, struct ubi_mkvol_req *req);
int ubi_remove_volume(struct ubi_volume_desc *desc, int no_vtbl);
int ubi_resize_volume(struct ubi_volume_desc *desc, int reserved_pebs);
int ubi_rename_volumes(struct ubi_device *ubi, struct list_head *rename_list);
int ubi_add_volume(struct ubi_device *ubi, struct ubi_volume *vol);
void ubi_free_volume(struct ubi_device *ubi, struct ubi_volume *vol);

/* upd.c */
int ubi_start_update(struct ubi_device *ubi, struct ubi_volume *vol,
		     long long bytes);
int ubi_more_update_data(struct ubi_device *ubi, struct ubi_volume *vol,
			 const void __user *buf, int count);
int ubi_start_leb_change(struct ubi_device *ubi, struct ubi_volume *vol,
			 const struct ubi_leb_change_req *req);
int ubi_more_leb_change_data(struct ubi_device *ubi, struct ubi_volume *vol,
			     const void __user *buf, int count);

/* misc.c */
int ubi_calc_data_len(const struct ubi_device *ubi, const void *buf,
		      int length);
int ubi_check_volume(struct ubi_device *ubi, int vol_id);
void ubi_calculate_reserved(struct ubi_device *ubi);

/* eba.c */
int ubi_eba_unmap_leb(struct ubi_device *ubi, struct ubi_volume *vol,
		      int lnum);
int ubi_eba_read_leb(struct ubi_device *ubi, struct ubi_volume *vol, int lnum,
		     void *buf, int offset, int len, int check);
int ubi_eba_write_leb(struct ubi_device *ubi, struct ubi_volume *vol, int lnum,
		      const void *buf, int offset, int len, int dtype);
int ubi_eba_write_leb_st(struct ubi_device *ubi, struct ubi_volume *vol,
			 int lnum, const void *buf, int len, int dtype,
			 int used_ebs);
int ubi_eba_atomic_leb_change(struct ubi_device *ubi, struct ubi_volume *vol,
			      int lnum, const void *buf, int len, int dtype);
int ubi_eba_copy_leb(struct ubi_device *ubi, int from, int to,
		     struct ubi_vid_hdr *vid_hdr);
int ubi_eba_init_scan(struct ubi_device *ubi, struct ubi_scan_info *si);

/* wl.c */
int ubi_wl_get_peb(struct ubi_device *ubi, int dtype);
int ubi_wl_put_peb(struct ubi_device *ubi, int pnum, int torture);
int ubi_wl_flush(struct ubi_device *ubi);
int ubi_wl_scrub_peb(struct ubi_device *ubi, int pnum);
int ubi_wl_init_scan(struct ubi_device *ubi, struct ubi_scan_info *si);
void ubi_wl_close(struct ubi_device *ubi);
int ubi_thread(void *u);

/* io.c */
int ubi_io_read(const struct ubi_device *ubi, void *buf, int pnum, int offset,
		int len);
int ubi_io_write(struct ubi_device *ubi, const void *buf, int pnum, int offset,
		 int len);
int ubi_io_sync_erase(struct ubi_device *ubi, int pnum, int torture);
int ubi_io_is_bad(const struct ubi_device *ubi, int pnum);
int ubi_io_mark_bad(const struct ubi_device *ubi, int pnum);
int ubi_io_read_ec_hdr(struct ubi_device *ubi, int pnum,
		       struct ubi_ec_hdr *ec_hdr, int verbose);
int ubi_io_write_ec_hdr(struct ubi_device *ubi, int pnum,
			struct ubi_ec_hdr *ec_hdr);
int ubi_io_read_vid_hdr(struct ubi_device *ubi, int pnum,
			struct ubi_vid_hdr *vid_hdr, int verbose);
int ubi_io_write_vid_hdr(struct ubi_device *ubi, int pnum,
			 struct ubi_vid_hdr *vid_hdr);

/* build.c */
int ubi_attach_mtd_dev(struct mtd_info *mtd, int ubi_num, int vid_hdr_offset);
int ubi_detach_mtd_dev(int ubi_num, int anyway);
struct ubi_device *ubi_get_device(int ubi_num);
void ubi_put_device(struct ubi_device *ubi);
struct ubi_device *ubi_get_by_major(int major);
int ubi_major2num(int major);
int ubi_volume_notify(struct ubi_device *ubi, struct ubi_volume *vol,
		      int ntype);
int ubi_notify_all(struct ubi_device *ubi, int ntype,
		   struct notifier_block *nb);
int ubi_enumerate_volumes(struct notifier_block *nb);

/* kapi.c */
void ubi_do_get_device_info(struct ubi_device *ubi, struct ubi_device_info *di);
void ubi_do_get_volume_info(struct ubi_device *ubi, struct ubi_volume *vol,
			    struct ubi_volume_info *vi);

#define ubi_rb_for_each_entry(rb, pos, root, member)                         \
	for (rb = rb_first(root),                                            \
	     pos = (rb ? container_of(rb, typeof(*pos), member) : NULL);     \
	     rb;                                                             \
	     rb = rb_next(rb),                                               \
	     pos = (rb ? container_of(rb, typeof(*pos), member) : NULL))

static inline struct ubi_vid_hdr *
ubi_zalloc_vid_hdr(const struct ubi_device *ubi, gfp_t gfp_flags)
{
	void *vid_hdr;

	vid_hdr = kzalloc(ubi->vid_hdr_alsize, gfp_flags);
	if (!vid_hdr)
		return NULL;

	/*
	 * VID headers may be stored at un-aligned flash offsets, so we shift
	 * the pointer.
	 */
	return vid_hdr + ubi->vid_hdr_shift;
}

static inline void ubi_free_vid_hdr(const struct ubi_device *ubi,
				    struct ubi_vid_hdr *vid_hdr)
{
	void *p = vid_hdr;

	if (!p)
		return;

	kfree(p - ubi->vid_hdr_shift);
}

static inline int ubi_io_read_data(const struct ubi_device *ubi, void *buf,
				   int pnum, int offset, int len)
{
	ubi_assert(offset >= 0);
	return ubi_io_read(ubi, buf, pnum, offset + ubi->leb_start, len);
}

static inline int ubi_io_write_data(struct ubi_device *ubi, const void *buf,
				    int pnum, int offset, int len)
{
	ubi_assert(offset >= 0);
	return ubi_io_write(ubi, buf, pnum, offset + ubi->leb_start, len);
}

static inline void ubi_ro_mode(struct ubi_device *ubi)
{
	if (!ubi->ro_mode) {
		ubi->ro_mode = 1;
		ubi_warn("switch to read-only mode");
	}
}

static inline int vol_id2idx(const struct ubi_device *ubi, int vol_id)
{
	if (vol_id >= UBI_INTERNAL_VOL_START)
		return vol_id - UBI_INTERNAL_VOL_START + ubi->vtbl_slots;
	else
		return vol_id;
}

static inline int idx2vol_id(const struct ubi_device *ubi, int idx)
{
	if (idx >= ubi->vtbl_slots)
		return idx - ubi->vtbl_slots + UBI_INTERNAL_VOL_START;
	else
		return idx;
}

#endif /* !__UBI_UBI_H__ */
