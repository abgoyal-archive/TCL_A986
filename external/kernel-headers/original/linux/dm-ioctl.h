
#ifndef _LINUX_DM_IOCTL_V4_H
#define _LINUX_DM_IOCTL_V4_H

#include <linux/types.h>

#define DM_DIR "mapper"		/* Slashes not supported */
#define DM_MAX_TYPE_NAME 16
#define DM_NAME_LEN 128
#define DM_UUID_LEN 129


struct dm_ioctl {
	/*
	 * The version number is made up of three parts:
	 * major - no backward or forward compatibility,
	 * minor - only backwards compatible,
	 * patch - both backwards and forwards compatible.
	 *
	 * All clients of the ioctl interface should fill in the
	 * version number of the interface that they were
	 * compiled with.
	 *
	 * All recognised ioctl commands (ie. those that don't
	 * return -ENOTTY) fill out this field, even if the
	 * command failed.
	 */
	uint32_t version[3];	/* in/out */
	uint32_t data_size;	/* total size of data passed in
				 * including this struct */

	uint32_t data_start;	/* offset to start of data
				 * relative to start of this struct */

	uint32_t target_count;	/* in/out */
	int32_t open_count;	/* out */
	uint32_t flags;		/* in/out */
	uint32_t event_nr;      	/* in/out */
	uint32_t padding;

	uint64_t dev;		/* in/out */

	char name[DM_NAME_LEN];	/* device name */
	char uuid[DM_UUID_LEN];	/* unique identifier for
				 * the block device */
	char data[7];		/* padding or data */
};

struct dm_target_spec {
	uint64_t sector_start;
	uint64_t length;
	int32_t status;		/* used when reading from kernel only */

	/*
	 * Location of the next dm_target_spec.
	 * - When specifying targets on a DM_TABLE_LOAD command, this value is
	 *   the number of bytes from the start of the "current" dm_target_spec
	 *   to the start of the "next" dm_target_spec.
	 * - When retrieving targets on a DM_TABLE_STATUS command, this value
	 *   is the number of bytes from the start of the first dm_target_spec
	 *   (that follows the dm_ioctl struct) to the start of the "next"
	 *   dm_target_spec.
	 */
	uint32_t next;

	char target_type[DM_MAX_TYPE_NAME];

	/*
	 * Parameter string starts immediately after this object.
	 * Be careful to add padding after string to ensure correct
	 * alignment of subsequent dm_target_spec.
	 */
};

struct dm_target_deps {
	uint32_t count;	/* Array size */
	uint32_t padding;	/* unused */
	uint64_t dev[0];	/* out */
};

struct dm_name_list {
	uint64_t dev;
	uint32_t next;		/* offset to the next record from
				   the _start_ of this */
	char name[0];
};

struct dm_target_versions {
        uint32_t next;
        uint32_t version[3];

        char name[0];
};

struct dm_target_msg {
	uint64_t sector;	/* Device sector */

	char message[0];
};

enum {
	/* Top level cmds */
	DM_VERSION_CMD = 0,
	DM_REMOVE_ALL_CMD,
	DM_LIST_DEVICES_CMD,

	/* device level cmds */
	DM_DEV_CREATE_CMD,
	DM_DEV_REMOVE_CMD,
	DM_DEV_RENAME_CMD,
	DM_DEV_SUSPEND_CMD,
	DM_DEV_STATUS_CMD,
	DM_DEV_WAIT_CMD,

	/* Table level cmds */
	DM_TABLE_LOAD_CMD,
	DM_TABLE_CLEAR_CMD,
	DM_TABLE_DEPS_CMD,
	DM_TABLE_STATUS_CMD,

	/* Added later */
	DM_LIST_VERSIONS_CMD,
	DM_TARGET_MSG_CMD,
	DM_DEV_SET_GEOMETRY_CMD
};

#define DM_IOCTL 0xfd

#define DM_VERSION       _IOWR(DM_IOCTL, DM_VERSION_CMD, struct dm_ioctl)
#define DM_REMOVE_ALL    _IOWR(DM_IOCTL, DM_REMOVE_ALL_CMD, struct dm_ioctl)
#define DM_LIST_DEVICES  _IOWR(DM_IOCTL, DM_LIST_DEVICES_CMD, struct dm_ioctl)

#define DM_DEV_CREATE    _IOWR(DM_IOCTL, DM_DEV_CREATE_CMD, struct dm_ioctl)
#define DM_DEV_REMOVE    _IOWR(DM_IOCTL, DM_DEV_REMOVE_CMD, struct dm_ioctl)
#define DM_DEV_RENAME    _IOWR(DM_IOCTL, DM_DEV_RENAME_CMD, struct dm_ioctl)
#define DM_DEV_SUSPEND   _IOWR(DM_IOCTL, DM_DEV_SUSPEND_CMD, struct dm_ioctl)
#define DM_DEV_STATUS    _IOWR(DM_IOCTL, DM_DEV_STATUS_CMD, struct dm_ioctl)
#define DM_DEV_WAIT      _IOWR(DM_IOCTL, DM_DEV_WAIT_CMD, struct dm_ioctl)

#define DM_TABLE_LOAD    _IOWR(DM_IOCTL, DM_TABLE_LOAD_CMD, struct dm_ioctl)
#define DM_TABLE_CLEAR   _IOWR(DM_IOCTL, DM_TABLE_CLEAR_CMD, struct dm_ioctl)
#define DM_TABLE_DEPS    _IOWR(DM_IOCTL, DM_TABLE_DEPS_CMD, struct dm_ioctl)
#define DM_TABLE_STATUS  _IOWR(DM_IOCTL, DM_TABLE_STATUS_CMD, struct dm_ioctl)

#define DM_LIST_VERSIONS _IOWR(DM_IOCTL, DM_LIST_VERSIONS_CMD, struct dm_ioctl)

#define DM_TARGET_MSG	 _IOWR(DM_IOCTL, DM_TARGET_MSG_CMD, struct dm_ioctl)
#define DM_DEV_SET_GEOMETRY	_IOWR(DM_IOCTL, DM_DEV_SET_GEOMETRY_CMD, struct dm_ioctl)

#define DM_VERSION_MAJOR	4
#define DM_VERSION_MINOR	14
#define DM_VERSION_PATCHLEVEL	0
#define DM_VERSION_EXTRA	"-ioctl (2008-04-23)"

/* Status bits */
#define DM_READONLY_FLAG	(1 << 0) /* In/Out */
#define DM_SUSPEND_FLAG		(1 << 1) /* In/Out */
#define DM_PERSISTENT_DEV_FLAG	(1 << 3) /* In */

#define DM_STATUS_TABLE_FLAG	(1 << 4) /* In */

#define DM_ACTIVE_PRESENT_FLAG   (1 << 5) /* Out */
#define DM_INACTIVE_PRESENT_FLAG (1 << 6) /* Out */

#define DM_BUFFER_FULL_FLAG	(1 << 8) /* Out */

#define DM_SKIP_BDGET_FLAG	(1 << 9) /* In */

#define DM_SKIP_LOCKFS_FLAG	(1 << 10) /* In */

#define DM_NOFLUSH_FLAG		(1 << 11) /* In */

#endif				/* _LINUX_DM_IOCTL_H */
