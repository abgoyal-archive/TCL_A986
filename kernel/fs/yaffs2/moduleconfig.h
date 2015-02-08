

#ifndef __YAFFS_CONFIG_H__
#define __YAFFS_CONFIG_H__

#ifdef YAFFS_OUT_OF_TREE

/* DO NOT UNSET THESE THREE. YAFFS2 will not compile if you do. */
#define CONFIG_YAFFS_FS
#define CONFIG_YAFFS_YAFFS1
#define CONFIG_YAFFS_YAFFS2

/* These options are independent of each other.  Select those that matter. */

/* Default: Not selected */
/* Meaning: Yaffs does its own ECC, rather than using MTD ECC */
/* #define CONFIG_YAFFS_DOES_ECC */

/* Default: Selected */
/* Meaning: Yaffs does its own ECC on tags for packed tags rather than use mtd */
#define CONFIG_YAFFS_DOES_TAGS_ECC

/* Default: Not selected */
/* Meaning: ECC byte order is 'wrong'.  Only meaningful if */
/*          CONFIG_YAFFS_DOES_ECC is set */
/* #define CONFIG_YAFFS_ECC_WRONG_ORDER */

/* Default: Not selected */
/* #define CONFIG_YAFFS_ALWAYS_CHECK_CHUNK_ERASED */

/* Default: Not Selected */
/* Meaning: At mount automatically empty all files from lost and found. */
/* This is done to fix an old problem where rmdir was not checking for an */
/* empty directory. This can also be achieved with a mount option. */
#define CONFIG_YAFFS_EMPTY_LOST_AND_FOUND

/* Default: Selected */
/* Meaning: Cache short names, taking more RAM, but faster look-ups */
#define CONFIG_YAFFS_SHORT_NAMES_IN_RAM

/* Default: Unselected */
/* Meaning: Select to disable block refreshing. */
/* Block Refreshing periodically rewrites the oldest block. */
/* #define CONFIG_DISABLE_BLOCK_REFRESHING */

/* Default: Unselected */
/* Meaning: Select to disable background processing */
/* #define CONFIG_DISABLE_BACKGROUND */


/* Default: Selected */
/* Meaning: Enable XATTR support */
#define CONFIG_YAFFS_XATTR

/* Default: Not selected */
/* Meaning: Use older-style on-NAND data format with pageStatus byte */
/* #define CONFIG_YAFFS_9BYTE_TAGS */

#endif /* YAFFS_OUT_OF_TREE */

#endif /* __YAFFS_CONFIG_H__ */
