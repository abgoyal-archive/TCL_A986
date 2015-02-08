

#ifndef _LINUX_NTFS_USNJRNL_H
#define _LINUX_NTFS_USNJRNL_H

#ifdef NTFS_RW

#include "types.h"
#include "endian.h"
#include "layout.h"
#include "volume.h"


/* Some $UsnJrnl related constants. */
#define UsnJrnlMajorVer		2
#define UsnJrnlMinorVer		0

typedef struct {
/*Ofs*/
/*   0*/sle64 maximum_size;	/* The maximum on-disk size of the $DATA/$J
				   attribute. */
/*   8*/sle64 allocation_delta;	/* Number of bytes by which to increase the
				   size of the $DATA/$J attribute. */
/*0x10*/sle64 journal_id;	/* Current id of the transaction log. */
/*0x18*/leUSN lowest_valid_usn;	/* Lowest valid usn in $DATA/$J for the
				   current journal_id. */
/* sizeof() = 32 (0x20) bytes */
} __attribute__ ((__packed__)) USN_HEADER;

enum {
	USN_REASON_DATA_OVERWRITE	= cpu_to_le32(0x00000001),
	USN_REASON_DATA_EXTEND		= cpu_to_le32(0x00000002),
	USN_REASON_DATA_TRUNCATION	= cpu_to_le32(0x00000004),
	USN_REASON_NAMED_DATA_OVERWRITE	= cpu_to_le32(0x00000010),
	USN_REASON_NAMED_DATA_EXTEND	= cpu_to_le32(0x00000020),
	USN_REASON_NAMED_DATA_TRUNCATION= cpu_to_le32(0x00000040),
	USN_REASON_FILE_CREATE		= cpu_to_le32(0x00000100),
	USN_REASON_FILE_DELETE		= cpu_to_le32(0x00000200),
	USN_REASON_EA_CHANGE		= cpu_to_le32(0x00000400),
	USN_REASON_SECURITY_CHANGE	= cpu_to_le32(0x00000800),
	USN_REASON_RENAME_OLD_NAME	= cpu_to_le32(0x00001000),
	USN_REASON_RENAME_NEW_NAME	= cpu_to_le32(0x00002000),
	USN_REASON_INDEXABLE_CHANGE	= cpu_to_le32(0x00004000),
	USN_REASON_BASIC_INFO_CHANGE	= cpu_to_le32(0x00008000),
	USN_REASON_HARD_LINK_CHANGE	= cpu_to_le32(0x00010000),
	USN_REASON_COMPRESSION_CHANGE	= cpu_to_le32(0x00020000),
	USN_REASON_ENCRYPTION_CHANGE	= cpu_to_le32(0x00040000),
	USN_REASON_OBJECT_ID_CHANGE	= cpu_to_le32(0x00080000),
	USN_REASON_REPARSE_POINT_CHANGE	= cpu_to_le32(0x00100000),
	USN_REASON_STREAM_CHANGE	= cpu_to_le32(0x00200000),
	USN_REASON_CLOSE		= cpu_to_le32(0x80000000),
};

typedef le32 USN_REASON_FLAGS;

enum {
	USN_SOURCE_DATA_MANAGEMENT	  = cpu_to_le32(0x00000001),
	USN_SOURCE_AUXILIARY_DATA	  = cpu_to_le32(0x00000002),
	USN_SOURCE_REPLICATION_MANAGEMENT = cpu_to_le32(0x00000004),
};

typedef le32 USN_SOURCE_INFO_FLAGS;

typedef struct {
/*Ofs*/
/*   0*/le32 length;		/* Byte size of this record (8-byte
				   aligned). */
/*   4*/le16 major_ver;		/* Major version of the transaction log used
				   for this record. */
/*   6*/le16 minor_ver;		/* Minor version of the transaction log used
				   for this record. */
/*   8*/leMFT_REF mft_reference;/* The mft reference of the file (or
				   directory) described by this record. */
/*0x10*/leMFT_REF parent_directory;/* The mft reference of the parent
				   directory of the file described by this
				   record. */
/*0x18*/leUSN usn;		/* The usn of this record.  Equals the offset
				   within the $DATA/$J attribute. */
/*0x20*/sle64 time;		/* Time when this record was created. */
/*0x28*/USN_REASON_FLAGS reason;/* Reason flags (see above). */
/*0x2c*/USN_SOURCE_INFO_FLAGS source_info;/* Source info flags (see above). */
/*0x30*/le32 security_id;	/* File security_id copied from
				   $STANDARD_INFORMATION. */
/*0x34*/FILE_ATTR_FLAGS file_attributes;	/* File attributes copied from
				   $STANDARD_INFORMATION or $FILE_NAME (not
				   sure which). */
/*0x38*/le16 file_name_size;	/* Size of the file name in bytes. */
/*0x3a*/le16 file_name_offset;	/* Offset to the file name in bytes from the
				   start of this record. */
/*0x3c*/ntfschar file_name[0];	/* Use when creating only.  When reading use
				   file_name_offset to determine the location
				   of the name. */
/* sizeof() = 60 (0x3c) bytes */
} __attribute__ ((__packed__)) USN_RECORD;

extern bool ntfs_stamp_usnjrnl(ntfs_volume *vol);

#endif /* NTFS_RW */

#endif /* _LINUX_NTFS_USNJRNL_H */
