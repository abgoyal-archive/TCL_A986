
#ifndef STAGE1_HEADER
#define STAGE1_HEADER	1


/* Define the version numbers here, so that Stage 1 can know them.  */
#define COMPAT_VERSION_MAJOR	3
#define COMPAT_VERSION_MINOR	2
#define COMPAT_VERSION		((COMPAT_VERSION_MINOR << 8) \
					| COMPAT_VERSION_MAJOR)

/* The signature for bootloader.  */
#define STAGE1_SIGNATURE	0xaa55

/* The offset of the end of BPB (BIOS Parameter Block).  */
#define STAGE1_BPBEND		0x3e

/* The offset of the major version.  */
#define STAGE1_VER_MAJ_OFFS	0x3e

/* The offset of BOOT_DRIVE.  */
#define STAGE1_BOOT_DRIVE	0x40

/* The offset of FORCE_LBA.  */
#define STAGE1_FORCE_LBA	0x41

/* The offset of STAGE2_ADDRESS.  */
#define STAGE1_STAGE2_ADDRESS	0x42

/* The offset of STAGE2_SECTOR.  */
#define STAGE1_STAGE2_SECTOR	0x44

/* The offset of STAGE2_SEGMENT.  */
#define STAGE1_STAGE2_SEGMENT	0x48

/* The offset of BOOT_DRIVE_CHECK.  */
#define STAGE1_BOOT_DRIVE_CHECK	0x4b

/* The offset of a magic number used by Windows NT.  */
#define STAGE1_WINDOWS_NT_MAGIC	0x1b8

/* The offset of the start of the partition table.  */
#define STAGE1_PARTSTART	0x1be

/* The offset of the end of the partition table.  */
#define STAGE1_PARTEND		0x1fe

/* The stack segment.  */
#define STAGE1_STACKSEG		0x2000

#define STAGE1_BUFFERSEG	0x7000

/* The address of drive parameters.  */
#define STAGE1_DRP_ADDR		0x7f00

/* The size of drive parameters.  */
#define STAGE1_DRP_SIZE		0x42

#define STAGE1_BIOS_HD_FLAG	0x80

/* The drive number of an invalid drive.  */
#define GRUB_INVALID_DRIVE	0xFF

#endif /* ! STAGE1_HEADER */
