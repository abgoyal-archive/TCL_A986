


#ifndef __ASM_BOOTX_H__
#define __ASM_BOOTX_H__

#include <linux/types.h>

#ifdef macintosh
#include <Types.h>
#include "linux_type_defs.h"
#endif

#ifdef macintosh
/* All this requires PowerPC alignment */
#pragma options align=power
#endif


#define BOOT_INFO_VERSION               5
#define BOOT_INFO_COMPATIBLE_VERSION    1

#define BOOT_ARCH_PCI                   0x00000001UL
#define BOOT_ARCH_NUBUS                 0x00000002UL
#define BOOT_ARCH_NUBUS_PDM             0x00000010UL
#define BOOT_ARCH_NUBUS_PERFORMA        0x00000020UL
#define BOOT_ARCH_NUBUS_POWERBOOK       0x00000040UL

/*  Maximum number of ranges in phys memory map */
#define MAX_MEM_MAP_SIZE				26

typedef struct boot_info_map_entry
{
    __u32       physAddr;                /* Physical starting address */
    __u32       size;                    /* Size in bytes */
} boot_info_map_entry_t;


typedef struct boot_infos
{
    /* Version of this structure */
    __u32       version;
    /* backward compatible down to version: */
    __u32       compatible_version;

    /* NEW (vers. 2) this holds the current _logical_ base addr of
       the frame buffer (for use by early boot message) */
    __u8*       logicalDisplayBase;

    /* NEW (vers. 4) Apple's machine identification */
    __u32       machineID;

    /* NEW (vers. 4) Detected hw architecture */
    __u32       architecture;

    /* The device tree (internal addresses relative to the beginning of the tree,
     * device tree offset relative to the beginning of this structure).
     * On pre-PCI macintosh (BOOT_ARCH_PCI bit set to 0 in architecture), this
     * field is 0.
     */
    __u32       deviceTreeOffset;        /* Device tree offset */
    __u32       deviceTreeSize;          /* Size of the device tree */

    /* Some infos about the current MacOS display */
    __u32       dispDeviceRect[4];       /* left,top,right,bottom */
    __u32       dispDeviceDepth;         /* (8, 16 or 32) */
    __u8*       dispDeviceBase;          /* base address (physical) */
    __u32       dispDeviceRowBytes;      /* rowbytes (in bytes) */
    __u32       dispDeviceColorsOffset;  /* Colormap (8 bits only) or 0 (*) */
    /* Optional offset in the registry to the current
     * MacOS display. (Can be 0 when not detected) */
     __u32      dispDeviceRegEntryOffset;

    /* Optional pointer to boot ramdisk (offset from this structure) */
    __u32       ramDisk;
    __u32       ramDiskSize;             /* size of ramdisk image */

    /* Kernel command line arguments (offset from this structure) */
    __u32       kernelParamsOffset;

    /* ALL BELOW NEW (vers. 4) */

    /* This defines the physical memory. Valid with BOOT_ARCH_NUBUS flag
       (non-PCI) only. On PCI, memory is contiguous and it's size is in the
       device-tree. */
    boot_info_map_entry_t
    	        physMemoryMap[MAX_MEM_MAP_SIZE]; /* Where the phys memory is */
    __u32       physMemoryMapSize;               /* How many entries in map */


    /* The framebuffer size (optional, currently 0) */
    __u32       frameBufferSize;         /* Represents a max size, can be 0. */

    /* NEW (vers. 5) */

    /* Total params size (args + colormap + device tree + ramdisk) */
    __u32       totalParamsSize;

} boot_infos_t;

#ifdef __KERNEL__
#define BOOTX_COLORTABLE_SIZE    (256UL*3UL*2UL)

struct bootx_dt_prop {
	u32	name;
	int	length;
	u32	value;
	u32	next;
};

struct bootx_dt_node {
	u32	unused0;
	u32	unused1;
	u32	phandle;	/* not really available */
	u32	unused2;
	u32	unused3;
	u32	unused4;
	u32	unused5;
	u32	full_name;
	u32	properties;
	u32	parent;
	u32	child;
	u32	sibling;
	u32	next;
	u32	allnext;
};

extern void bootx_init(unsigned long r4, unsigned long phys);

#endif /* __KERNEL__ */

#ifdef macintosh
#pragma options align=reset
#endif

#endif
