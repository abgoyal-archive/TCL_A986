/* nbi.h - definitions for Net Boot Image */

#ifndef GRUB_NBI_HEADER
#define GRUB_NBI_HEADER

#define NBI_MAGIC		0x1B031336
#define NBI_DEST_ADDR		0x10000
#define NBI_DEST_SEG		0x1000
#define NBI_DEST_OFF		0x0000
#define RELOCATED_ADDR		0x8000
#define RELOCATED_SEG		0x0800
#define RELOCATED_OFF		0x0000
#define STAGE2_START_ADDR	0x8200

#endif /* ! GRUB_NBI_HEADER */
