/* hercules.h - hercules console interface */

#ifndef GRUB_HERCULES_HEADER
#define GRUB_HERCULES_HEADER	1

/* Macros.  */
#define HERCULES_VIDEO_ADDR	RAW_ADDR (0xB0000)
#define HERCULES_WIDTH		80
#define HERCULES_HEIGHT		25
#define HERCULES_INDEX_REG	0x3b4
#define HERCULES_DATA_REG	0x3b5

#endif /* ! GRUB_HERCULES_HEADER */
