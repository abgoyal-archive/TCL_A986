#ifndef __diskio_h
#define __diskio_h
#ifdef __TURBOC__
#ifndef __LARGE__
# error "ext2fs/DOS library requires LARGE model!"
#endif
#endif

#ifdef __TURBOC__
#include "msdos.h"
#endif

typedef struct
{
  unsigned short       cyl;     /* Cylinder (or track) */
  unsigned short       head;
  unsigned short       sector;
  unsigned short       offset;  /* Offset of byte within the sector */
} CHS;

typedef struct
{
  char                 *dev;  /* _Linux_ device name (like "/dev/hda1") */
  unsigned char        phys;  /* Physical DOS drive number */
  unsigned long        start; /* LBA address of partition start */
  unsigned long        len;   /* length of partition in sectors */
  unsigned char        pno;   /* Partition number (read from *dev) */

  /* This partition's drive geometry */
  unsigned short       cyls;
  unsigned short       heads;
  unsigned short       sects;
} PARTITION;

#ifdef __DJGPP__
#pragma pack(1)
#endif
typedef struct
{
  unsigned char        active;
  unsigned char        start_head;
  unsigned char        start_sec;
  unsigned char        start_cyl;
  unsigned char        type;
  unsigned char        end_head;
  unsigned char        end_sec;
  unsigned char        end_cyl;
  unsigned long        first_sec_rel;
  unsigned long        size;
} PTABLE_ENTRY;
#ifdef __DJGPP__
#pragma pack()
#endif

#define DISK_READ          0x02
#define DISK_WRITE         0x03
#define DISK_GET_GEOMETRY  0x08
#define DISK_READY         0x10

#define ERR_BADDEV         0x00000001L
#define ERR_HARDWARE       0x00000002L
#define ERR_NOTSUPP        0x00000003L
#define ERR_NOTEXT2FS      0x00000004L
#define ERR_EMPTYPART      0x00000005L
#define ERR_LINUXSWAP      0x00000006L


extern unsigned long        _dio_error;

extern unsigned long        _dio_hw_error;

#define HW_OK()             ((unsigned char)_dio_hw_error == 0x00)
#define HW_BAD_CMD()        ((unsigned char)_dio_hw_error == 0x01)
#define HW_NO_ADDR_MARK()   ((unsigned char)_dio_hw_error == 0x02)
#define HW_WRITE_PROT()     ((unsigned char)_dio_hw_error == 0x03)
#define HW_NO_SECTOR()      ((unsigned char)_dio_hw_error == 0x04)
#define HW_RESET_FAIL()     ((unsigned char)_dio_hw_error == 0x05)
#define HW_DISK_CHANGED()   ((unsigned char)_dio_hw_error == 0x06)
#define HW_DRIVE_FAIL()     ((unsigned char)_dio_hw_error == 0x07)
#define HW_DMA_OVERRUN()    ((unsigned char)_dio_hw_error == 0x08)
#define HW_DMA_BOUNDARY()   ((unsigned char)_dio_hw_error == 0x09)
#define HW_BAD_SECTOR()     ((unsigned char)_dio_hw_error == 0x0A)
#define HW_BAD_TRACK()      ((unsigned char)_dio_hw_error == 0x0B)
#define HW_UNSUPP_TRACK()   ((unsigned char)_dio_hw_error == 0x0C)
#define HW_BAD_CRC_ECC()    ((unsigned char)_dio_hw_error == 0x10)
#define HW_CRC_ECC_CORR()   ((unsigned char)_dio_hw_error == 0x11)
#define HW_CONTR_FAIL()     ((unsigned char)_dio_hw_error == 0x20)
#define HW_SEEK_FAIL()      ((unsigned char)_dio_hw_error == 0x40)
#define HW_ATTACH_FAIL()    ((unsigned char)_dio_hw_error == 0x80)
#define HW_DRIVE_NREADY()   ((unsigned char)_dio_hw_error == 0xAA)
#define HW_UNDEF_ERROR()    ((unsigned char)_dio_hw_error == 0xBB)
#define HW_WRITE_FAULT()    ((unsigned char)_dio_hw_error == 0xCC)
#define HW_STATUS_ERROR()   ((unsigned char)_dio_hw_error == 0xE0)
#define HW_SENSE_FAIL()     ((unsigned char)_dio_hw_error == 0xFF)


int open_partition(char *dev);

#endif /* __diskio_h */
