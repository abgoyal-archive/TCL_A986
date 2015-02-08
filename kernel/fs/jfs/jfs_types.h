
#ifndef _H_JFS_TYPES
#define	_H_JFS_TYPES


#include <linux/types.h>
#include <linux/nls.h>

#include "endian24.h"

typedef u16 tid_t;
typedef u16 lid_t;

struct timestruc_t {
	__le32 tv_sec;
	__le32 tv_nsec;
};


#define LEFTMOSTONE	0x80000000
#define	HIGHORDER	0x80000000u	/* high order bit on	*/
#define	ONES		0xffffffffu	/* all bit on		*/

typedef struct {
	unsigned len:24;
	unsigned addr1:8;
	__le32 addr2;
} pxd_t;

/* xd_t field construction */

#define	PXDlength(pxd, length32)	((pxd)->len = __cpu_to_le24(length32))
#define	PXDaddress(pxd, address64)\
{\
	(pxd)->addr1 = ((s64)address64) >> 32;\
	(pxd)->addr2 = __cpu_to_le32((address64) & 0xffffffff);\
}

/* xd_t field extraction */
#define	lengthPXD(pxd)	__le24_to_cpu((pxd)->len)
#define	addressPXD(pxd)\
	( ((s64)((pxd)->addr1)) << 32 | __le32_to_cpu((pxd)->addr2))

#define MAXTREEHEIGHT 8
/* pxd list */
struct pxdlist {
	s16 maxnpxd;
	s16 npxd;
	pxd_t pxd[MAXTREEHEIGHT];
};


typedef struct {
	unsigned flag:8;	/* 1: flags */
	unsigned rsrvd:24;
	__le32 size;		/* 4: size in byte */
	unsigned len:24;	/* 3: length in unit of fsblksize */
	unsigned addr1:8;	/* 1: address in unit of fsblksize */
	__le32 addr2;		/* 4: address in unit of fsblksize */
} dxd_t;			/* - 16 - */

/* dxd_t flags */
#define	DXD_INDEX	0x80	/* B+-tree index */
#define	DXD_INLINE	0x40	/* in-line data extent */
#define	DXD_EXTENT	0x20	/* out-of-line single extent */
#define	DXD_FILE	0x10	/* out-of-line file (inode) */
#define DXD_CORRUPT	0x08	/* Inconsistency detected */

#define	DXDlength	PXDlength
#define	DXDaddress	PXDaddress
#define	lengthDXD	lengthPXD
#define	addressDXD	addressPXD
#define DXDsize(dxd, size32) ((dxd)->size = cpu_to_le32(size32))
#define sizeDXD(dxd)	le32_to_cpu((dxd)->size)

struct component_name {
	int namlen;
	wchar_t *name;
};


struct dasd {
	u8 thresh;		/* Alert Threshold (in percent)		*/
	u8 delta;		/* Alert Threshold delta (in percent)	*/
	u8 rsrvd1;
	u8 limit_hi;		/* DASD limit (in logical blocks)	*/
	__le32 limit_lo;	/* DASD limit (in logical blocks)	*/
	u8 rsrvd2[3];
	u8 used_hi;		/* DASD usage (in logical blocks)	*/
	__le32 used_lo;		/* DASD usage (in logical blocks)	*/
};

#define DASDLIMIT(dasdp) \
	(((u64)((dasdp)->limit_hi) << 32) + __le32_to_cpu((dasdp)->limit_lo))
#define setDASDLIMIT(dasdp, limit)\
{\
	(dasdp)->limit_hi = ((u64)limit) >> 32;\
	(dasdp)->limit_lo = __cpu_to_le32(limit);\
}
#define DASDUSED(dasdp) \
	(((u64)((dasdp)->used_hi) << 32) + __le32_to_cpu((dasdp)->used_lo))
#define setDASDUSED(dasdp, used)\
{\
	(dasdp)->used_hi = ((u64)used) >> 32;\
	(dasdp)->used_lo = __cpu_to_le32(used);\
}

#endif				/* !_H_JFS_TYPES */
