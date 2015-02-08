

#ifndef __XEN_PUBLIC_IO_BLKIF_H__
#define __XEN_PUBLIC_IO_BLKIF_H__

#include "ring.h"
#include "../grant_table.h"


typedef uint16_t blkif_vdev_t;
typedef uint64_t blkif_sector_t;

#define BLKIF_OP_READ              0
#define BLKIF_OP_WRITE             1
#define BLKIF_OP_WRITE_BARRIER     2

#define BLKIF_MAX_SEGMENTS_PER_REQUEST 11

struct blkif_request {
	uint8_t        operation;    /* BLKIF_OP_???                         */
	uint8_t        nr_segments;  /* number of segments                   */
	blkif_vdev_t   handle;       /* only for read/write requests         */
	uint64_t       id;           /* private guest value, echoed in resp  */
	blkif_sector_t sector_number;/* start sector idx on disk (r/w only)  */
	struct blkif_request_segment {
		grant_ref_t gref;        /* reference to I/O buffer frame        */
		/* @first_sect: first sector in frame to transfer (inclusive).   */
		/* @last_sect: last sector in frame to transfer (inclusive).     */
		uint8_t     first_sect, last_sect;
	} seg[BLKIF_MAX_SEGMENTS_PER_REQUEST];
};

struct blkif_response {
	uint64_t        id;              /* copied from request */
	uint8_t         operation;       /* copied from request */
	int16_t         status;          /* BLKIF_RSP_???       */
};

 /* Operation not supported (only happens on barrier writes). */
#define BLKIF_RSP_EOPNOTSUPP  -2
 /* Operation failed for some unspecified reason (-EIO). */
#define BLKIF_RSP_ERROR       -1
 /* Operation completed successfully. */
#define BLKIF_RSP_OKAY         0


DEFINE_RING_TYPES(blkif, struct blkif_request, struct blkif_response);

#define VDISK_CDROM        0x1
#define VDISK_REMOVABLE    0x2
#define VDISK_READONLY     0x4

#endif /* __XEN_PUBLIC_IO_BLKIF_H__ */
