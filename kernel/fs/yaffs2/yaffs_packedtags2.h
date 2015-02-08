

/* This is used to pack YAFFS2 tags, not YAFFS1tags. */

#ifndef __YAFFS_PACKEDTAGS2_H__
#define __YAFFS_PACKEDTAGS2_H__

#include "yaffs_guts.h"
#include "yaffs_ecc.h"

typedef struct {
	unsigned sequenceNumber;
	unsigned objectId;
	unsigned chunkId;
	unsigned byteCount;
} yaffs_PackedTags2TagsPart;

typedef struct {
	yaffs_PackedTags2TagsPart t;
	yaffs_ECCOther ecc;
} yaffs_PackedTags2;

/* Full packed tags with ECC, used for oob tags */
void yaffs_PackTags2(yaffs_PackedTags2 *pt, const yaffs_ExtendedTags *t, int tagsECC);
void yaffs_UnpackTags2(yaffs_ExtendedTags *t, yaffs_PackedTags2 *pt, int tagsECC);

/* Only the tags part (no ECC for use with inband tags */
void yaffs_PackTags2TagsPart(yaffs_PackedTags2TagsPart *pt, const yaffs_ExtendedTags *t);
void yaffs_UnpackTags2TagsPart(yaffs_ExtendedTags *t, yaffs_PackedTags2TagsPart *pt);
#endif
