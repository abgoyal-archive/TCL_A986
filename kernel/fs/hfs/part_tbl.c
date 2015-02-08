

#include "hfs_fs.h"

struct new_pmap {
	__be16	pmSig;		/* signature */
	__be16	reSigPad;	/* padding */
	__be32	pmMapBlkCnt;	/* partition blocks count */
	__be32	pmPyPartStart;	/* physical block start of partition */
	__be32	pmPartBlkCnt;	/* physical block count of partition */
	u8	pmPartName[32];	/* (null terminated?) string
				   giving the name of this
				   partition */
	u8	pmPartType[32];	/* (null terminated?) string
				   giving the type of this
				   partition */
	/* a bunch more stuff we don't need */
} __packed;

struct old_pmap {
	__be16		pdSig;	/* Signature bytes */
	struct 	old_pmap_entry {
		__be32	pdStart;
		__be32	pdSize;
		__be32	pdFSID;
	}	pdEntry[42];
} __packed;

int hfs_part_find(struct super_block *sb,
		  sector_t *part_start, sector_t *part_size)
{
	struct buffer_head *bh;
	__be16 *data;
	int i, size, res;

	res = -ENOENT;
	bh = sb_bread512(sb, *part_start + HFS_PMAP_BLK, data);
	if (!bh)
		return -EIO;

	switch (be16_to_cpu(*data)) {
	case HFS_OLD_PMAP_MAGIC:
	  {
		struct old_pmap *pm;
		struct old_pmap_entry *p;

		pm = (struct old_pmap *)bh->b_data;
		p = pm->pdEntry;
		size = 42;
		for (i = 0; i < size; p++, i++) {
			if (p->pdStart && p->pdSize &&
			    p->pdFSID == cpu_to_be32(0x54465331)/*"TFS1"*/ &&
			    (HFS_SB(sb)->part < 0 || HFS_SB(sb)->part == i)) {
				*part_start += be32_to_cpu(p->pdStart);
				*part_size = be32_to_cpu(p->pdSize);
				res = 0;
			}
		}
		break;
	  }
	case HFS_NEW_PMAP_MAGIC:
	  {
		struct new_pmap *pm;

		pm = (struct new_pmap *)bh->b_data;
		size = be32_to_cpu(pm->pmMapBlkCnt);
		for (i = 0; i < size;) {
			if (!memcmp(pm->pmPartType,"Apple_HFS", 9) &&
			    (HFS_SB(sb)->part < 0 || HFS_SB(sb)->part == i)) {
				*part_start += be32_to_cpu(pm->pmPyPartStart);
				*part_size = be32_to_cpu(pm->pmPartBlkCnt);
				res = 0;
				break;
			}
			brelse(bh);
			bh = sb_bread512(sb, *part_start + HFS_PMAP_BLK + ++i, pm);
			if (!bh)
				return -EIO;
			if (pm->pmSig != cpu_to_be16(HFS_NEW_PMAP_MAGIC))
				break;
		}
		break;
	  }
	}
	brelse(bh);

	return res;
}
