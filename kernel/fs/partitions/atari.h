

struct partition_info
{
  u8 flg;			/* bit 0: active; bit 7: bootable */
  char id[3];			/* "GEM", "BGM", "XGM", or other */
  __be32 st;			/* start of partition */
  __be32 siz;			/* length of partition */
};

struct rootsector
{
  char unused[0x156];		/* room for boot code */
  struct partition_info icdpart[8];	/* info for ICD-partitions 5..12 */
  char unused2[0xc];
  u32 hd_siz;			/* size of disk in blocks */
  struct partition_info part[4];
  u32 bsl_st;			/* start of bad sector list */
  u32 bsl_cnt;			/* length of bad sector list */
  u16 checksum;			/* checksum for bootable disks */
} __attribute__((__packed__));

int atari_partition(struct parsed_partitions *state);
