

#ifndef ISCSI_IBFT_H
#define ISCSI_IBFT_H

struct ibft_table_header {
	char signature[4];
	u32 length;
	u8 revision;
	u8 checksum;
	char oem_id[6];
	char oem_table_id[8];
	char reserved[24];
} __attribute__((__packed__));

extern struct ibft_table_header *ibft_addr;

#ifdef CONFIG_ISCSI_IBFT_FIND
unsigned long find_ibft_region(unsigned long *sizep);
#else
static inline unsigned long find_ibft_region(unsigned long *sizep)
{
	*sizep = 0;
	return 0;
}
#endif

#endif /* ISCSI_IBFT_H */
