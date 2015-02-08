
#ifndef MTD_PARTITIONS_H
#define MTD_PARTITIONS_H

#include <linux/types.h>



struct mtd_partition {
	char *name;			/* identifier string */
	u_int32_t size;			/* partition size */
	u_int32_t offset;		/* offset within the master MTD space */
	u_int32_t mask_flags;		/* master MTD flags to mask out for this partition */
	struct nand_ecclayout *ecclayout;	/* out of band layout for this partition (NAND only)*/
	struct mtd_info **mtdp;		/* pointer to store the MTD object */
};

#define MTDPART_OFS_NXTBLK	(-2)
#define MTDPART_OFS_APPEND	(-1)
#define MTDPART_SIZ_FULL	(0)


int add_mtd_partitions(struct mtd_info *, const struct mtd_partition *, int);
int del_mtd_partitions(struct mtd_info *);


struct mtd_part_parser {
	struct list_head list;
	struct module *owner;
	const char *name;
	int (*parse_fn)(struct mtd_info *, struct mtd_partition **, unsigned long);
};

extern int register_mtd_parser(struct mtd_part_parser *parser);
extern int deregister_mtd_parser(struct mtd_part_parser *parser);
extern int parse_mtd_partitions(struct mtd_info *master, const char **types,
				struct mtd_partition **pparts, unsigned long origin);

#define put_partition_parser(p) do { module_put((p)->owner); } while(0)

#endif

