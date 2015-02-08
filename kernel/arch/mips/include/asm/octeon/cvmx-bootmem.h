


#ifndef __CVMX_BOOTMEM_H__
#define __CVMX_BOOTMEM_H__
/* Must be multiple of 8, changing breaks ABI */
#define CVMX_BOOTMEM_NAME_LEN 128

/* Can change without breaking ABI */
#define CVMX_BOOTMEM_NUM_NAMED_BLOCKS 64

/* minimum alignment of bootmem alloced blocks */
#define CVMX_BOOTMEM_ALIGNMENT_SIZE     (16ull)

/* Flags for cvmx_bootmem_phy_mem* functions */
/* Allocate from end of block instead of beginning */
#define CVMX_BOOTMEM_FLAG_END_ALLOC    (1 << 0)

/* Don't do any locking. */
#define CVMX_BOOTMEM_FLAG_NO_LOCKING   (1 << 1)

struct cvmx_bootmem_block_header {
	/*
	 * Note: these are referenced from assembly routines in the
	 * bootloader, so this structure should not be changed
	 * without changing those routines as well.
	 */
	uint64_t next_block_addr;
	uint64_t size;

};

struct cvmx_bootmem_named_block_desc {
	/* Base address of named block */
	uint64_t base_addr;
	/*
	 * Size actually allocated for named block (may differ from
	 * requested).
	 */
	uint64_t size;
	/* name of named block */
	char name[CVMX_BOOTMEM_NAME_LEN];
};

/* Current descriptor versions */
/* CVMX bootmem descriptor major version */
#define CVMX_BOOTMEM_DESC_MAJ_VER   3

/* CVMX bootmem descriptor minor version */
#define CVMX_BOOTMEM_DESC_MIN_VER   0

struct cvmx_bootmem_desc {
	/* spinlock to control access to list */
	uint32_t lock;
	/* flags for indicating various conditions */
	uint32_t flags;
	uint64_t head_addr;

	/* Incremented when incompatible changes made */
	uint32_t major_version;

	/*
	 * Incremented changed when compatible changes made, reset to
	 * zero when major incremented.
	 */
	uint32_t minor_version;

	uint64_t app_data_addr;
	uint64_t app_data_size;

	/* number of elements in named blocks array */
	uint32_t named_block_num_blocks;

	/* length of name array in bootmem blocks */
	uint32_t named_block_name_len;
	/* address of named memory block descriptors */
	uint64_t named_block_array_addr;

};

extern int cvmx_bootmem_init(void *mem_desc_ptr);

extern void *cvmx_bootmem_alloc(uint64_t size, uint64_t alignment);

extern void *cvmx_bootmem_alloc_address(uint64_t size, uint64_t address,
					uint64_t alignment);

extern void *cvmx_bootmem_alloc_range(uint64_t size, uint64_t alignment,
				      uint64_t min_addr, uint64_t max_addr);



extern void *cvmx_bootmem_alloc_named(uint64_t size, uint64_t alignment,
				      char *name);



extern void *cvmx_bootmem_alloc_named_address(uint64_t size, uint64_t address,
					      char *name);



extern void *cvmx_bootmem_alloc_named_range(uint64_t size, uint64_t min_addr,
					    uint64_t max_addr, uint64_t align,
					    char *name);

extern int cvmx_bootmem_free_named(char *name);

struct cvmx_bootmem_named_block_desc *cvmx_bootmem_find_named_block(char *name);

int64_t cvmx_bootmem_phy_alloc(uint64_t req_size, uint64_t address_min,
			       uint64_t address_max, uint64_t alignment,
			       uint32_t flags);

int64_t cvmx_bootmem_phy_named_block_alloc(uint64_t size, uint64_t min_addr,
					   uint64_t max_addr,
					   uint64_t alignment,
					   char *name, uint32_t flags);

struct cvmx_bootmem_named_block_desc *
cvmx_bootmem_phy_named_block_find(char *name, uint32_t flags);

int cvmx_bootmem_phy_named_block_free(char *name, uint32_t flags);

int __cvmx_bootmem_phy_free(uint64_t phy_addr, uint64_t size, uint32_t flags);

void cvmx_bootmem_lock(void);

void cvmx_bootmem_unlock(void);

#endif /*   __CVMX_BOOTMEM_H__ */
