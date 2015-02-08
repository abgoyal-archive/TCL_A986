
#ifndef BITMAP_H
#define BITMAP_H 1

#define BITMAP_MAJOR_LO 3
#define BITMAP_MAJOR_HI 4
#define	BITMAP_MAJOR_HOSTENDIAN 3

#define BITMAP_MINOR 39


#ifdef __KERNEL__

#define PAGE_BITS (PAGE_SIZE << 3)
#define PAGE_BIT_SHIFT (PAGE_SHIFT + 3)

typedef __u16 bitmap_counter_t;
#define COUNTER_BITS 16
#define COUNTER_BIT_SHIFT 4
#define COUNTER_BYTE_RATIO (COUNTER_BITS / 8)
#define COUNTER_BYTE_SHIFT (COUNTER_BIT_SHIFT - 3)

#define NEEDED_MASK ((bitmap_counter_t) (1 << (COUNTER_BITS - 1)))
#define RESYNC_MASK ((bitmap_counter_t) (1 << (COUNTER_BITS - 2)))
#define COUNTER_MAX ((bitmap_counter_t) RESYNC_MASK - 1)
#define NEEDED(x) (((bitmap_counter_t) x) & NEEDED_MASK)
#define RESYNC(x) (((bitmap_counter_t) x) & RESYNC_MASK)
#define COUNTER(x) (((bitmap_counter_t) x) & COUNTER_MAX)

/* how many counters per page? */
#define PAGE_COUNTER_RATIO (PAGE_BITS / COUNTER_BITS)
/* same, except a shift value for more efficient bitops */
#define PAGE_COUNTER_SHIFT (PAGE_BIT_SHIFT - COUNTER_BIT_SHIFT)
/* same, except a mask value for more efficient bitops */
#define PAGE_COUNTER_MASK  (PAGE_COUNTER_RATIO - 1)

#define BITMAP_BLOCK_SIZE 512
#define BITMAP_BLOCK_SHIFT 9

/* how many blocks per chunk? (this is variable) */
#define CHUNK_BLOCK_RATIO(bitmap) ((bitmap)->mddev->bitmap_info.chunksize >> BITMAP_BLOCK_SHIFT)
#define CHUNK_BLOCK_SHIFT(bitmap) ((bitmap)->chunkshift - BITMAP_BLOCK_SHIFT)
#define CHUNK_BLOCK_MASK(bitmap) (CHUNK_BLOCK_RATIO(bitmap) - 1)

/* when hijacked, the counters and bits represent even larger "chunks" */
/* there will be 1024 chunks represented by each counter in the page pointers */
#define PAGEPTR_BLOCK_RATIO(bitmap) \
			(CHUNK_BLOCK_RATIO(bitmap) << PAGE_COUNTER_SHIFT >> 1)
#define PAGEPTR_BLOCK_SHIFT(bitmap) \
			(CHUNK_BLOCK_SHIFT(bitmap) + PAGE_COUNTER_SHIFT - 1)
#define PAGEPTR_BLOCK_MASK(bitmap) (PAGEPTR_BLOCK_RATIO(bitmap) - 1)

#endif


#define BITMAP_MAGIC 0x6d746962

/* use these for bitmap->flags and bitmap->sb->state bit-fields */
enum bitmap_state {
	BITMAP_STALE  = 0x002,  /* the bitmap file is out of date or had -EIO */
	BITMAP_WRITE_ERROR = 0x004, /* A write error has occurred */
	BITMAP_HOSTENDIAN = 0x8000,
};

/* the superblock at the front of the bitmap file -- little endian */
typedef struct bitmap_super_s {
	__le32 magic;        /*  0  BITMAP_MAGIC */
	__le32 version;      /*  4  the bitmap major for now, could change... */
	__u8  uuid[16];      /*  8  128 bit uuid - must match md device uuid */
	__le64 events;       /* 24  event counter for the bitmap (1)*/
	__le64 events_cleared;/*32  event counter when last bit cleared (2) */
	__le64 sync_size;    /* 40  the size of the md device's sync range(3) */
	__le32 state;        /* 48  bitmap state information */
	__le32 chunksize;    /* 52  the bitmap chunk size in bytes */
	__le32 daemon_sleep; /* 56  seconds between disk flushes */
	__le32 write_behind; /* 60  number of outstanding write-behind writes */

	__u8  pad[256 - 64]; /* set to zero */
} bitmap_super_t;


#ifdef __KERNEL__

/* the in-memory bitmap is represented by bitmap_pages */
struct bitmap_page {
	/*
	 * map points to the actual memory page
	 */
	char *map;
	/*
	 * in emergencies (when map cannot be alloced), hijack the map
	 * pointer and use it as two counters itself
	 */
	unsigned int hijacked:1;
	/*
	 * count of dirty bits on the page
	 */
	unsigned int  count:31;
};

/* keep track of bitmap file pages that have pending writes on them */
struct page_list {
	struct list_head list;
	struct page *page;
};

/* the main bitmap structure - one per mddev */
struct bitmap {
	struct bitmap_page *bp;
	unsigned long pages; /* total number of pages in the bitmap */
	unsigned long missing_pages; /* number of pages not yet allocated */

	mddev_t *mddev; /* the md device that the bitmap is for */

	int counter_bits; /* how many bits per block counter */

	/* bitmap chunksize -- how much data does each bit represent? */
	unsigned long chunkshift; /* chunksize = 2^chunkshift (for bitops) */
	unsigned long chunks; /* total number of data chunks for the array */

	/* We hold a count on the chunk currently being synced, and drop
	 * it when the last block is started.  If the resync is aborted
	 * midway, we need to be able to drop that count, so we remember
	 * the counted chunk..
	 */
	unsigned long syncchunk;

	__u64	events_cleared;
	int need_sync;

	/* bitmap spinlock */
	spinlock_t lock;

	struct file *file; /* backing disk file */
	struct page *sb_page; /* cached copy of the bitmap file superblock */
	struct page **filemap; /* list of cache pages for the file */
	unsigned long *filemap_attr; /* attributes associated w/ filemap pages */
	unsigned long file_pages; /* number of pages in the file */
	int last_page_size; /* bytes in the last page */

	unsigned long flags;

	int allclean;

	atomic_t behind_writes;
	unsigned long behind_writes_used; /* highest actual value at runtime */

	/*
	 * the bitmap daemon - periodically wakes up and sweeps the bitmap
	 * file, cleaning up bits and flushing out pages to disk as necessary
	 */
	unsigned long daemon_lastrun; /* jiffies of last run */
	unsigned long last_end_sync; /* when we lasted called end_sync to
				      * update bitmap with resync progress */

	atomic_t pending_writes; /* pending writes to the bitmap file */
	wait_queue_head_t write_wait;
	wait_queue_head_t overflow_wait;
	wait_queue_head_t behind_wait;

	struct sysfs_dirent *sysfs_can_clear;
};

/* the bitmap API */

/* these are used only by md/bitmap */
int  bitmap_create(mddev_t *mddev);
void bitmap_flush(mddev_t *mddev);
void bitmap_destroy(mddev_t *mddev);

void bitmap_print_sb(struct bitmap *bitmap);
void bitmap_update_sb(struct bitmap *bitmap);

int  bitmap_setallbits(struct bitmap *bitmap);
void bitmap_write_all(struct bitmap *bitmap);

void bitmap_dirty_bits(struct bitmap *bitmap, unsigned long s, unsigned long e);

/* these are exported */
int bitmap_startwrite(struct bitmap *bitmap, sector_t offset,
			unsigned long sectors, int behind);
void bitmap_endwrite(struct bitmap *bitmap, sector_t offset,
			unsigned long sectors, int success, int behind);
int bitmap_start_sync(struct bitmap *bitmap, sector_t offset, int *blocks, int degraded);
void bitmap_end_sync(struct bitmap *bitmap, sector_t offset, int *blocks, int aborted);
void bitmap_close_sync(struct bitmap *bitmap);
void bitmap_cond_end_sync(struct bitmap *bitmap, sector_t sector);

void bitmap_unplug(struct bitmap *bitmap);
void bitmap_daemon_work(mddev_t *mddev);
#endif

#endif
