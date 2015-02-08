


struct gen_pool {
	rwlock_t lock;
	struct list_head chunks;	/* list of chunks in this pool */
	int min_alloc_order;		/* minimum allocation order */
};

struct gen_pool_chunk {
	spinlock_t lock;
	struct list_head next_chunk;	/* next chunk in pool */
	unsigned long start_addr;	/* starting address of memory chunk */
	unsigned long end_addr;		/* ending address of memory chunk */
	unsigned long bits[0];		/* bitmap for allocating memory chunk */
};

extern struct gen_pool *gen_pool_create(int, int);
extern int gen_pool_add(struct gen_pool *, unsigned long, size_t, int);
extern void gen_pool_destroy(struct gen_pool *);
extern unsigned long gen_pool_alloc(struct gen_pool *, size_t);
extern void gen_pool_free(struct gen_pool *, unsigned long, size_t);
