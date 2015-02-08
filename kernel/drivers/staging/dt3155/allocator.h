

int allocator_free_dma(unsigned long address);
unsigned long allocator_allocate_dma(unsigned long kilobytes, gfp_t flags);
int allocator_init(u32 *);
void allocator_cleanup(void);
