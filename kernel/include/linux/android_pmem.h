

#ifndef _ANDROID_PMEM_H_
#define _ANDROID_PMEM_H_

#define PMEM_IOCTL_MAGIC 'p'
#define PMEM_GET_PHYS		_IOW(PMEM_IOCTL_MAGIC, 1, unsigned int)
#define PMEM_MAP		_IOW(PMEM_IOCTL_MAGIC, 2, unsigned int)
#define PMEM_GET_SIZE		_IOW(PMEM_IOCTL_MAGIC, 3, unsigned int)
#define PMEM_UNMAP		_IOW(PMEM_IOCTL_MAGIC, 4, unsigned int)
#define PMEM_ALLOCATE		_IOW(PMEM_IOCTL_MAGIC, 5, unsigned int)
#define PMEM_CONNECT		_IOW(PMEM_IOCTL_MAGIC, 6, unsigned int)
#define PMEM_GET_TOTAL_SIZE	_IOW(PMEM_IOCTL_MAGIC, 7, unsigned int)
#define PMEM_CACHE_FLUSH	_IOW(PMEM_IOCTL_MAGIC, 8, unsigned int)

struct android_pmem_platform_data
{
	const char* name;
	/* starting physical address of memory region */
	unsigned long start;
	/* size of memory region */
	unsigned long size;
	/* set to indicate the region should not be managed with an allocator */
	unsigned no_allocator;
	/* set to indicate maps of this region should be cached, if a mix of
	 * cached and uncached is desired, set this and open the device with
	 * O_SYNC to get an uncached region */
	unsigned cached;
	/* The MSM7k has bits to enable a write buffer in the bus controller*/
	unsigned buffered;
};

struct pmem_region {
	unsigned long offset;
	unsigned long len;
};

#ifdef CONFIG_ANDROID_PMEM
int is_pmem_file(struct file *file);
int get_pmem_file(int fd, unsigned long *start, unsigned long *vstart,
		  unsigned long *end, struct file **filp);
int get_pmem_user_addr(struct file *file, unsigned long *start,
		       unsigned long *end);
void put_pmem_file(struct file* file);
void flush_pmem_file(struct file *file, unsigned long start, unsigned long len);
int pmem_setup(struct android_pmem_platform_data *pdata,
	       long (*ioctl)(struct file *, unsigned int, unsigned long),
	       int (*release)(struct inode *, struct file *));
int pmem_remap(struct pmem_region *region, struct file *file,
	       unsigned operation);

#else
static inline int is_pmem_file(struct file *file) { return 0; }
static inline int get_pmem_file(int fd, unsigned long *start,
				unsigned long *vstart, unsigned long *end,
				struct file **filp) { return -ENOSYS; }
static inline int get_pmem_user_addr(struct file *file, unsigned long *start,
				     unsigned long *end) { return -ENOSYS; }
static inline void put_pmem_file(struct file* file) { return; }
static inline void flush_pmem_file(struct file *file, unsigned long start,
				   unsigned long len) { return; }
static inline int pmem_setup(struct android_pmem_platform_data *pdata,
	      long (*ioctl)(struct file *, unsigned int, unsigned long),
	      int (*release)(struct inode *, struct file *)) { return -ENOSYS; }

static inline int pmem_remap(struct pmem_region *region, struct file *file,
			     unsigned operation) { return -ENOSYS; }
#endif

#endif //_ANDROID_PPP_H_

