
#ifndef __XFS_IOMAP_H__
#define __XFS_IOMAP_H__

typedef enum {
	/* base extent manipulation calls */
	BMAPI_READ = (1 << 0),		/* read extents */
	BMAPI_WRITE = (1 << 1),		/* create extents */
	BMAPI_ALLOCATE = (1 << 2),	/* delayed allocate to real extents */
	/* modifiers */
	BMAPI_IGNSTATE = (1 << 4),	/* ignore unwritten state on read */
	BMAPI_DIRECT = (1 << 5),	/* direct instead of buffered write */
	BMAPI_MMAP = (1 << 6),		/* allocate for mmap write */
	BMAPI_TRYLOCK = (1 << 7),	/* non-blocking request */
} bmapi_flags_t;

#define BMAPI_FLAGS \
	{ BMAPI_READ,		"READ" }, \
	{ BMAPI_WRITE,		"WRITE" }, \
	{ BMAPI_ALLOCATE,	"ALLOCATE" }, \
	{ BMAPI_IGNSTATE,	"IGNSTATE" }, \
	{ BMAPI_DIRECT,		"DIRECT" }, \
	{ BMAPI_MMAP,		"MMAP" }, \
	{ BMAPI_TRYLOCK,	"TRYLOCK" }

struct xfs_inode;
struct xfs_bmbt_irec;

extern int xfs_iomap(struct xfs_inode *, xfs_off_t, ssize_t, int,
		     struct xfs_bmbt_irec *, int *, int *);
extern int xfs_iomap_write_unwritten(struct xfs_inode *, xfs_off_t, size_t);

#endif /* __XFS_IOMAP_H__*/
