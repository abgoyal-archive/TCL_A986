
#ifndef __XFS_BIT_H__
#define	__XFS_BIT_H__


static inline __uint64_t xfs_mask64hi(int n)
{
	return (__uint64_t)-1 << (64 - (n));
}
static inline __uint32_t xfs_mask32lo(int n)
{
	return ((__uint32_t)1 << (n)) - 1;
}
static inline __uint64_t xfs_mask64lo(int n)
{
	return ((__uint64_t)1 << (n)) - 1;
}

/* Get high bit set out of 32-bit argument, -1 if none set */
static inline int xfs_highbit32(__uint32_t v)
{
	return fls(v) - 1;
}

/* Get high bit set out of 64-bit argument, -1 if none set */
static inline int xfs_highbit64(__uint64_t v)
{
	return fls64(v) - 1;
}

/* Get low bit set out of 32-bit argument, -1 if none set */
static inline int xfs_lowbit32(__uint32_t v)
{
	return ffs(v) - 1;
}

/* Get low bit set out of 64-bit argument, -1 if none set */
static inline int xfs_lowbit64(__uint64_t v)
{
	__uint32_t	w = (__uint32_t)v;
	int		n = 0;

	if (w) {	/* lower bits */
		n = ffs(w);
	} else {	/* upper bits */
		w = (__uint32_t)(v >> 32);
		if (w && (n = ffs(w)))
		n += 32;
	}
	return n - 1;
}

/* Return whether bitmap is empty (1 == empty) */
extern int xfs_bitmap_empty(uint *map, uint size);

/* Count continuous one bits in map starting with start_bit */
extern int xfs_contig_bits(uint *map, uint size, uint start_bit);

/* Find next set bit in map */
extern int xfs_next_bit(uint *map, uint size, uint start_bit);

#endif	/* __XFS_BIT_H__ */
