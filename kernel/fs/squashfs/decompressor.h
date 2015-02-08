
#ifndef DECOMPRESSOR_H
#define DECOMPRESSOR_H

struct squashfs_decompressor {
	void	*(*init)(struct squashfs_sb_info *);
	void	(*free)(void *);
	int	(*decompress)(struct squashfs_sb_info *, void **,
		struct buffer_head **, int, int, int, int, int);
	int	id;
	char	*name;
	int	supported;
};

static inline void *squashfs_decompressor_init(struct squashfs_sb_info *msblk)
{
	return msblk->decompressor->init(msblk);
}

static inline void squashfs_decompressor_free(struct squashfs_sb_info *msblk,
	void *s)
{
	if (msblk->decompressor)
		msblk->decompressor->free(s);
}

static inline int squashfs_decompress(struct squashfs_sb_info *msblk,
	void **buffer, struct buffer_head **bh, int b, int offset, int length,
	int srclength, int pages)
{
	return msblk->decompressor->decompress(msblk, buffer, bh, b, offset,
		length, srclength, pages);
}
#endif
