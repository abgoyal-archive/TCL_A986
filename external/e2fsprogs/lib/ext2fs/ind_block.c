
#include <stdio.h>
#include <string.h>
#if HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "ext2_fs.h"
#include "ext2fs.h"

errcode_t ext2fs_read_ind_block(ext2_filsys fs, blk_t blk, void *buf)
{
	errcode_t	retval;
	blk_t		*block_nr;
	int		i;
	int		limit = fs->blocksize >> 2;

	if ((fs->flags & EXT2_FLAG_IMAGE_FILE) &&
	    (fs->io != fs->image_io))
		memset(buf, 0, fs->blocksize);
	else {
		retval = io_channel_read_blk(fs->io, blk, 1, buf);
		if (retval)
			return retval;
	}
#ifdef EXT2FS_ENABLE_SWAPFS
	if (fs->flags & (EXT2_FLAG_SWAP_BYTES | EXT2_FLAG_SWAP_BYTES_READ)) {
		block_nr = (blk_t *) buf;
		for (i = 0; i < limit; i++, block_nr++)
			*block_nr = ext2fs_swab32(*block_nr);
	}
#endif
	return 0;
}

errcode_t ext2fs_write_ind_block(ext2_filsys fs, blk_t blk, void *buf)
{
	blk_t		*block_nr;
	int		i;
	int		limit = fs->blocksize >> 2;

	if (fs->flags & EXT2_FLAG_IMAGE_FILE)
		return 0;

#ifdef EXT2FS_ENABLE_SWAPFS
	if (fs->flags & (EXT2_FLAG_SWAP_BYTES | EXT2_FLAG_SWAP_BYTES_WRITE)) {
		block_nr = (blk_t *) buf;
		for (i = 0; i < limit; i++, block_nr++)
			*block_nr = ext2fs_swab32(*block_nr);
	}
#endif
	return io_channel_write_blk(fs->io, blk, 1, buf);
}


