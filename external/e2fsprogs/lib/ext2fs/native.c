
#include <stdio.h>

#include "ext2_fs.h"
#include "ext2fs.h"

int ext2fs_native_flag(void)
{
#ifdef WORDS_BIGENDIAN
	return EXT2_FLAG_SWAP_BYTES;
#else
	return 0;
#endif
}

	
	
