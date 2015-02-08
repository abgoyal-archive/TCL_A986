
#include "ext2fs.h"

struct ext2_struct_u32_list {
	int	magic;
	int	num;
	int	size;
	__u32	*list;
	int	badblocks_flags;
};

struct ext2_struct_u32_iterate {
	int			magic;
	ext2_u32_list		bb;
	int			ptr;
};


struct ext2_struct_dblist {
	int			magic;
	ext2_filsys		fs;
	ext2_ino_t		size;
	ext2_ino_t		count;
	int			sorted;
	struct ext2_db_entry *	list;
};

struct dir_context {
	ext2_ino_t		dir;
	int		flags;
	char		*buf;
	int (*func)(ext2_ino_t	dir,
		    int	entry,
		    struct ext2_dir_entry *dirent,
		    int	offset,
		    int	blocksize,
		    char	*buf,
		    void	*priv_data);
	void		*priv_data;
	errcode_t	errcode;
};

struct ext2_inode_cache {
	void *				buffer;
	blk_t				buffer_blk;
	int				cache_last;
	int				cache_size;
	int				refcount;
	struct ext2_inode_cache_ent	*cache;
};

struct ext2_inode_cache_ent {
	ext2_ino_t		ino;
	struct ext2_inode	inode;
};

/* Function prototypes */

extern int ext2fs_process_dir_block(ext2_filsys  	fs,
				    blk_t		*blocknr,
				    e2_blkcnt_t		blockcnt,
				    blk_t		ref_block,
				    int			ref_offset,
				    void		*priv_data);


