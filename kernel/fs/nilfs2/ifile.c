

#include <linux/types.h>
#include <linux/buffer_head.h>
#include "nilfs.h"
#include "mdt.h"
#include "alloc.h"
#include "ifile.h"


struct nilfs_ifile_info {
	struct nilfs_mdt_info mi;
	struct nilfs_palloc_cache palloc_cache;
};

static inline struct nilfs_ifile_info *NILFS_IFILE_I(struct inode *ifile)
{
	return (struct nilfs_ifile_info *)NILFS_MDT(ifile);
}

int nilfs_ifile_create_inode(struct inode *ifile, ino_t *out_ino,
			     struct buffer_head **out_bh)
{
	struct nilfs_palloc_req req;
	int ret;

	req.pr_entry_nr = 0;  /* 0 says find free inode from beginning of
				 a group. dull code!! */
	req.pr_entry_bh = NULL;

	ret = nilfs_palloc_prepare_alloc_entry(ifile, &req);
	if (!ret) {
		ret = nilfs_palloc_get_entry_block(ifile, req.pr_entry_nr, 1,
						   &req.pr_entry_bh);
		if (ret < 0)
			nilfs_palloc_abort_alloc_entry(ifile, &req);
	}
	if (ret < 0) {
		brelse(req.pr_entry_bh);
		return ret;
	}
	nilfs_palloc_commit_alloc_entry(ifile, &req);
	nilfs_mdt_mark_buffer_dirty(req.pr_entry_bh);
	nilfs_mdt_mark_dirty(ifile);
	*out_ino = (ino_t)req.pr_entry_nr;
	*out_bh = req.pr_entry_bh;
	return 0;
}

int nilfs_ifile_delete_inode(struct inode *ifile, ino_t ino)
{
	struct nilfs_palloc_req req = {
		.pr_entry_nr = ino, .pr_entry_bh = NULL
	};
	struct nilfs_inode *raw_inode;
	void *kaddr;
	int ret;

	ret = nilfs_palloc_prepare_free_entry(ifile, &req);
	if (!ret) {
		ret = nilfs_palloc_get_entry_block(ifile, req.pr_entry_nr, 0,
						   &req.pr_entry_bh);
		if (ret < 0)
			nilfs_palloc_abort_free_entry(ifile, &req);
	}
	if (ret < 0) {
		brelse(req.pr_entry_bh);
		return ret;
	}

	kaddr = kmap_atomic(req.pr_entry_bh->b_page, KM_USER0);
	raw_inode = nilfs_palloc_block_get_entry(ifile, req.pr_entry_nr,
						 req.pr_entry_bh, kaddr);
	raw_inode->i_flags = 0;
	kunmap_atomic(kaddr, KM_USER0);

	nilfs_mdt_mark_buffer_dirty(req.pr_entry_bh);
	brelse(req.pr_entry_bh);

	nilfs_palloc_commit_free_entry(ifile, &req);

	return 0;
}

int nilfs_ifile_get_inode_block(struct inode *ifile, ino_t ino,
				struct buffer_head **out_bh)
{
	struct super_block *sb = ifile->i_sb;
	int err;

	if (unlikely(!NILFS_VALID_INODE(sb, ino))) {
		nilfs_error(sb, __func__, "bad inode number: %lu",
			    (unsigned long) ino);
		return -EINVAL;
	}

	err = nilfs_palloc_get_entry_block(ifile, ino, 0, out_bh);
	if (unlikely(err)) {
		if (err == -EINVAL)
			nilfs_error(sb, __func__, "ifile is broken");
		else
			nilfs_warning(sb, __func__,
				      "unable to read inode: %lu",
				      (unsigned long) ino);
	}
	return err;
}

struct inode *nilfs_ifile_new(struct nilfs_sb_info *sbi, size_t inode_size)
{
	struct inode *ifile;
	int err;

	ifile = nilfs_mdt_new(sbi->s_nilfs, sbi->s_super, NILFS_IFILE_INO,
			      sizeof(struct nilfs_ifile_info));
	if (ifile) {
		err = nilfs_palloc_init_blockgroup(ifile, inode_size);
		if (unlikely(err)) {
			nilfs_mdt_destroy(ifile);
			return NULL;
		}
		nilfs_palloc_setup_cache(ifile,
					 &NILFS_IFILE_I(ifile)->palloc_cache);
	}
	return ifile;
}
