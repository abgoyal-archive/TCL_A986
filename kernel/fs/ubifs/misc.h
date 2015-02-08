


#ifndef __UBIFS_MISC_H__
#define __UBIFS_MISC_H__

static inline int ubifs_zn_dirty(const struct ubifs_znode *znode)
{
	return !!test_bit(DIRTY_ZNODE, &znode->flags);
}

static inline void ubifs_wake_up_bgt(struct ubifs_info *c)
{
	if (c->bgt && !c->need_bgt) {
		c->need_bgt = 1;
		wake_up_process(c->bgt);
	}
}

static inline struct ubifs_znode *
ubifs_tnc_find_child(struct ubifs_znode *znode, int start)
{
	while (start < znode->child_cnt) {
		if (znode->zbranch[start].znode)
			return znode->zbranch[start].znode;
		start += 1;
	}

	return NULL;
}

static inline struct ubifs_inode *ubifs_inode(const struct inode *inode)
{
	return container_of(inode, struct ubifs_inode, vfs_inode);
}

static inline int ubifs_compr_present(int compr_type)
{
	ubifs_assert(compr_type >= 0 && compr_type < UBIFS_COMPR_TYPES_CNT);
	return !!ubifs_compressors[compr_type]->capi_name;
}

static inline const char *ubifs_compr_name(int compr_type)
{
	ubifs_assert(compr_type >= 0 && compr_type < UBIFS_COMPR_TYPES_CNT);
	return ubifs_compressors[compr_type]->name;
}

static inline int ubifs_wbuf_sync(struct ubifs_wbuf *wbuf)
{
	int err;

	mutex_lock_nested(&wbuf->io_mutex, wbuf->jhead);
	err = ubifs_wbuf_sync_nolock(wbuf);
	mutex_unlock(&wbuf->io_mutex);
	return err;
}

static inline int ubifs_leb_unmap(const struct ubifs_info *c, int lnum)
{
	int err;

	if (c->ro_media)
		return -EROFS;
	err = ubi_leb_unmap(c->ubi, lnum);
	if (err) {
		ubifs_err("unmap LEB %d failed, error %d", lnum, err);
		return err;
	}

	return 0;
}

static inline int ubifs_leb_write(const struct ubifs_info *c, int lnum,
				  const void *buf, int offs, int len, int dtype)
{
	int err;

	if (c->ro_media)
		return -EROFS;
	err = ubi_leb_write(c->ubi, lnum, buf, offs, len, dtype);
	if (err) {
		ubifs_err("writing %d bytes at %d:%d, error %d",
			  len, lnum, offs, err);
		return err;
	}

	return 0;
}

static inline int ubifs_leb_change(const struct ubifs_info *c, int lnum,
				   const void *buf, int len, int dtype)
{
	int err;

	if (c->ro_media)
		return -EROFS;
	err = ubi_leb_change(c->ubi, lnum, buf, len, dtype);
	if (err) {
		ubifs_err("changing %d bytes in LEB %d, error %d",
			  len, lnum, err);
		return err;
	}

	return 0;
}

static inline int ubifs_encode_dev(union ubifs_dev_desc *dev, dev_t rdev)
{
	if (new_valid_dev(rdev)) {
		dev->new = cpu_to_le32(new_encode_dev(rdev));
		return sizeof(dev->new);
	} else {
		dev->huge = cpu_to_le64(huge_encode_dev(rdev));
		return sizeof(dev->huge);
	}
}

static inline int ubifs_add_dirt(struct ubifs_info *c, int lnum, int dirty)
{
	return ubifs_update_one_lp(c, lnum, LPROPS_NC, dirty, 0, 0);
}

static inline int ubifs_return_leb(struct ubifs_info *c, int lnum)
{
	return ubifs_change_one_lp(c, lnum, LPROPS_NC, LPROPS_NC, 0,
				   LPROPS_TAKEN, 0);
}

static inline int ubifs_idx_node_sz(const struct ubifs_info *c, int child_cnt)
{
	return UBIFS_IDX_NODE_SZ + (UBIFS_BRANCH_SZ + c->key_len) * child_cnt;
}

static inline
struct ubifs_branch *ubifs_idx_branch(const struct ubifs_info *c,
				      const struct ubifs_idx_node *idx,
				      int bnum)
{
	return (struct ubifs_branch *)((void *)idx->branches +
				       (UBIFS_BRANCH_SZ + c->key_len) * bnum);
}

static inline void *ubifs_idx_key(const struct ubifs_info *c,
				  const struct ubifs_idx_node *idx)
{
	return (void *)((struct ubifs_branch *)idx->branches)->key;
}

static inline struct timespec ubifs_current_time(struct inode *inode)
{
	return (inode->i_sb->s_time_gran < NSEC_PER_SEC) ?
		current_fs_time(inode->i_sb) : CURRENT_TIME_SEC;
}

static inline int ubifs_tnc_lookup(struct ubifs_info *c,
				   const union ubifs_key *key, void *node)
{
	return ubifs_tnc_locate(c, key, node, NULL, NULL);
}

static inline void ubifs_get_lprops(struct ubifs_info *c)
{
	mutex_lock(&c->lp_mutex);
}

static inline void ubifs_release_lprops(struct ubifs_info *c)
{
	ubifs_assert(mutex_is_locked(&c->lp_mutex));
	ubifs_assert(c->lst.empty_lebs >= 0 &&
		     c->lst.empty_lebs <= c->main_lebs);
	mutex_unlock(&c->lp_mutex);
}

#endif /* __UBIFS_MISC_H__ */
