


#include <linux/crc32.h>
#include <linux/slab.h>
#include "ubifs.h"

enum {
	NAME_LESS    = 0,
	NAME_MATCHES = 1,
	NAME_GREATER = 2,
	NOT_ON_MEDIA = 3,
};

static int insert_old_idx(struct ubifs_info *c, int lnum, int offs)
{
	struct ubifs_old_idx *old_idx, *o;
	struct rb_node **p, *parent = NULL;

	old_idx = kmalloc(sizeof(struct ubifs_old_idx), GFP_NOFS);
	if (unlikely(!old_idx))
		return -ENOMEM;
	old_idx->lnum = lnum;
	old_idx->offs = offs;

	p = &c->old_idx.rb_node;
	while (*p) {
		parent = *p;
		o = rb_entry(parent, struct ubifs_old_idx, rb);
		if (lnum < o->lnum)
			p = &(*p)->rb_left;
		else if (lnum > o->lnum)
			p = &(*p)->rb_right;
		else if (offs < o->offs)
			p = &(*p)->rb_left;
		else if (offs > o->offs)
			p = &(*p)->rb_right;
		else {
			ubifs_err("old idx added twice!");
			kfree(old_idx);
			return 0;
		}
	}
	rb_link_node(&old_idx->rb, parent, p);
	rb_insert_color(&old_idx->rb, &c->old_idx);
	return 0;
}

int insert_old_idx_znode(struct ubifs_info *c, struct ubifs_znode *znode)
{
	if (znode->parent) {
		struct ubifs_zbranch *zbr;

		zbr = &znode->parent->zbranch[znode->iip];
		if (zbr->len)
			return insert_old_idx(c, zbr->lnum, zbr->offs);
	} else
		if (c->zroot.len)
			return insert_old_idx(c, c->zroot.lnum,
					      c->zroot.offs);
	return 0;
}

static int ins_clr_old_idx_znode(struct ubifs_info *c,
				 struct ubifs_znode *znode)
{
	int err;

	if (znode->parent) {
		struct ubifs_zbranch *zbr;

		zbr = &znode->parent->zbranch[znode->iip];
		if (zbr->len) {
			err = insert_old_idx(c, zbr->lnum, zbr->offs);
			if (err)
				return err;
			zbr->lnum = 0;
			zbr->offs = 0;
			zbr->len = 0;
		}
	} else
		if (c->zroot.len) {
			err = insert_old_idx(c, c->zroot.lnum, c->zroot.offs);
			if (err)
				return err;
			c->zroot.lnum = 0;
			c->zroot.offs = 0;
			c->zroot.len = 0;
		}
	return 0;
}

void destroy_old_idx(struct ubifs_info *c)
{
	struct rb_node *this = c->old_idx.rb_node;
	struct ubifs_old_idx *old_idx;

	while (this) {
		if (this->rb_left) {
			this = this->rb_left;
			continue;
		} else if (this->rb_right) {
			this = this->rb_right;
			continue;
		}
		old_idx = rb_entry(this, struct ubifs_old_idx, rb);
		this = rb_parent(this);
		if (this) {
			if (this->rb_left == &old_idx->rb)
				this->rb_left = NULL;
			else
				this->rb_right = NULL;
		}
		kfree(old_idx);
	}
	c->old_idx = RB_ROOT;
}

static struct ubifs_znode *copy_znode(struct ubifs_info *c,
				      struct ubifs_znode *znode)
{
	struct ubifs_znode *zn;

	zn = kmalloc(c->max_znode_sz, GFP_NOFS);
	if (unlikely(!zn))
		return ERR_PTR(-ENOMEM);

	memcpy(zn, znode, c->max_znode_sz);
	zn->cnext = NULL;
	__set_bit(DIRTY_ZNODE, &zn->flags);
	__clear_bit(COW_ZNODE, &zn->flags);

	ubifs_assert(!test_bit(OBSOLETE_ZNODE, &znode->flags));
	__set_bit(OBSOLETE_ZNODE, &znode->flags);

	if (znode->level != 0) {
		int i;
		const int n = zn->child_cnt;

		/* The children now have new parent */
		for (i = 0; i < n; i++) {
			struct ubifs_zbranch *zbr = &zn->zbranch[i];

			if (zbr->znode)
				zbr->znode->parent = zn;
		}
	}

	atomic_long_inc(&c->dirty_zn_cnt);
	return zn;
}

static int add_idx_dirt(struct ubifs_info *c, int lnum, int dirt)
{
	c->calc_idx_sz -= ALIGN(dirt, 8);
	return ubifs_add_dirt(c, lnum, dirt);
}

static struct ubifs_znode *dirty_cow_znode(struct ubifs_info *c,
					   struct ubifs_zbranch *zbr)
{
	struct ubifs_znode *znode = zbr->znode;
	struct ubifs_znode *zn;
	int err;

	if (!test_bit(COW_ZNODE, &znode->flags)) {
		/* znode is not being committed */
		if (!test_and_set_bit(DIRTY_ZNODE, &znode->flags)) {
			atomic_long_inc(&c->dirty_zn_cnt);
			atomic_long_dec(&c->clean_zn_cnt);
			atomic_long_dec(&ubifs_clean_zn_cnt);
			err = add_idx_dirt(c, zbr->lnum, zbr->len);
			if (unlikely(err))
				return ERR_PTR(err);
		}
		return znode;
	}

	zn = copy_znode(c, znode);
	if (IS_ERR(zn))
		return zn;

	if (zbr->len) {
		err = insert_old_idx(c, zbr->lnum, zbr->offs);
		if (unlikely(err))
			return ERR_PTR(err);
		err = add_idx_dirt(c, zbr->lnum, zbr->len);
	} else
		err = 0;

	zbr->znode = zn;
	zbr->lnum = 0;
	zbr->offs = 0;
	zbr->len = 0;

	if (unlikely(err))
		return ERR_PTR(err);
	return zn;
}

static int lnc_add(struct ubifs_info *c, struct ubifs_zbranch *zbr,
		   const void *node)
{
	int err;
	void *lnc_node;
	const struct ubifs_dent_node *dent = node;

	ubifs_assert(!zbr->leaf);
	ubifs_assert(zbr->len != 0);
	ubifs_assert(is_hash_key(c, &zbr->key));

	err = ubifs_validate_entry(c, dent);
	if (err) {
		dbg_dump_stack();
		dbg_dump_node(c, dent);
		return err;
	}

	lnc_node = kmalloc(zbr->len, GFP_NOFS);
	if (!lnc_node)
		/* We don't have to have the cache, so no error */
		return 0;

	memcpy(lnc_node, node, zbr->len);
	zbr->leaf = lnc_node;
	return 0;
}

 /**
 * lnc_add_directly - add a leaf node to the leaf-node-cache.
 * @c: UBIFS file-system description object
 * @zbr: zbranch of leaf node
 * @node: leaf node
 *
 * This function is similar to 'lnc_add()', but it does not create a copy of
 * @node but inserts @node to TNC directly.
 */
static int lnc_add_directly(struct ubifs_info *c, struct ubifs_zbranch *zbr,
			    void *node)
{
	int err;

	ubifs_assert(!zbr->leaf);
	ubifs_assert(zbr->len != 0);

	err = ubifs_validate_entry(c, node);
	if (err) {
		dbg_dump_stack();
		dbg_dump_node(c, node);
		return err;
	}

	zbr->leaf = node;
	return 0;
}

static void lnc_free(struct ubifs_zbranch *zbr)
{
	if (!zbr->leaf)
		return;
	kfree(zbr->leaf);
	zbr->leaf = NULL;
}

static int tnc_read_node_nm(struct ubifs_info *c, struct ubifs_zbranch *zbr,
			    void *node)
{
	int err;

	ubifs_assert(is_hash_key(c, &zbr->key));

	if (zbr->leaf) {
		/* Read from the leaf node cache */
		ubifs_assert(zbr->len != 0);
		memcpy(node, zbr->leaf, zbr->len);
		return 0;
	}

	err = ubifs_tnc_read_node(c, zbr, node);
	if (err)
		return err;

	/* Add the node to the leaf node cache */
	err = lnc_add(c, zbr, node);
	return err;
}

static int try_read_node(const struct ubifs_info *c, void *buf, int type,
			 int len, int lnum, int offs)
{
	int err, node_len;
	struct ubifs_ch *ch = buf;
	uint32_t crc, node_crc;

	dbg_io("LEB %d:%d, %s, length %d", lnum, offs, dbg_ntype(type), len);

	err = ubi_read(c->ubi, lnum, buf, offs, len);
	if (err) {
		ubifs_err("cannot read node type %d from LEB %d:%d, error %d",
			  type, lnum, offs, err);
		return err;
	}

	if (le32_to_cpu(ch->magic) != UBIFS_NODE_MAGIC)
		return 0;

	if (ch->node_type != type)
		return 0;

	node_len = le32_to_cpu(ch->len);
	if (node_len != len)
		return 0;

	if (type == UBIFS_DATA_NODE && !c->always_chk_crc && c->no_chk_data_crc)
		return 1;

	crc = crc32(UBIFS_CRC32_INIT, buf + 8, node_len - 8);
	node_crc = le32_to_cpu(ch->crc);
	if (crc != node_crc)
		return 0;

	return 1;
}

static int fallible_read_node(struct ubifs_info *c, const union ubifs_key *key,
			      struct ubifs_zbranch *zbr, void *node)
{
	int ret;

	dbg_tnc("LEB %d:%d, key %s", zbr->lnum, zbr->offs, DBGKEY(key));

	ret = try_read_node(c, node, key_type(c, key), zbr->len, zbr->lnum,
			    zbr->offs);
	if (ret == 1) {
		union ubifs_key node_key;
		struct ubifs_dent_node *dent = node;

		/* All nodes have key in the same place */
		key_read(c, &dent->key, &node_key);
		if (keys_cmp(c, key, &node_key) != 0)
			ret = 0;
	}
	if (ret == 0 && c->replaying)
		dbg_mnt("dangling branch LEB %d:%d len %d, key %s",
			zbr->lnum, zbr->offs, zbr->len, DBGKEY(key));
	return ret;
}

static int matches_name(struct ubifs_info *c, struct ubifs_zbranch *zbr,
			const struct qstr *nm)
{
	struct ubifs_dent_node *dent;
	int nlen, err;

	/* If possible, match against the dent in the leaf node cache */
	if (!zbr->leaf) {
		dent = kmalloc(zbr->len, GFP_NOFS);
		if (!dent)
			return -ENOMEM;

		err = ubifs_tnc_read_node(c, zbr, dent);
		if (err)
			goto out_free;

		/* Add the node to the leaf node cache */
		err = lnc_add_directly(c, zbr, dent);
		if (err)
			goto out_free;
	} else
		dent = zbr->leaf;

	nlen = le16_to_cpu(dent->nlen);
	err = memcmp(dent->name, nm->name, min_t(int, nlen, nm->len));
	if (err == 0) {
		if (nlen == nm->len)
			return NAME_MATCHES;
		else if (nlen < nm->len)
			return NAME_LESS;
		else
			return NAME_GREATER;
	} else if (err < 0)
		return NAME_LESS;
	else
		return NAME_GREATER;

out_free:
	kfree(dent);
	return err;
}

static struct ubifs_znode *get_znode(struct ubifs_info *c,
				     struct ubifs_znode *znode, int n)
{
	struct ubifs_zbranch *zbr;

	zbr = &znode->zbranch[n];
	if (zbr->znode)
		znode = zbr->znode;
	else
		znode = ubifs_load_znode(c, zbr, znode, n);
	return znode;
}

static int tnc_next(struct ubifs_info *c, struct ubifs_znode **zn, int *n)
{
	struct ubifs_znode *znode = *zn;
	int nn = *n;

	nn += 1;
	if (nn < znode->child_cnt) {
		*n = nn;
		return 0;
	}
	while (1) {
		struct ubifs_znode *zp;

		zp = znode->parent;
		if (!zp)
			return -ENOENT;
		nn = znode->iip + 1;
		znode = zp;
		if (nn < znode->child_cnt) {
			znode = get_znode(c, znode, nn);
			if (IS_ERR(znode))
				return PTR_ERR(znode);
			while (znode->level != 0) {
				znode = get_znode(c, znode, 0);
				if (IS_ERR(znode))
					return PTR_ERR(znode);
			}
			nn = 0;
			break;
		}
	}
	*zn = znode;
	*n = nn;
	return 0;
}

static int tnc_prev(struct ubifs_info *c, struct ubifs_znode **zn, int *n)
{
	struct ubifs_znode *znode = *zn;
	int nn = *n;

	if (nn > 0) {
		*n = nn - 1;
		return 0;
	}
	while (1) {
		struct ubifs_znode *zp;

		zp = znode->parent;
		if (!zp)
			return -ENOENT;
		nn = znode->iip - 1;
		znode = zp;
		if (nn >= 0) {
			znode = get_znode(c, znode, nn);
			if (IS_ERR(znode))
				return PTR_ERR(znode);
			while (znode->level != 0) {
				nn = znode->child_cnt - 1;
				znode = get_znode(c, znode, nn);
				if (IS_ERR(znode))
					return PTR_ERR(znode);
			}
			nn = znode->child_cnt - 1;
			break;
		}
	}
	*zn = znode;
	*n = nn;
	return 0;
}

static int resolve_collision(struct ubifs_info *c, const union ubifs_key *key,
			     struct ubifs_znode **zn, int *n,
			     const struct qstr *nm)
{
	int err;

	err = matches_name(c, &(*zn)->zbranch[*n], nm);
	if (unlikely(err < 0))
		return err;
	if (err == NAME_MATCHES)
		return 1;

	if (err == NAME_GREATER) {
		/* Look left */
		while (1) {
			err = tnc_prev(c, zn, n);
			if (err == -ENOENT) {
				ubifs_assert(*n == 0);
				*n = -1;
				return 0;
			}
			if (err < 0)
				return err;
			if (keys_cmp(c, &(*zn)->zbranch[*n].key, key)) {
				/*
				 * We have found the branch after which we would
				 * like to insert, but inserting in this znode
				 * may still be wrong. Consider the following 3
				 * znodes, in the case where we are resolving a
				 * collision with Key2.
				 *
				 *                  znode zp
				 *            ----------------------
				 * level 1     |  Key0  |  Key1  |
				 *            -----------------------
				 *                 |            |
				 *       znode za  |            |  znode zb
				 *          ------------      ------------
				 * level 0  |  Key0  |        |  Key2  |
				 *          ------------      ------------
				 *
				 * The lookup finds Key2 in znode zb. Lets say
				 * there is no match and the name is greater so
				 * we look left. When we find Key0, we end up
				 * here. If we return now, we will insert into
				 * znode za at slot n = 1.  But that is invalid
				 * according to the parent's keys.  Key2 must
				 * be inserted into znode zb.
				 *
				 * Note, this problem is not relevant for the
				 * case when we go right, because
				 * 'tnc_insert()' would correct the parent key.
				 */
				if (*n == (*zn)->child_cnt - 1) {
					err = tnc_next(c, zn, n);
					if (err) {
						/* Should be impossible */
						ubifs_assert(0);
						if (err == -ENOENT)
							err = -EINVAL;
						return err;
					}
					ubifs_assert(*n == 0);
					*n = -1;
				}
				return 0;
			}
			err = matches_name(c, &(*zn)->zbranch[*n], nm);
			if (err < 0)
				return err;
			if (err == NAME_LESS)
				return 0;
			if (err == NAME_MATCHES)
				return 1;
			ubifs_assert(err == NAME_GREATER);
		}
	} else {
		int nn = *n;
		struct ubifs_znode *znode = *zn;

		/* Look right */
		while (1) {
			err = tnc_next(c, &znode, &nn);
			if (err == -ENOENT)
				return 0;
			if (err < 0)
				return err;
			if (keys_cmp(c, &znode->zbranch[nn].key, key))
				return 0;
			err = matches_name(c, &znode->zbranch[nn], nm);
			if (err < 0)
				return err;
			if (err == NAME_GREATER)
				return 0;
			*zn = znode;
			*n = nn;
			if (err == NAME_MATCHES)
				return 1;
			ubifs_assert(err == NAME_LESS);
		}
	}
}

static int fallible_matches_name(struct ubifs_info *c,
				 struct ubifs_zbranch *zbr,
				 const struct qstr *nm)
{
	struct ubifs_dent_node *dent;
	int nlen, err;

	/* If possible, match against the dent in the leaf node cache */
	if (!zbr->leaf) {
		dent = kmalloc(zbr->len, GFP_NOFS);
		if (!dent)
			return -ENOMEM;

		err = fallible_read_node(c, &zbr->key, zbr, dent);
		if (err < 0)
			goto out_free;
		if (err == 0) {
			/* The node was not present */
			err = NOT_ON_MEDIA;
			goto out_free;
		}
		ubifs_assert(err == 1);

		err = lnc_add_directly(c, zbr, dent);
		if (err)
			goto out_free;
	} else
		dent = zbr->leaf;

	nlen = le16_to_cpu(dent->nlen);
	err = memcmp(dent->name, nm->name, min_t(int, nlen, nm->len));
	if (err == 0) {
		if (nlen == nm->len)
			return NAME_MATCHES;
		else if (nlen < nm->len)
			return NAME_LESS;
		else
			return NAME_GREATER;
	} else if (err < 0)
		return NAME_LESS;
	else
		return NAME_GREATER;

out_free:
	kfree(dent);
	return err;
}

static int fallible_resolve_collision(struct ubifs_info *c,
				      const union ubifs_key *key,
				      struct ubifs_znode **zn, int *n,
				      const struct qstr *nm, int adding)
{
	struct ubifs_znode *o_znode = NULL, *znode = *zn;
	int uninitialized_var(o_n), err, cmp, unsure = 0, nn = *n;

	cmp = fallible_matches_name(c, &znode->zbranch[nn], nm);
	if (unlikely(cmp < 0))
		return cmp;
	if (cmp == NAME_MATCHES)
		return 1;
	if (cmp == NOT_ON_MEDIA) {
		o_znode = znode;
		o_n = nn;
		/*
		 * We are unlucky and hit a dangling branch straight away.
		 * Now we do not really know where to go to find the needed
		 * branch - to the left or to the right. Well, let's try left.
		 */
		unsure = 1;
	} else if (!adding)
		unsure = 1; /* Remove a dangling branch wherever it is */

	if (cmp == NAME_GREATER || unsure) {
		/* Look left */
		while (1) {
			err = tnc_prev(c, zn, n);
			if (err == -ENOENT) {
				ubifs_assert(*n == 0);
				*n = -1;
				break;
			}
			if (err < 0)
				return err;
			if (keys_cmp(c, &(*zn)->zbranch[*n].key, key)) {
				/* See comments in 'resolve_collision()' */
				if (*n == (*zn)->child_cnt - 1) {
					err = tnc_next(c, zn, n);
					if (err) {
						/* Should be impossible */
						ubifs_assert(0);
						if (err == -ENOENT)
							err = -EINVAL;
						return err;
					}
					ubifs_assert(*n == 0);
					*n = -1;
				}
				break;
			}
			err = fallible_matches_name(c, &(*zn)->zbranch[*n], nm);
			if (err < 0)
				return err;
			if (err == NAME_MATCHES)
				return 1;
			if (err == NOT_ON_MEDIA) {
				o_znode = *zn;
				o_n = *n;
				continue;
			}
			if (!adding)
				continue;
			if (err == NAME_LESS)
				break;
			else
				unsure = 0;
		}
	}

	if (cmp == NAME_LESS || unsure) {
		/* Look right */
		*zn = znode;
		*n = nn;
		while (1) {
			err = tnc_next(c, &znode, &nn);
			if (err == -ENOENT)
				break;
			if (err < 0)
				return err;
			if (keys_cmp(c, &znode->zbranch[nn].key, key))
				break;
			err = fallible_matches_name(c, &znode->zbranch[nn], nm);
			if (err < 0)
				return err;
			if (err == NAME_GREATER)
				break;
			*zn = znode;
			*n = nn;
			if (err == NAME_MATCHES)
				return 1;
			if (err == NOT_ON_MEDIA) {
				o_znode = znode;
				o_n = nn;
			}
		}
	}

	/* Never match a dangling branch when adding */
	if (adding || !o_znode)
		return 0;

	dbg_mnt("dangling match LEB %d:%d len %d %s",
		o_znode->zbranch[o_n].lnum, o_znode->zbranch[o_n].offs,
		o_znode->zbranch[o_n].len, DBGKEY(key));
	*zn = o_znode;
	*n = o_n;
	return 1;
}

static int matches_position(struct ubifs_zbranch *zbr, int lnum, int offs)
{
	if (zbr->lnum == lnum && zbr->offs == offs)
		return 1;
	else
		return 0;
}

static int resolve_collision_directly(struct ubifs_info *c,
				      const union ubifs_key *key,
				      struct ubifs_znode **zn, int *n,
				      int lnum, int offs)
{
	struct ubifs_znode *znode;
	int nn, err;

	znode = *zn;
	nn = *n;
	if (matches_position(&znode->zbranch[nn], lnum, offs))
		return 1;

	/* Look left */
	while (1) {
		err = tnc_prev(c, &znode, &nn);
		if (err == -ENOENT)
			break;
		if (err < 0)
			return err;
		if (keys_cmp(c, &znode->zbranch[nn].key, key))
			break;
		if (matches_position(&znode->zbranch[nn], lnum, offs)) {
			*zn = znode;
			*n = nn;
			return 1;
		}
	}

	/* Look right */
	znode = *zn;
	nn = *n;
	while (1) {
		err = tnc_next(c, &znode, &nn);
		if (err == -ENOENT)
			return 0;
		if (err < 0)
			return err;
		if (keys_cmp(c, &znode->zbranch[nn].key, key))
			return 0;
		*zn = znode;
		*n = nn;
		if (matches_position(&znode->zbranch[nn], lnum, offs))
			return 1;
	}
}

static struct ubifs_znode *dirty_cow_bottom_up(struct ubifs_info *c,
					       struct ubifs_znode *znode)
{
	struct ubifs_znode *zp;
	int *path = c->bottom_up_buf, p = 0;

	ubifs_assert(c->zroot.znode);
	ubifs_assert(znode);
	if (c->zroot.znode->level > BOTTOM_UP_HEIGHT) {
		kfree(c->bottom_up_buf);
		c->bottom_up_buf = kmalloc(c->zroot.znode->level * sizeof(int),
					   GFP_NOFS);
		if (!c->bottom_up_buf)
			return ERR_PTR(-ENOMEM);
		path = c->bottom_up_buf;
	}
	if (c->zroot.znode->level) {
		/* Go up until parent is dirty */
		while (1) {
			int n;

			zp = znode->parent;
			if (!zp)
				break;
			n = znode->iip;
			ubifs_assert(p < c->zroot.znode->level);
			path[p++] = n;
			if (!zp->cnext && ubifs_zn_dirty(znode))
				break;
			znode = zp;
		}
	}

	/* Come back down, dirtying as we go */
	while (1) {
		struct ubifs_zbranch *zbr;

		zp = znode->parent;
		if (zp) {
			ubifs_assert(path[p - 1] >= 0);
			ubifs_assert(path[p - 1] < zp->child_cnt);
			zbr = &zp->zbranch[path[--p]];
			znode = dirty_cow_znode(c, zbr);
		} else {
			ubifs_assert(znode == c->zroot.znode);
			znode = dirty_cow_znode(c, &c->zroot);
		}
		if (IS_ERR(znode) || !p)
			break;
		ubifs_assert(path[p - 1] >= 0);
		ubifs_assert(path[p - 1] < znode->child_cnt);
		znode = znode->zbranch[path[p - 1]].znode;
	}

	return znode;
}

int ubifs_lookup_level0(struct ubifs_info *c, const union ubifs_key *key,
			struct ubifs_znode **zn, int *n)
{
	int err, exact;
	struct ubifs_znode *znode;
	unsigned long time = get_seconds();

	dbg_tnc("search key %s", DBGKEY(key));

	znode = c->zroot.znode;
	if (unlikely(!znode)) {
		znode = ubifs_load_znode(c, &c->zroot, NULL, 0);
		if (IS_ERR(znode))
			return PTR_ERR(znode);
	}

	znode->time = time;

	while (1) {
		struct ubifs_zbranch *zbr;

		exact = ubifs_search_zbranch(c, znode, key, n);

		if (znode->level == 0)
			break;

		if (*n < 0)
			*n = 0;
		zbr = &znode->zbranch[*n];

		if (zbr->znode) {
			znode->time = time;
			znode = zbr->znode;
			continue;
		}

		/* znode is not in TNC cache, load it from the media */
		znode = ubifs_load_znode(c, zbr, znode, *n);
		if (IS_ERR(znode))
			return PTR_ERR(znode);
	}

	*zn = znode;
	if (exact || !is_hash_key(c, key) || *n != -1) {
		dbg_tnc("found %d, lvl %d, n %d", exact, znode->level, *n);
		return exact;
	}

	/*
	 * Here is a tricky place. We have not found the key and this is a
	 * "hashed" key, which may collide. The rest of the code deals with
	 * situations like this:
	 *
	 *                  | 3 | 5 |
	 *                  /       \
	 *          | 3 | 5 |      | 6 | 7 | (x)
	 *
	 * Or more a complex example:
	 *
	 *                | 1 | 5 |
	 *                /       \
	 *       | 1 | 3 |         | 5 | 8 |
	 *              \           /
	 *          | 5 | 5 |   | 6 | 7 | (x)
	 *
	 * In the examples, if we are looking for key "5", we may reach nodes
	 * marked with "(x)". In this case what we have do is to look at the
	 * left and see if there is "5" key there. If there is, we have to
	 * return it.
	 *
	 * Note, this whole situation is possible because we allow to have
	 * elements which are equivalent to the next key in the parent in the
	 * children of current znode. For example, this happens if we split a
	 * znode like this: | 3 | 5 | 5 | 6 | 7 |, which results in something
	 * like this:
	 *                      | 3 | 5 |
	 *                       /     \
	 *                | 3 | 5 |   | 5 | 6 | 7 |
	 *                              ^
	 * And this becomes what is at the first "picture" after key "5" marked
	 * with "^" is removed. What could be done is we could prohibit
	 * splitting in the middle of the colliding sequence. Also, when
	 * removing the leftmost key, we would have to correct the key of the
	 * parent node, which would introduce additional complications. Namely,
	 * if we changed the leftmost key of the parent znode, the garbage
	 * collector would be unable to find it (GC is doing this when GC'ing
	 * indexing LEBs). Although we already have an additional RB-tree where
	 * we save such changed znodes (see 'ins_clr_old_idx_znode()') until
	 * after the commit. But anyway, this does not look easy to implement
	 * so we did not try this.
	 */
	err = tnc_prev(c, &znode, n);
	if (err == -ENOENT) {
		dbg_tnc("found 0, lvl %d, n -1", znode->level);
		*n = -1;
		return 0;
	}
	if (unlikely(err < 0))
		return err;
	if (keys_cmp(c, key, &znode->zbranch[*n].key)) {
		dbg_tnc("found 0, lvl %d, n -1", znode->level);
		*n = -1;
		return 0;
	}

	dbg_tnc("found 1, lvl %d, n %d", znode->level, *n);
	*zn = znode;
	return 1;
}

static int lookup_level0_dirty(struct ubifs_info *c, const union ubifs_key *key,
			       struct ubifs_znode **zn, int *n)
{
	int err, exact;
	struct ubifs_znode *znode;
	unsigned long time = get_seconds();

	dbg_tnc("search and dirty key %s", DBGKEY(key));

	znode = c->zroot.znode;
	if (unlikely(!znode)) {
		znode = ubifs_load_znode(c, &c->zroot, NULL, 0);
		if (IS_ERR(znode))
			return PTR_ERR(znode);
	}

	znode = dirty_cow_znode(c, &c->zroot);
	if (IS_ERR(znode))
		return PTR_ERR(znode);

	znode->time = time;

	while (1) {
		struct ubifs_zbranch *zbr;

		exact = ubifs_search_zbranch(c, znode, key, n);

		if (znode->level == 0)
			break;

		if (*n < 0)
			*n = 0;
		zbr = &znode->zbranch[*n];

		if (zbr->znode) {
			znode->time = time;
			znode = dirty_cow_znode(c, zbr);
			if (IS_ERR(znode))
				return PTR_ERR(znode);
			continue;
		}

		/* znode is not in TNC cache, load it from the media */
		znode = ubifs_load_znode(c, zbr, znode, *n);
		if (IS_ERR(znode))
			return PTR_ERR(znode);
		znode = dirty_cow_znode(c, zbr);
		if (IS_ERR(znode))
			return PTR_ERR(znode);
	}

	*zn = znode;
	if (exact || !is_hash_key(c, key) || *n != -1) {
		dbg_tnc("found %d, lvl %d, n %d", exact, znode->level, *n);
		return exact;
	}

	/*
	 * See huge comment at 'lookup_level0_dirty()' what is the rest of the
	 * code.
	 */
	err = tnc_prev(c, &znode, n);
	if (err == -ENOENT) {
		*n = -1;
		dbg_tnc("found 0, lvl %d, n -1", znode->level);
		return 0;
	}
	if (unlikely(err < 0))
		return err;
	if (keys_cmp(c, key, &znode->zbranch[*n].key)) {
		*n = -1;
		dbg_tnc("found 0, lvl %d, n -1", znode->level);
		return 0;
	}

	if (znode->cnext || !ubifs_zn_dirty(znode)) {
		znode = dirty_cow_bottom_up(c, znode);
		if (IS_ERR(znode))
			return PTR_ERR(znode);
	}

	dbg_tnc("found 1, lvl %d, n %d", znode->level, *n);
	*zn = znode;
	return 1;
}

static int maybe_leb_gced(struct ubifs_info *c, int lnum, int gc_seq1)
{
	int gc_seq2, gced_lnum;

	gced_lnum = c->gced_lnum;
	smp_rmb();
	gc_seq2 = c->gc_seq;
	/* Same seq means no GC */
	if (gc_seq1 == gc_seq2)
		return 0;
	/* Different by more than 1 means we don't know */
	if (gc_seq1 + 1 != gc_seq2)
		return 1;
	/*
	 * We have seen the sequence number has increased by 1. Now we need to
	 * be sure we read the right LEB number, so read it again.
	 */
	smp_rmb();
	if (gced_lnum != c->gced_lnum)
		return 1;
	/* Finally we can check lnum */
	if (gced_lnum == lnum)
		return 1;
	return 0;
}

int ubifs_tnc_locate(struct ubifs_info *c, const union ubifs_key *key,
		     void *node, int *lnum, int *offs)
{
	int found, n, err, safely = 0, gc_seq1;
	struct ubifs_znode *znode;
	struct ubifs_zbranch zbr, *zt;

again:
	mutex_lock(&c->tnc_mutex);
	found = ubifs_lookup_level0(c, key, &znode, &n);
	if (!found) {
		err = -ENOENT;
		goto out;
	} else if (found < 0) {
		err = found;
		goto out;
	}
	zt = &znode->zbranch[n];
	if (lnum) {
		*lnum = zt->lnum;
		*offs = zt->offs;
	}
	if (is_hash_key(c, key)) {
		/*
		 * In this case the leaf node cache gets used, so we pass the
		 * address of the zbranch and keep the mutex locked
		 */
		err = tnc_read_node_nm(c, zt, node);
		goto out;
	}
	if (safely) {
		err = ubifs_tnc_read_node(c, zt, node);
		goto out;
	}
	/* Drop the TNC mutex prematurely and race with garbage collection */
	zbr = znode->zbranch[n];
	gc_seq1 = c->gc_seq;
	mutex_unlock(&c->tnc_mutex);

	if (ubifs_get_wbuf(c, zbr.lnum)) {
		/* We do not GC journal heads */
		err = ubifs_tnc_read_node(c, &zbr, node);
		return err;
	}

	err = fallible_read_node(c, key, &zbr, node);
	if (err <= 0 || maybe_leb_gced(c, zbr.lnum, gc_seq1)) {
		/*
		 * The node may have been GC'ed out from under us so try again
		 * while keeping the TNC mutex locked.
		 */
		safely = 1;
		goto again;
	}
	return 0;

out:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

int ubifs_tnc_get_bu_keys(struct ubifs_info *c, struct bu_info *bu)
{
	int n, err = 0, lnum = -1, uninitialized_var(offs);
	int uninitialized_var(len);
	unsigned int block = key_block(c, &bu->key);
	struct ubifs_znode *znode;

	bu->cnt = 0;
	bu->blk_cnt = 0;
	bu->eof = 0;

	mutex_lock(&c->tnc_mutex);
	/* Find first key */
	err = ubifs_lookup_level0(c, &bu->key, &znode, &n);
	if (err < 0)
		goto out;
	if (err) {
		/* Key found */
		len = znode->zbranch[n].len;
		/* The buffer must be big enough for at least 1 node */
		if (len > bu->buf_len) {
			err = -EINVAL;
			goto out;
		}
		/* Add this key */
		bu->zbranch[bu->cnt++] = znode->zbranch[n];
		bu->blk_cnt += 1;
		lnum = znode->zbranch[n].lnum;
		offs = ALIGN(znode->zbranch[n].offs + len, 8);
	}
	while (1) {
		struct ubifs_zbranch *zbr;
		union ubifs_key *key;
		unsigned int next_block;

		/* Find next key */
		err = tnc_next(c, &znode, &n);
		if (err)
			goto out;
		zbr = &znode->zbranch[n];
		key = &zbr->key;
		/* See if there is another data key for this file */
		if (key_inum(c, key) != key_inum(c, &bu->key) ||
		    key_type(c, key) != UBIFS_DATA_KEY) {
			err = -ENOENT;
			goto out;
		}
		if (lnum < 0) {
			/* First key found */
			lnum = zbr->lnum;
			offs = ALIGN(zbr->offs + zbr->len, 8);
			len = zbr->len;
			if (len > bu->buf_len) {
				err = -EINVAL;
				goto out;
			}
		} else {
			/*
			 * The data nodes must be in consecutive positions in
			 * the same LEB.
			 */
			if (zbr->lnum != lnum || zbr->offs != offs)
				goto out;
			offs += ALIGN(zbr->len, 8);
			len = ALIGN(len, 8) + zbr->len;
			/* Must not exceed buffer length */
			if (len > bu->buf_len)
				goto out;
		}
		/* Allow for holes */
		next_block = key_block(c, key);
		bu->blk_cnt += (next_block - block - 1);
		if (bu->blk_cnt >= UBIFS_MAX_BULK_READ)
			goto out;
		block = next_block;
		/* Add this key */
		bu->zbranch[bu->cnt++] = *zbr;
		bu->blk_cnt += 1;
		/* See if we have room for more */
		if (bu->cnt >= UBIFS_MAX_BULK_READ)
			goto out;
		if (bu->blk_cnt >= UBIFS_MAX_BULK_READ)
			goto out;
	}
out:
	if (err == -ENOENT) {
		bu->eof = 1;
		err = 0;
	}
	bu->gc_seq = c->gc_seq;
	mutex_unlock(&c->tnc_mutex);
	if (err)
		return err;
	/*
	 * An enormous hole could cause bulk-read to encompass too many
	 * page cache pages, so limit the number here.
	 */
	if (bu->blk_cnt > UBIFS_MAX_BULK_READ)
		bu->blk_cnt = UBIFS_MAX_BULK_READ;
	/*
	 * Ensure that bulk-read covers a whole number of page cache
	 * pages.
	 */
	if (UBIFS_BLOCKS_PER_PAGE == 1 ||
	    !(bu->blk_cnt & (UBIFS_BLOCKS_PER_PAGE - 1)))
		return 0;
	if (bu->eof) {
		/* At the end of file we can round up */
		bu->blk_cnt += UBIFS_BLOCKS_PER_PAGE - 1;
		return 0;
	}
	/* Exclude data nodes that do not make up a whole page cache page */
	block = key_block(c, &bu->key) + bu->blk_cnt;
	block &= ~(UBIFS_BLOCKS_PER_PAGE - 1);
	while (bu->cnt) {
		if (key_block(c, &bu->zbranch[bu->cnt - 1].key) < block)
			break;
		bu->cnt -= 1;
	}
	return 0;
}

static int read_wbuf(struct ubifs_wbuf *wbuf, void *buf, int len, int lnum,
		     int offs)
{
	const struct ubifs_info *c = wbuf->c;
	int rlen, overlap;

	dbg_io("LEB %d:%d, length %d", lnum, offs, len);
	ubifs_assert(wbuf && lnum >= 0 && lnum < c->leb_cnt && offs >= 0);
	ubifs_assert(!(offs & 7) && offs < c->leb_size);
	ubifs_assert(offs + len <= c->leb_size);

	spin_lock(&wbuf->lock);
	overlap = (lnum == wbuf->lnum && offs + len > wbuf->offs);
	if (!overlap) {
		/* We may safely unlock the write-buffer and read the data */
		spin_unlock(&wbuf->lock);
		return ubi_read(c->ubi, lnum, buf, offs, len);
	}

	/* Don't read under wbuf */
	rlen = wbuf->offs - offs;
	if (rlen < 0)
		rlen = 0;

	/* Copy the rest from the write-buffer */
	memcpy(buf + rlen, wbuf->buf + offs + rlen - wbuf->offs, len - rlen);
	spin_unlock(&wbuf->lock);

	if (rlen > 0)
		/* Read everything that goes before write-buffer */
		return ubi_read(c->ubi, lnum, buf, offs, rlen);

	return 0;
}

static int validate_data_node(struct ubifs_info *c, void *buf,
			      struct ubifs_zbranch *zbr)
{
	union ubifs_key key1;
	struct ubifs_ch *ch = buf;
	int err, len;

	if (ch->node_type != UBIFS_DATA_NODE) {
		ubifs_err("bad node type (%d but expected %d)",
			  ch->node_type, UBIFS_DATA_NODE);
		goto out_err;
	}

	err = ubifs_check_node(c, buf, zbr->lnum, zbr->offs, 0, 0);
	if (err) {
		ubifs_err("expected node type %d", UBIFS_DATA_NODE);
		goto out;
	}

	len = le32_to_cpu(ch->len);
	if (len != zbr->len) {
		ubifs_err("bad node length %d, expected %d", len, zbr->len);
		goto out_err;
	}

	/* Make sure the key of the read node is correct */
	key_read(c, buf + UBIFS_KEY_OFFSET, &key1);
	if (!keys_eq(c, &zbr->key, &key1)) {
		ubifs_err("bad key in node at LEB %d:%d",
			  zbr->lnum, zbr->offs);
		dbg_tnc("looked for key %s found node's key %s",
			DBGKEY(&zbr->key), DBGKEY1(&key1));
		goto out_err;
	}

	return 0;

out_err:
	err = -EINVAL;
out:
	ubifs_err("bad node at LEB %d:%d", zbr->lnum, zbr->offs);
	dbg_dump_node(c, buf);
	dbg_dump_stack();
	return err;
}

int ubifs_tnc_bulk_read(struct ubifs_info *c, struct bu_info *bu)
{
	int lnum = bu->zbranch[0].lnum, offs = bu->zbranch[0].offs, len, err, i;
	struct ubifs_wbuf *wbuf;
	void *buf;

	len = bu->zbranch[bu->cnt - 1].offs;
	len += bu->zbranch[bu->cnt - 1].len - offs;
	if (len > bu->buf_len) {
		ubifs_err("buffer too small %d vs %d", bu->buf_len, len);
		return -EINVAL;
	}

	/* Do the read */
	wbuf = ubifs_get_wbuf(c, lnum);
	if (wbuf)
		err = read_wbuf(wbuf, bu->buf, len, lnum, offs);
	else
		err = ubi_read(c->ubi, lnum, bu->buf, offs, len);

	/* Check for a race with GC */
	if (maybe_leb_gced(c, lnum, bu->gc_seq))
		return -EAGAIN;

	if (err && err != -EBADMSG) {
		ubifs_err("failed to read from LEB %d:%d, error %d",
			  lnum, offs, err);
		dbg_dump_stack();
		dbg_tnc("key %s", DBGKEY(&bu->key));
		return err;
	}

	/* Validate the nodes read */
	buf = bu->buf;
	for (i = 0; i < bu->cnt; i++) {
		err = validate_data_node(c, buf, &bu->zbranch[i]);
		if (err)
			return err;
		buf = buf + ALIGN(bu->zbranch[i].len, 8);
	}

	return 0;
}

static int do_lookup_nm(struct ubifs_info *c, const union ubifs_key *key,
			void *node, const struct qstr *nm)
{
	int found, n, err;
	struct ubifs_znode *znode;

	dbg_tnc("name '%.*s' key %s", nm->len, nm->name, DBGKEY(key));
	mutex_lock(&c->tnc_mutex);
	found = ubifs_lookup_level0(c, key, &znode, &n);
	if (!found) {
		err = -ENOENT;
		goto out_unlock;
	} else if (found < 0) {
		err = found;
		goto out_unlock;
	}

	ubifs_assert(n >= 0);

	err = resolve_collision(c, key, &znode, &n, nm);
	dbg_tnc("rc returned %d, znode %p, n %d", err, znode, n);
	if (unlikely(err < 0))
		goto out_unlock;
	if (err == 0) {
		err = -ENOENT;
		goto out_unlock;
	}

	err = tnc_read_node_nm(c, &znode->zbranch[n], node);

out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

int ubifs_tnc_lookup_nm(struct ubifs_info *c, const union ubifs_key *key,
			void *node, const struct qstr *nm)
{
	int err, len;
	const struct ubifs_dent_node *dent = node;

	/*
	 * We assume that in most of the cases there are no name collisions and
	 * 'ubifs_tnc_lookup()' returns us the right direntry.
	 */
	err = ubifs_tnc_lookup(c, key, node);
	if (err)
		return err;

	len = le16_to_cpu(dent->nlen);
	if (nm->len == len && !memcmp(dent->name, nm->name, len))
		return 0;

	/*
	 * Unluckily, there are hash collisions and we have to iterate over
	 * them look at each direntry with colliding name hash sequentially.
	 */
	return do_lookup_nm(c, key, node, nm);
}

static void correct_parent_keys(const struct ubifs_info *c,
				struct ubifs_znode *znode)
{
	union ubifs_key *key, *key1;

	ubifs_assert(znode->parent);
	ubifs_assert(znode->iip == 0);

	key = &znode->zbranch[0].key;
	key1 = &znode->parent->zbranch[0].key;

	while (keys_cmp(c, key, key1) < 0) {
		key_copy(c, key, key1);
		znode = znode->parent;
		znode->alt = 1;
		if (!znode->parent || znode->iip)
			break;
		key1 = &znode->parent->zbranch[0].key;
	}
}

static void insert_zbranch(struct ubifs_znode *znode,
			   const struct ubifs_zbranch *zbr, int n)
{
	int i;

	ubifs_assert(ubifs_zn_dirty(znode));

	if (znode->level) {
		for (i = znode->child_cnt; i > n; i--) {
			znode->zbranch[i] = znode->zbranch[i - 1];
			if (znode->zbranch[i].znode)
				znode->zbranch[i].znode->iip = i;
		}
		if (zbr->znode)
			zbr->znode->iip = n;
	} else
		for (i = znode->child_cnt; i > n; i--)
			znode->zbranch[i] = znode->zbranch[i - 1];

	znode->zbranch[n] = *zbr;
	znode->child_cnt += 1;

	/*
	 * After inserting at slot zero, the lower bound of the key range of
	 * this znode may have changed. If this znode is subsequently split
	 * then the upper bound of the key range may change, and furthermore
	 * it could change to be lower than the original lower bound. If that
	 * happens, then it will no longer be possible to find this znode in the
	 * TNC using the key from the index node on flash. That is bad because
	 * if it is not found, we will assume it is obsolete and may overwrite
	 * it. Then if there is an unclean unmount, we will start using the
	 * old index which will be broken.
	 *
	 * So we first mark znodes that have insertions at slot zero, and then
	 * if they are split we add their lnum/offs to the old_idx tree.
	 */
	if (n == 0)
		znode->alt = 1;
}

static int tnc_insert(struct ubifs_info *c, struct ubifs_znode *znode,
		      struct ubifs_zbranch *zbr, int n)
{
	struct ubifs_znode *zn, *zi, *zp;
	int i, keep, move, appending = 0;
	union ubifs_key *key = &zbr->key, *key1;

	ubifs_assert(n >= 0 && n <= c->fanout);

	/* Implement naive insert for now */
again:
	zp = znode->parent;
	if (znode->child_cnt < c->fanout) {
		ubifs_assert(n != c->fanout);
		dbg_tnc("inserted at %d level %d, key %s", n, znode->level,
			DBGKEY(key));

		insert_zbranch(znode, zbr, n);

		/* Ensure parent's key is correct */
		if (n == 0 && zp && znode->iip == 0)
			correct_parent_keys(c, znode);

		return 0;
	}

	/*
	 * Unfortunately, @znode does not have more empty slots and we have to
	 * split it.
	 */
	dbg_tnc("splitting level %d, key %s", znode->level, DBGKEY(key));

	if (znode->alt)
		/*
		 * We can no longer be sure of finding this znode by key, so we
		 * record it in the old_idx tree.
		 */
		ins_clr_old_idx_znode(c, znode);

	zn = kzalloc(c->max_znode_sz, GFP_NOFS);
	if (!zn)
		return -ENOMEM;
	zn->parent = zp;
	zn->level = znode->level;

	/* Decide where to split */
	if (znode->level == 0 && key_type(c, key) == UBIFS_DATA_KEY) {
		/* Try not to split consecutive data keys */
		if (n == c->fanout) {
			key1 = &znode->zbranch[n - 1].key;
			if (key_inum(c, key1) == key_inum(c, key) &&
			    key_type(c, key1) == UBIFS_DATA_KEY)
				appending = 1;
		} else
			goto check_split;
	} else if (appending && n != c->fanout) {
		/* Try not to split consecutive data keys */
		appending = 0;
check_split:
		if (n >= (c->fanout + 1) / 2) {
			key1 = &znode->zbranch[0].key;
			if (key_inum(c, key1) == key_inum(c, key) &&
			    key_type(c, key1) == UBIFS_DATA_KEY) {
				key1 = &znode->zbranch[n].key;
				if (key_inum(c, key1) != key_inum(c, key) ||
				    key_type(c, key1) != UBIFS_DATA_KEY) {
					keep = n;
					move = c->fanout - keep;
					zi = znode;
					goto do_split;
				}
			}
		}
	}

	if (appending) {
		keep = c->fanout;
		move = 0;
	} else {
		keep = (c->fanout + 1) / 2;
		move = c->fanout - keep;
	}

	/*
	 * Although we don't at present, we could look at the neighbors and see
	 * if we can move some zbranches there.
	 */

	if (n < keep) {
		/* Insert into existing znode */
		zi = znode;
		move += 1;
		keep -= 1;
	} else {
		/* Insert into new znode */
		zi = zn;
		n -= keep;
		/* Re-parent */
		if (zn->level != 0)
			zbr->znode->parent = zn;
	}

do_split:

	__set_bit(DIRTY_ZNODE, &zn->flags);
	atomic_long_inc(&c->dirty_zn_cnt);

	zn->child_cnt = move;
	znode->child_cnt = keep;

	dbg_tnc("moving %d, keeping %d", move, keep);

	/* Move zbranch */
	for (i = 0; i < move; i++) {
		zn->zbranch[i] = znode->zbranch[keep + i];
		/* Re-parent */
		if (zn->level != 0)
			if (zn->zbranch[i].znode) {
				zn->zbranch[i].znode->parent = zn;
				zn->zbranch[i].znode->iip = i;
			}
	}

	/* Insert new key and branch */
	dbg_tnc("inserting at %d level %d, key %s", n, zn->level, DBGKEY(key));

	insert_zbranch(zi, zbr, n);

	/* Insert new znode (produced by spitting) into the parent */
	if (zp) {
		if (n == 0 && zi == znode && znode->iip == 0)
			correct_parent_keys(c, znode);

		/* Locate insertion point */
		n = znode->iip + 1;

		/* Tail recursion */
		zbr->key = zn->zbranch[0].key;
		zbr->znode = zn;
		zbr->lnum = 0;
		zbr->offs = 0;
		zbr->len = 0;
		znode = zp;

		goto again;
	}

	/* We have to split root znode */
	dbg_tnc("creating new zroot at level %d", znode->level + 1);

	zi = kzalloc(c->max_znode_sz, GFP_NOFS);
	if (!zi)
		return -ENOMEM;

	zi->child_cnt = 2;
	zi->level = znode->level + 1;

	__set_bit(DIRTY_ZNODE, &zi->flags);
	atomic_long_inc(&c->dirty_zn_cnt);

	zi->zbranch[0].key = znode->zbranch[0].key;
	zi->zbranch[0].znode = znode;
	zi->zbranch[0].lnum = c->zroot.lnum;
	zi->zbranch[0].offs = c->zroot.offs;
	zi->zbranch[0].len = c->zroot.len;
	zi->zbranch[1].key = zn->zbranch[0].key;
	zi->zbranch[1].znode = zn;

	c->zroot.lnum = 0;
	c->zroot.offs = 0;
	c->zroot.len = 0;
	c->zroot.znode = zi;

	zn->parent = zi;
	zn->iip = 1;
	znode->parent = zi;
	znode->iip = 0;

	return 0;
}

int ubifs_tnc_add(struct ubifs_info *c, const union ubifs_key *key, int lnum,
		  int offs, int len)
{
	int found, n, err = 0;
	struct ubifs_znode *znode;

	mutex_lock(&c->tnc_mutex);
	dbg_tnc("%d:%d, len %d, key %s", lnum, offs, len, DBGKEY(key));
	found = lookup_level0_dirty(c, key, &znode, &n);
	if (!found) {
		struct ubifs_zbranch zbr;

		zbr.znode = NULL;
		zbr.lnum = lnum;
		zbr.offs = offs;
		zbr.len = len;
		key_copy(c, key, &zbr.key);
		err = tnc_insert(c, znode, &zbr, n + 1);
	} else if (found == 1) {
		struct ubifs_zbranch *zbr = &znode->zbranch[n];

		lnc_free(zbr);
		err = ubifs_add_dirt(c, zbr->lnum, zbr->len);
		zbr->lnum = lnum;
		zbr->offs = offs;
		zbr->len = len;
	} else
		err = found;
	if (!err)
		err = dbg_check_tnc(c, 0);
	mutex_unlock(&c->tnc_mutex);

	return err;
}

int ubifs_tnc_replace(struct ubifs_info *c, const union ubifs_key *key,
		      int old_lnum, int old_offs, int lnum, int offs, int len)
{
	int found, n, err = 0;
	struct ubifs_znode *znode;

	mutex_lock(&c->tnc_mutex);
	dbg_tnc("old LEB %d:%d, new LEB %d:%d, len %d, key %s", old_lnum,
		old_offs, lnum, offs, len, DBGKEY(key));
	found = lookup_level0_dirty(c, key, &znode, &n);
	if (found < 0) {
		err = found;
		goto out_unlock;
	}

	if (found == 1) {
		struct ubifs_zbranch *zbr = &znode->zbranch[n];

		found = 0;
		if (zbr->lnum == old_lnum && zbr->offs == old_offs) {
			lnc_free(zbr);
			err = ubifs_add_dirt(c, zbr->lnum, zbr->len);
			if (err)
				goto out_unlock;
			zbr->lnum = lnum;
			zbr->offs = offs;
			zbr->len = len;
			found = 1;
		} else if (is_hash_key(c, key)) {
			found = resolve_collision_directly(c, key, &znode, &n,
							   old_lnum, old_offs);
			dbg_tnc("rc returned %d, znode %p, n %d, LEB %d:%d",
				found, znode, n, old_lnum, old_offs);
			if (found < 0) {
				err = found;
				goto out_unlock;
			}

			if (found) {
				/* Ensure the znode is dirtied */
				if (znode->cnext || !ubifs_zn_dirty(znode)) {
					znode = dirty_cow_bottom_up(c, znode);
					if (IS_ERR(znode)) {
						err = PTR_ERR(znode);
						goto out_unlock;
					}
				}
				zbr = &znode->zbranch[n];
				lnc_free(zbr);
				err = ubifs_add_dirt(c, zbr->lnum,
						     zbr->len);
				if (err)
					goto out_unlock;
				zbr->lnum = lnum;
				zbr->offs = offs;
				zbr->len = len;
			}
		}
	}

	if (!found)
		err = ubifs_add_dirt(c, lnum, len);

	if (!err)
		err = dbg_check_tnc(c, 0);

out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

int ubifs_tnc_add_nm(struct ubifs_info *c, const union ubifs_key *key,
		     int lnum, int offs, int len, const struct qstr *nm)
{
	int found, n, err = 0;
	struct ubifs_znode *znode;

	mutex_lock(&c->tnc_mutex);
	dbg_tnc("LEB %d:%d, name '%.*s', key %s", lnum, offs, nm->len, nm->name,
		DBGKEY(key));
	found = lookup_level0_dirty(c, key, &znode, &n);
	if (found < 0) {
		err = found;
		goto out_unlock;
	}

	if (found == 1) {
		if (c->replaying)
			found = fallible_resolve_collision(c, key, &znode, &n,
							   nm, 1);
		else
			found = resolve_collision(c, key, &znode, &n, nm);
		dbg_tnc("rc returned %d, znode %p, n %d", found, znode, n);
		if (found < 0) {
			err = found;
			goto out_unlock;
		}

		/* Ensure the znode is dirtied */
		if (znode->cnext || !ubifs_zn_dirty(znode)) {
			znode = dirty_cow_bottom_up(c, znode);
			if (IS_ERR(znode)) {
				err = PTR_ERR(znode);
				goto out_unlock;
			}
		}

		if (found == 1) {
			struct ubifs_zbranch *zbr = &znode->zbranch[n];

			lnc_free(zbr);
			err = ubifs_add_dirt(c, zbr->lnum, zbr->len);
			zbr->lnum = lnum;
			zbr->offs = offs;
			zbr->len = len;
			goto out_unlock;
		}
	}

	if (!found) {
		struct ubifs_zbranch zbr;

		zbr.znode = NULL;
		zbr.lnum = lnum;
		zbr.offs = offs;
		zbr.len = len;
		key_copy(c, key, &zbr.key);
		err = tnc_insert(c, znode, &zbr, n + 1);
		if (err)
			goto out_unlock;
		if (c->replaying) {
			/*
			 * We did not find it in the index so there may be a
			 * dangling branch still in the index. So we remove it
			 * by passing 'ubifs_tnc_remove_nm()' the same key but
			 * an unmatchable name.
			 */
			struct qstr noname = { .len = 0, .name = "" };

			err = dbg_check_tnc(c, 0);
			mutex_unlock(&c->tnc_mutex);
			if (err)
				return err;
			return ubifs_tnc_remove_nm(c, key, &noname);
		}
	}

out_unlock:
	if (!err)
		err = dbg_check_tnc(c, 0);
	mutex_unlock(&c->tnc_mutex);
	return err;
}

static int tnc_delete(struct ubifs_info *c, struct ubifs_znode *znode, int n)
{
	struct ubifs_zbranch *zbr;
	struct ubifs_znode *zp;
	int i, err;

	/* Delete without merge for now */
	ubifs_assert(znode->level == 0);
	ubifs_assert(n >= 0 && n < c->fanout);
	dbg_tnc("deleting %s", DBGKEY(&znode->zbranch[n].key));

	zbr = &znode->zbranch[n];
	lnc_free(zbr);

	err = ubifs_add_dirt(c, zbr->lnum, zbr->len);
	if (err) {
		dbg_dump_znode(c, znode);
		return err;
	}

	/* We do not "gap" zbranch slots */
	for (i = n; i < znode->child_cnt - 1; i++)
		znode->zbranch[i] = znode->zbranch[i + 1];
	znode->child_cnt -= 1;

	if (znode->child_cnt > 0)
		return 0;

	/*
	 * This was the last zbranch, we have to delete this znode from the
	 * parent.
	 */

	do {
		ubifs_assert(!test_bit(OBSOLETE_ZNODE, &znode->flags));
		ubifs_assert(ubifs_zn_dirty(znode));

		zp = znode->parent;
		n = znode->iip;

		atomic_long_dec(&c->dirty_zn_cnt);

		err = insert_old_idx_znode(c, znode);
		if (err)
			return err;

		if (znode->cnext) {
			__set_bit(OBSOLETE_ZNODE, &znode->flags);
			atomic_long_inc(&c->clean_zn_cnt);
			atomic_long_inc(&ubifs_clean_zn_cnt);
		} else
			kfree(znode);
		znode = zp;
	} while (znode->child_cnt == 1); /* while removing last child */

	/* Remove from znode, entry n - 1 */
	znode->child_cnt -= 1;
	ubifs_assert(znode->level != 0);
	for (i = n; i < znode->child_cnt; i++) {
		znode->zbranch[i] = znode->zbranch[i + 1];
		if (znode->zbranch[i].znode)
			znode->zbranch[i].znode->iip = i;
	}

	/*
	 * If this is the root and it has only 1 child then
	 * collapse the tree.
	 */
	if (!znode->parent) {
		while (znode->child_cnt == 1 && znode->level != 0) {
			zp = znode;
			zbr = &znode->zbranch[0];
			znode = get_znode(c, znode, 0);
			if (IS_ERR(znode))
				return PTR_ERR(znode);
			znode = dirty_cow_znode(c, zbr);
			if (IS_ERR(znode))
				return PTR_ERR(znode);
			znode->parent = NULL;
			znode->iip = 0;
			if (c->zroot.len) {
				err = insert_old_idx(c, c->zroot.lnum,
						     c->zroot.offs);
				if (err)
					return err;
			}
			c->zroot.lnum = zbr->lnum;
			c->zroot.offs = zbr->offs;
			c->zroot.len = zbr->len;
			c->zroot.znode = znode;
			ubifs_assert(!test_bit(OBSOLETE_ZNODE,
				     &zp->flags));
			ubifs_assert(test_bit(DIRTY_ZNODE, &zp->flags));
			atomic_long_dec(&c->dirty_zn_cnt);

			if (zp->cnext) {
				__set_bit(OBSOLETE_ZNODE, &zp->flags);
				atomic_long_inc(&c->clean_zn_cnt);
				atomic_long_inc(&ubifs_clean_zn_cnt);
			} else
				kfree(zp);
		}
	}

	return 0;
}

int ubifs_tnc_remove(struct ubifs_info *c, const union ubifs_key *key)
{
	int found, n, err = 0;
	struct ubifs_znode *znode;

	mutex_lock(&c->tnc_mutex);
	dbg_tnc("key %s", DBGKEY(key));
	found = lookup_level0_dirty(c, key, &znode, &n);
	if (found < 0) {
		err = found;
		goto out_unlock;
	}
	if (found == 1)
		err = tnc_delete(c, znode, n);
	if (!err)
		err = dbg_check_tnc(c, 0);

out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

int ubifs_tnc_remove_nm(struct ubifs_info *c, const union ubifs_key *key,
			const struct qstr *nm)
{
	int n, err;
	struct ubifs_znode *znode;

	mutex_lock(&c->tnc_mutex);
	dbg_tnc("%.*s, key %s", nm->len, nm->name, DBGKEY(key));
	err = lookup_level0_dirty(c, key, &znode, &n);
	if (err < 0)
		goto out_unlock;

	if (err) {
		if (c->replaying)
			err = fallible_resolve_collision(c, key, &znode, &n,
							 nm, 0);
		else
			err = resolve_collision(c, key, &znode, &n, nm);
		dbg_tnc("rc returned %d, znode %p, n %d", err, znode, n);
		if (err < 0)
			goto out_unlock;
		if (err) {
			/* Ensure the znode is dirtied */
			if (znode->cnext || !ubifs_zn_dirty(znode)) {
				    znode = dirty_cow_bottom_up(c, znode);
				    if (IS_ERR(znode)) {
					    err = PTR_ERR(znode);
					    goto out_unlock;
				    }
			}
			err = tnc_delete(c, znode, n);
		}
	}

out_unlock:
	if (!err)
		err = dbg_check_tnc(c, 0);
	mutex_unlock(&c->tnc_mutex);
	return err;
}

static int key_in_range(struct ubifs_info *c, union ubifs_key *key,
			union ubifs_key *from_key, union ubifs_key *to_key)
{
	if (keys_cmp(c, key, from_key) < 0)
		return 0;
	if (keys_cmp(c, key, to_key) > 0)
		return 0;
	return 1;
}

int ubifs_tnc_remove_range(struct ubifs_info *c, union ubifs_key *from_key,
			   union ubifs_key *to_key)
{
	int i, n, k, err = 0;
	struct ubifs_znode *znode;
	union ubifs_key *key;

	mutex_lock(&c->tnc_mutex);
	while (1) {
		/* Find first level 0 znode that contains keys to remove */
		err = ubifs_lookup_level0(c, from_key, &znode, &n);
		if (err < 0)
			goto out_unlock;

		if (err)
			key = from_key;
		else {
			err = tnc_next(c, &znode, &n);
			if (err == -ENOENT) {
				err = 0;
				goto out_unlock;
			}
			if (err < 0)
				goto out_unlock;
			key = &znode->zbranch[n].key;
			if (!key_in_range(c, key, from_key, to_key)) {
				err = 0;
				goto out_unlock;
			}
		}

		/* Ensure the znode is dirtied */
		if (znode->cnext || !ubifs_zn_dirty(znode)) {
			znode = dirty_cow_bottom_up(c, znode);
			if (IS_ERR(znode)) {
				err = PTR_ERR(znode);
				goto out_unlock;
			}
		}

		/* Remove all keys in range except the first */
		for (i = n + 1, k = 0; i < znode->child_cnt; i++, k++) {
			key = &znode->zbranch[i].key;
			if (!key_in_range(c, key, from_key, to_key))
				break;
			lnc_free(&znode->zbranch[i]);
			err = ubifs_add_dirt(c, znode->zbranch[i].lnum,
					     znode->zbranch[i].len);
			if (err) {
				dbg_dump_znode(c, znode);
				goto out_unlock;
			}
			dbg_tnc("removing %s", DBGKEY(key));
		}
		if (k) {
			for (i = n + 1 + k; i < znode->child_cnt; i++)
				znode->zbranch[i - k] = znode->zbranch[i];
			znode->child_cnt -= k;
		}

		/* Now delete the first */
		err = tnc_delete(c, znode, n);
		if (err)
			goto out_unlock;
	}

out_unlock:
	if (!err)
		err = dbg_check_tnc(c, 0);
	mutex_unlock(&c->tnc_mutex);
	return err;
}

int ubifs_tnc_remove_ino(struct ubifs_info *c, ino_t inum)
{
	union ubifs_key key1, key2;
	struct ubifs_dent_node *xent, *pxent = NULL;
	struct qstr nm = { .name = NULL };

	dbg_tnc("ino %lu", (unsigned long)inum);

	/*
	 * Walk all extended attribute entries and remove them together with
	 * corresponding extended attribute inodes.
	 */
	lowest_xent_key(c, &key1, inum);
	while (1) {
		ino_t xattr_inum;
		int err;

		xent = ubifs_tnc_next_ent(c, &key1, &nm);
		if (IS_ERR(xent)) {
			err = PTR_ERR(xent);
			if (err == -ENOENT)
				break;
			return err;
		}

		xattr_inum = le64_to_cpu(xent->inum);
		dbg_tnc("xent '%s', ino %lu", xent->name,
			(unsigned long)xattr_inum);

		nm.name = xent->name;
		nm.len = le16_to_cpu(xent->nlen);
		err = ubifs_tnc_remove_nm(c, &key1, &nm);
		if (err) {
			kfree(xent);
			return err;
		}

		lowest_ino_key(c, &key1, xattr_inum);
		highest_ino_key(c, &key2, xattr_inum);
		err = ubifs_tnc_remove_range(c, &key1, &key2);
		if (err) {
			kfree(xent);
			return err;
		}

		kfree(pxent);
		pxent = xent;
		key_read(c, &xent->key, &key1);
	}

	kfree(pxent);
	lowest_ino_key(c, &key1, inum);
	highest_ino_key(c, &key2, inum);

	return ubifs_tnc_remove_range(c, &key1, &key2);
}

struct ubifs_dent_node *ubifs_tnc_next_ent(struct ubifs_info *c,
					   union ubifs_key *key,
					   const struct qstr *nm)
{
	int n, err, type = key_type(c, key);
	struct ubifs_znode *znode;
	struct ubifs_dent_node *dent;
	struct ubifs_zbranch *zbr;
	union ubifs_key *dkey;

	dbg_tnc("%s %s", nm->name ? (char *)nm->name : "(lowest)", DBGKEY(key));
	ubifs_assert(is_hash_key(c, key));

	mutex_lock(&c->tnc_mutex);
	err = ubifs_lookup_level0(c, key, &znode, &n);
	if (unlikely(err < 0))
		goto out_unlock;

	if (nm->name) {
		if (err) {
			/* Handle collisions */
			err = resolve_collision(c, key, &znode, &n, nm);
			dbg_tnc("rc returned %d, znode %p, n %d",
				err, znode, n);
			if (unlikely(err < 0))
				goto out_unlock;
		}

		/* Now find next entry */
		err = tnc_next(c, &znode, &n);
		if (unlikely(err))
			goto out_unlock;
	} else {
		/*
		 * The full name of the entry was not given, in which case the
		 * behavior of this function is a little different and it
		 * returns current entry, not the next one.
		 */
		if (!err) {
			/*
			 * However, the given key does not exist in the TNC
			 * tree and @znode/@n variables contain the closest
			 * "preceding" element. Switch to the next one.
			 */
			err = tnc_next(c, &znode, &n);
			if (err)
				goto out_unlock;
		}
	}

	zbr = &znode->zbranch[n];
	dent = kmalloc(zbr->len, GFP_NOFS);
	if (unlikely(!dent)) {
		err = -ENOMEM;
		goto out_unlock;
	}

	/*
	 * The above 'tnc_next()' call could lead us to the next inode, check
	 * this.
	 */
	dkey = &zbr->key;
	if (key_inum(c, dkey) != key_inum(c, key) ||
	    key_type(c, dkey) != type) {
		err = -ENOENT;
		goto out_free;
	}

	err = tnc_read_node_nm(c, zbr, dent);
	if (unlikely(err))
		goto out_free;

	mutex_unlock(&c->tnc_mutex);
	return dent;

out_free:
	kfree(dent);
out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return ERR_PTR(err);
}

static void tnc_destroy_cnext(struct ubifs_info *c)
{
	struct ubifs_znode *cnext;

	if (!c->cnext)
		return;
	ubifs_assert(c->cmt_state == COMMIT_BROKEN);
	cnext = c->cnext;
	do {
		struct ubifs_znode *znode = cnext;

		cnext = cnext->cnext;
		if (test_bit(OBSOLETE_ZNODE, &znode->flags))
			kfree(znode);
	} while (cnext && cnext != c->cnext);
}

void ubifs_tnc_close(struct ubifs_info *c)
{
	long clean_freed;

	tnc_destroy_cnext(c);
	if (c->zroot.znode) {
		clean_freed = ubifs_destroy_tnc_subtree(c->zroot.znode);
		atomic_long_sub(clean_freed, &ubifs_clean_zn_cnt);
	}
	kfree(c->gap_lebs);
	kfree(c->ilebs);
	destroy_old_idx(c);
}

static struct ubifs_znode *left_znode(struct ubifs_info *c,
				      struct ubifs_znode *znode)
{
	int level = znode->level;

	while (1) {
		int n = znode->iip - 1;

		/* Go up until we can go left */
		znode = znode->parent;
		if (!znode)
			return NULL;
		if (n >= 0) {
			/* Now go down the rightmost branch to 'level' */
			znode = get_znode(c, znode, n);
			if (IS_ERR(znode))
				return znode;
			while (znode->level != level) {
				n = znode->child_cnt - 1;
				znode = get_znode(c, znode, n);
				if (IS_ERR(znode))
					return znode;
			}
			break;
		}
	}
	return znode;
}

static struct ubifs_znode *right_znode(struct ubifs_info *c,
				       struct ubifs_znode *znode)
{
	int level = znode->level;

	while (1) {
		int n = znode->iip + 1;

		/* Go up until we can go right */
		znode = znode->parent;
		if (!znode)
			return NULL;
		if (n < znode->child_cnt) {
			/* Now go down the leftmost branch to 'level' */
			znode = get_znode(c, znode, n);
			if (IS_ERR(znode))
				return znode;
			while (znode->level != level) {
				znode = get_znode(c, znode, 0);
				if (IS_ERR(znode))
					return znode;
			}
			break;
		}
	}
	return znode;
}

static struct ubifs_znode *lookup_znode(struct ubifs_info *c,
					union ubifs_key *key, int level,
					int lnum, int offs)
{
	struct ubifs_znode *znode, *zn;
	int n, nn;

	/*
	 * The arguments have probably been read off flash, so don't assume
	 * they are valid.
	 */
	if (level < 0)
		return ERR_PTR(-EINVAL);

	/* Get the root znode */
	znode = c->zroot.znode;
	if (!znode) {
		znode = ubifs_load_znode(c, &c->zroot, NULL, 0);
		if (IS_ERR(znode))
			return znode;
	}
	/* Check if it is the one we are looking for */
	if (c->zroot.lnum == lnum && c->zroot.offs == offs)
		return znode;
	/* Descend to the parent level i.e. (level + 1) */
	if (level >= znode->level)
		return NULL;
	while (1) {
		ubifs_search_zbranch(c, znode, key, &n);
		if (n < 0) {
			/*
			 * We reached a znode where the leftmost key is greater
			 * than the key we are searching for. This is the same
			 * situation as the one described in a huge comment at
			 * the end of the 'ubifs_lookup_level0()' function. And
			 * for exactly the same reasons we have to try to look
			 * left before giving up.
			 */
			znode = left_znode(c, znode);
			if (!znode)
				return NULL;
			if (IS_ERR(znode))
				return znode;
			ubifs_search_zbranch(c, znode, key, &n);
			ubifs_assert(n >= 0);
		}
		if (znode->level == level + 1)
			break;
		znode = get_znode(c, znode, n);
		if (IS_ERR(znode))
			return znode;
	}
	/* Check if the child is the one we are looking for */
	if (znode->zbranch[n].lnum == lnum && znode->zbranch[n].offs == offs)
		return get_znode(c, znode, n);
	/* If the key is unique, there is nowhere else to look */
	if (!is_hash_key(c, key))
		return NULL;
	/*
	 * The key is not unique and so may be also in the znodes to either
	 * side.
	 */
	zn = znode;
	nn = n;
	/* Look left */
	while (1) {
		/* Move one branch to the left */
		if (n)
			n -= 1;
		else {
			znode = left_znode(c, znode);
			if (!znode)
				break;
			if (IS_ERR(znode))
				return znode;
			n = znode->child_cnt - 1;
		}
		/* Check it */
		if (znode->zbranch[n].lnum == lnum &&
		    znode->zbranch[n].offs == offs)
			return get_znode(c, znode, n);
		/* Stop if the key is less than the one we are looking for */
		if (keys_cmp(c, &znode->zbranch[n].key, key) < 0)
			break;
	}
	/* Back to the middle */
	znode = zn;
	n = nn;
	/* Look right */
	while (1) {
		/* Move one branch to the right */
		if (++n >= znode->child_cnt) {
			znode = right_znode(c, znode);
			if (!znode)
				break;
			if (IS_ERR(znode))
				return znode;
			n = 0;
		}
		/* Check it */
		if (znode->zbranch[n].lnum == lnum &&
		    znode->zbranch[n].offs == offs)
			return get_znode(c, znode, n);
		/* Stop if the key is greater than the one we are looking for */
		if (keys_cmp(c, &znode->zbranch[n].key, key) > 0)
			break;
	}
	return NULL;
}

int is_idx_node_in_tnc(struct ubifs_info *c, union ubifs_key *key, int level,
		       int lnum, int offs)
{
	struct ubifs_znode *znode;

	znode = lookup_znode(c, key, level, lnum, offs);
	if (!znode)
		return 0;
	if (IS_ERR(znode))
		return PTR_ERR(znode);

	return ubifs_zn_dirty(znode) ? 1 : 2;
}

static int is_leaf_node_in_tnc(struct ubifs_info *c, union ubifs_key *key,
			       int lnum, int offs)
{
	struct ubifs_zbranch *zbr;
	struct ubifs_znode *znode, *zn;
	int n, found, err, nn;
	const int unique = !is_hash_key(c, key);

	found = ubifs_lookup_level0(c, key, &znode, &n);
	if (found < 0)
		return found; /* Error code */
	if (!found)
		return 0;
	zbr = &znode->zbranch[n];
	if (lnum == zbr->lnum && offs == zbr->offs)
		return 1; /* Found it */
	if (unique)
		return 0;
	/*
	 * Because the key is not unique, we have to look left
	 * and right as well
	 */
	zn = znode;
	nn = n;
	/* Look left */
	while (1) {
		err = tnc_prev(c, &znode, &n);
		if (err == -ENOENT)
			break;
		if (err)
			return err;
		if (keys_cmp(c, key, &znode->zbranch[n].key))
			break;
		zbr = &znode->zbranch[n];
		if (lnum == zbr->lnum && offs == zbr->offs)
			return 1; /* Found it */
	}
	/* Look right */
	znode = zn;
	n = nn;
	while (1) {
		err = tnc_next(c, &znode, &n);
		if (err) {
			if (err == -ENOENT)
				return 0;
			return err;
		}
		if (keys_cmp(c, key, &znode->zbranch[n].key))
			break;
		zbr = &znode->zbranch[n];
		if (lnum == zbr->lnum && offs == zbr->offs)
			return 1; /* Found it */
	}
	return 0;
}

int ubifs_tnc_has_node(struct ubifs_info *c, union ubifs_key *key, int level,
		       int lnum, int offs, int is_idx)
{
	int err;

	mutex_lock(&c->tnc_mutex);
	if (is_idx) {
		err = is_idx_node_in_tnc(c, key, level, lnum, offs);
		if (err < 0)
			goto out_unlock;
		if (err == 1)
			/* The index node was found but it was dirty */
			err = 0;
		else if (err == 2)
			/* The index node was found and it was clean */
			err = 1;
		else
			BUG_ON(err != 0);
	} else
		err = is_leaf_node_in_tnc(c, key, lnum, offs);

out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

int ubifs_dirty_idx_node(struct ubifs_info *c, union ubifs_key *key, int level,
			 int lnum, int offs)
{
	struct ubifs_znode *znode;
	int err = 0;

	mutex_lock(&c->tnc_mutex);
	znode = lookup_znode(c, key, level, lnum, offs);
	if (!znode)
		goto out_unlock;
	if (IS_ERR(znode)) {
		err = PTR_ERR(znode);
		goto out_unlock;
	}
	znode = dirty_cow_bottom_up(c, znode);
	if (IS_ERR(znode)) {
		err = PTR_ERR(znode);
		goto out_unlock;
	}

out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

#ifdef CONFIG_UBIFS_FS_DEBUG

int dbg_check_inode_size(struct ubifs_info *c, const struct inode *inode,
			 loff_t size)
{
	int err, n;
	union ubifs_key from_key, to_key, *key;
	struct ubifs_znode *znode;
	unsigned int block;

	if (!S_ISREG(inode->i_mode))
		return 0;
	if (!(ubifs_chk_flags & UBIFS_CHK_GEN))
		return 0;

	block = (size + UBIFS_BLOCK_SIZE - 1) >> UBIFS_BLOCK_SHIFT;
	data_key_init(c, &from_key, inode->i_ino, block);
	highest_data_key(c, &to_key, inode->i_ino);

	mutex_lock(&c->tnc_mutex);
	err = ubifs_lookup_level0(c, &from_key, &znode, &n);
	if (err < 0)
		goto out_unlock;

	if (err) {
		err = -EINVAL;
		key = &from_key;
		goto out_dump;
	}

	err = tnc_next(c, &znode, &n);
	if (err == -ENOENT) {
		err = 0;
		goto out_unlock;
	}
	if (err < 0)
		goto out_unlock;

	ubifs_assert(err == 0);
	key = &znode->zbranch[n].key;
	if (!key_in_range(c, key, &from_key, &to_key))
		goto out_unlock;

out_dump:
	block = key_block(c, key);
	ubifs_err("inode %lu has size %lld, but there are data at offset %lld "
		  "(data key %s)", (unsigned long)inode->i_ino, size,
		  ((loff_t)block) << UBIFS_BLOCK_SHIFT, DBGKEY(key));
	dbg_dump_inode(c, inode);
	dbg_dump_stack();
	err = -EINVAL;

out_unlock:
	mutex_unlock(&c->tnc_mutex);
	return err;
}

#endif /* CONFIG_UBIFS_FS_DEBUG */
