

#include "ctree.h"
#include "disk-io.h"
#include "transaction.h"

int btrfs_find_highest_inode(struct btrfs_root *root, u64 *objectid)
{
	struct btrfs_path *path;
	int ret;
	struct extent_buffer *l;
	struct btrfs_key search_key;
	struct btrfs_key found_key;
	int slot;

	path = btrfs_alloc_path();
	BUG_ON(!path);

	search_key.objectid = BTRFS_LAST_FREE_OBJECTID;
	search_key.type = -1;
	search_key.offset = (u64)-1;
	ret = btrfs_search_slot(NULL, root, &search_key, path, 0, 0);
	if (ret < 0)
		goto error;
	BUG_ON(ret == 0);
	if (path->slots[0] > 0) {
		slot = path->slots[0] - 1;
		l = path->nodes[0];
		btrfs_item_key_to_cpu(l, &found_key, slot);
		*objectid = max_t(u64, found_key.objectid,
				  BTRFS_FIRST_FREE_OBJECTID - 1);
	} else {
		*objectid = BTRFS_FIRST_FREE_OBJECTID - 1;
	}
	ret = 0;
error:
	btrfs_free_path(path);
	return ret;
}

int btrfs_find_free_objectid(struct btrfs_trans_handle *trans,
			     struct btrfs_root *root,
			     u64 dirid, u64 *objectid)
{
	int ret;
	mutex_lock(&root->objectid_mutex);

	if (unlikely(root->highest_objectid < BTRFS_FIRST_FREE_OBJECTID)) {
		ret = btrfs_find_highest_inode(root, &root->highest_objectid);
		if (ret)
			goto out;
	}

	if (unlikely(root->highest_objectid >= BTRFS_LAST_FREE_OBJECTID)) {
		ret = -ENOSPC;
		goto out;
	}

	*objectid = ++root->highest_objectid;
	ret = 0;
out:
	mutex_unlock(&root->objectid_mutex);
	return ret;
}
