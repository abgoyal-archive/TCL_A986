

#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/posix_acl_xattr.h>
#include "jfs_incore.h"
#include "jfs_txnmgr.h"
#include "jfs_xattr.h"
#include "jfs_acl.h"

static struct posix_acl *jfs_get_acl(struct inode *inode, int type)
{
	struct posix_acl *acl;
	char *ea_name;
	int size;
	char *value = NULL;

	acl = get_cached_acl(inode, type);
	if (acl != ACL_NOT_CACHED)
		return acl;

	switch(type) {
		case ACL_TYPE_ACCESS:
			ea_name = POSIX_ACL_XATTR_ACCESS;
			break;
		case ACL_TYPE_DEFAULT:
			ea_name = POSIX_ACL_XATTR_DEFAULT;
			break;
		default:
			return ERR_PTR(-EINVAL);
	}

	size = __jfs_getxattr(inode, ea_name, NULL, 0);

	if (size > 0) {
		value = kmalloc(size, GFP_KERNEL);
		if (!value)
			return ERR_PTR(-ENOMEM);
		size = __jfs_getxattr(inode, ea_name, value, size);
	}

	if (size < 0) {
		if (size == -ENODATA)
			acl = NULL;
		else
			acl = ERR_PTR(size);
	} else {
		acl = posix_acl_from_xattr(value, size);
	}
	kfree(value);
	if (!IS_ERR(acl))
		set_cached_acl(inode, type, acl);
	return acl;
}

static int jfs_set_acl(tid_t tid, struct inode *inode, int type,
		       struct posix_acl *acl)
{
	char *ea_name;
	int rc;
	int size = 0;
	char *value = NULL;

	if (S_ISLNK(inode->i_mode))
		return -EOPNOTSUPP;

	switch(type) {
		case ACL_TYPE_ACCESS:
			ea_name = POSIX_ACL_XATTR_ACCESS;
			break;
		case ACL_TYPE_DEFAULT:
			ea_name = POSIX_ACL_XATTR_DEFAULT;
			if (!S_ISDIR(inode->i_mode))
				return acl ? -EACCES : 0;
			break;
		default:
			return -EINVAL;
	}
	if (acl) {
		size = posix_acl_xattr_size(acl->a_count);
		value = kmalloc(size, GFP_KERNEL);
		if (!value)
			return -ENOMEM;
		rc = posix_acl_to_xattr(acl, value, size);
		if (rc < 0)
			goto out;
	}
	rc = __jfs_setxattr(tid, inode, ea_name, value, size, 0);
out:
	kfree(value);

	if (!rc)
		set_cached_acl(inode, type, acl);

	return rc;
}

int jfs_check_acl(struct inode *inode, int mask)
{
	struct posix_acl *acl = jfs_get_acl(inode, ACL_TYPE_ACCESS);

	if (IS_ERR(acl))
		return PTR_ERR(acl);
	if (acl) {
		int error = posix_acl_permission(inode, acl, mask);
		posix_acl_release(acl);
		return error;
	}

	return -EAGAIN;
}

int jfs_init_acl(tid_t tid, struct inode *inode, struct inode *dir)
{
	struct posix_acl *acl = NULL;
	struct posix_acl *clone;
	mode_t mode;
	int rc = 0;

	if (S_ISLNK(inode->i_mode))
		return 0;

	acl = jfs_get_acl(dir, ACL_TYPE_DEFAULT);
	if (IS_ERR(acl))
		return PTR_ERR(acl);

	if (acl) {
		if (S_ISDIR(inode->i_mode)) {
			rc = jfs_set_acl(tid, inode, ACL_TYPE_DEFAULT, acl);
			if (rc)
				goto cleanup;
		}
		clone = posix_acl_clone(acl, GFP_KERNEL);
		if (!clone) {
			rc = -ENOMEM;
			goto cleanup;
		}
		mode = inode->i_mode;
		rc = posix_acl_create_masq(clone, &mode);
		if (rc >= 0) {
			inode->i_mode = mode;
			if (rc > 0)
				rc = jfs_set_acl(tid, inode, ACL_TYPE_ACCESS,
						 clone);
		}
		posix_acl_release(clone);
cleanup:
		posix_acl_release(acl);
	} else
		inode->i_mode &= ~current_umask();

	JFS_IP(inode)->mode2 = (JFS_IP(inode)->mode2 & 0xffff0000) |
			       inode->i_mode;

	return rc;
}

int jfs_acl_chmod(struct inode *inode)
{
	struct posix_acl *acl, *clone;
	int rc;

	if (S_ISLNK(inode->i_mode))
		return -EOPNOTSUPP;

	acl = jfs_get_acl(inode, ACL_TYPE_ACCESS);
	if (IS_ERR(acl) || !acl)
		return PTR_ERR(acl);

	clone = posix_acl_clone(acl, GFP_KERNEL);
	posix_acl_release(acl);
	if (!clone)
		return -ENOMEM;

	rc = posix_acl_chmod_masq(clone, inode->i_mode);
	if (!rc) {
		tid_t tid = txBegin(inode->i_sb, 0);
		mutex_lock(&JFS_IP(inode)->commit_mutex);
		rc = jfs_set_acl(tid, inode, ACL_TYPE_ACCESS, clone);
		if (!rc)
			rc = txCommit(tid, 1, &inode, 0);
		txEnd(tid);
		mutex_unlock(&JFS_IP(inode)->commit_mutex);
	}

	posix_acl_release(clone);
	return rc;
}
