

#ifndef OCFS2_ACL_H
#define OCFS2_ACL_H

#include <linux/posix_acl_xattr.h>

struct ocfs2_acl_entry {
	__le16 e_tag;
	__le16 e_perm;
	__le32 e_id;
};

extern int ocfs2_check_acl(struct inode *, int);
extern int ocfs2_acl_chmod(struct inode *);
extern int ocfs2_init_acl(handle_t *, struct inode *, struct inode *,
			  struct buffer_head *, struct buffer_head *,
			  struct ocfs2_alloc_context *,
			  struct ocfs2_alloc_context *);

#endif /* OCFS2_ACL_H */
