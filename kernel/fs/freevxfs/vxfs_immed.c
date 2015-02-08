

#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/namei.h>

#include "vxfs.h"
#include "vxfs_extern.h"
#include "vxfs_inode.h"


static void *	vxfs_immed_follow_link(struct dentry *, struct nameidata *);

static int	vxfs_immed_readpage(struct file *, struct page *);

const struct inode_operations vxfs_immed_symlink_iops = {
	.readlink =		generic_readlink,
	.follow_link =		vxfs_immed_follow_link,
};

const struct address_space_operations vxfs_immed_aops = {
	.readpage =		vxfs_immed_readpage,
};

static void *
vxfs_immed_follow_link(struct dentry *dp, struct nameidata *np)
{
	struct vxfs_inode_info		*vip = VXFS_INO(dp->d_inode);
	nd_set_link(np, vip->vii_immed.vi_immed);
	return NULL;
}

static int
vxfs_immed_readpage(struct file *fp, struct page *pp)
{
	struct vxfs_inode_info	*vip = VXFS_INO(pp->mapping->host);
	u_int64_t	offset = (u_int64_t)pp->index << PAGE_CACHE_SHIFT;
	caddr_t		kaddr;

	kaddr = kmap(pp);
	memcpy(kaddr, vip->vii_immed.vi_immed + offset, PAGE_CACHE_SIZE);
	kunmap(pp);
	
	flush_dcache_page(pp);
	SetPageUptodate(pp);
        unlock_page(pp);

	return 0;
}
