

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/slab.h>

#include <linux/configfs.h>
#include "configfs_internal.h"

/* Random magic number */
#define CONFIGFS_MAGIC 0x62656570

struct vfsmount * configfs_mount = NULL;
struct super_block * configfs_sb = NULL;
struct kmem_cache *configfs_dir_cachep;
static int configfs_mnt_count = 0;

static const struct super_operations configfs_ops = {
	.statfs		= simple_statfs,
	.drop_inode	= generic_delete_inode,
};

static struct config_group configfs_root_group = {
	.cg_item = {
		.ci_namebuf	= "root",
		.ci_name	= configfs_root_group.cg_item.ci_namebuf,
	},
};

int configfs_is_root(struct config_item *item)
{
	return item == &configfs_root_group.cg_item;
}

static struct configfs_dirent configfs_root = {
	.s_sibling	= LIST_HEAD_INIT(configfs_root.s_sibling),
	.s_children	= LIST_HEAD_INIT(configfs_root.s_children),
	.s_element	= &configfs_root_group.cg_item,
	.s_type		= CONFIGFS_ROOT,
	.s_iattr	= NULL,
};

static int configfs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct inode *inode;
	struct dentry *root;

	sb->s_blocksize = PAGE_CACHE_SIZE;
	sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
	sb->s_magic = CONFIGFS_MAGIC;
	sb->s_op = &configfs_ops;
	sb->s_time_gran = 1;
	configfs_sb = sb;

	inode = configfs_new_inode(S_IFDIR | S_IRWXU | S_IRUGO | S_IXUGO,
				   &configfs_root);
	if (inode) {
		inode->i_op = &configfs_dir_inode_operations;
		inode->i_fop = &configfs_dir_operations;
		/* directory inodes start off with i_nlink == 2 (for "." entry) */
		inc_nlink(inode);
	} else {
		pr_debug("configfs: could not get root inode\n");
		return -ENOMEM;
	}

	root = d_alloc_root(inode);
	if (!root) {
		pr_debug("%s: could not get root dentry!\n",__func__);
		iput(inode);
		return -ENOMEM;
	}
	config_group_init(&configfs_root_group);
	configfs_root_group.cg_item.ci_dentry = root;
	root->d_fsdata = &configfs_root;
	sb->s_root = root;
	return 0;
}

static int configfs_get_sb(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data, struct vfsmount *mnt)
{
	return get_sb_single(fs_type, flags, data, configfs_fill_super, mnt);
}

static struct file_system_type configfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "configfs",
	.get_sb		= configfs_get_sb,
	.kill_sb	= kill_litter_super,
};

int configfs_pin_fs(void)
{
	return simple_pin_fs(&configfs_fs_type, &configfs_mount,
			     &configfs_mnt_count);
}

void configfs_release_fs(void)
{
	simple_release_fs(&configfs_mount, &configfs_mnt_count);
}


static struct kobject *config_kobj;

static int __init configfs_init(void)
{
	int err = -ENOMEM;

	configfs_dir_cachep = kmem_cache_create("configfs_dir_cache",
						sizeof(struct configfs_dirent),
						0, 0, NULL);
	if (!configfs_dir_cachep)
		goto out;

	config_kobj = kobject_create_and_add("config", kernel_kobj);
	if (!config_kobj) {
		kmem_cache_destroy(configfs_dir_cachep);
		configfs_dir_cachep = NULL;
		goto out;
	}

	err = register_filesystem(&configfs_fs_type);
	if (err) {
		printk(KERN_ERR "configfs: Unable to register filesystem!\n");
		kobject_put(config_kobj);
		kmem_cache_destroy(configfs_dir_cachep);
		configfs_dir_cachep = NULL;
		goto out;
	}

	err = configfs_inode_init();
	if (err) {
		unregister_filesystem(&configfs_fs_type);
		kobject_put(config_kobj);
		kmem_cache_destroy(configfs_dir_cachep);
		configfs_dir_cachep = NULL;
	}
out:
	return err;
}

static void __exit configfs_exit(void)
{
	unregister_filesystem(&configfs_fs_type);
	kobject_put(config_kobj);
	kmem_cache_destroy(configfs_dir_cachep);
	configfs_dir_cachep = NULL;
	configfs_inode_exit();
}

MODULE_AUTHOR("Oracle");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.2");
MODULE_DESCRIPTION("Simple RAM filesystem for user driven kernel subsystem configuration.");

module_init(configfs_init);
module_exit(configfs_exit);
