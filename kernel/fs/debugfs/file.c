

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/namei.h>
#include <linux/debugfs.h>

static ssize_t default_read_file(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t default_write_file(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	return count;
}

static int default_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

const struct file_operations debugfs_file_operations = {
	.read =		default_read_file,
	.write =	default_write_file,
	.open =		default_open,
};

static void *debugfs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	nd_set_link(nd, dentry->d_inode->i_private);
	return NULL;
}

const struct inode_operations debugfs_link_operations = {
	.readlink       = generic_readlink,
	.follow_link    = debugfs_follow_link,
};

static int debugfs_u8_set(void *data, u64 val)
{
	*(u8 *)data = val;
	return 0;
}
static int debugfs_u8_get(void *data, u64 *val)
{
	*val = *(u8 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u8, debugfs_u8_get, debugfs_u8_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u8_ro, debugfs_u8_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u8_wo, NULL, debugfs_u8_set, "%llu\n");

struct dentry *debugfs_create_u8(const char *name, mode_t mode,
				 struct dentry *parent, u8 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u8_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u8_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u8);
}
EXPORT_SYMBOL_GPL(debugfs_create_u8);

static int debugfs_u16_set(void *data, u64 val)
{
	*(u16 *)data = val;
	return 0;
}
static int debugfs_u16_get(void *data, u64 *val)
{
	*val = *(u16 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u16, debugfs_u16_get, debugfs_u16_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u16_ro, debugfs_u16_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u16_wo, NULL, debugfs_u16_set, "%llu\n");

struct dentry *debugfs_create_u16(const char *name, mode_t mode,
				  struct dentry *parent, u16 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u16_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u16_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u16);
}
EXPORT_SYMBOL_GPL(debugfs_create_u16);

static int debugfs_u32_set(void *data, u64 val)
{
	*(u32 *)data = val;
	return 0;
}
static int debugfs_u32_get(void *data, u64 *val)
{
	*val = *(u32 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u32, debugfs_u32_get, debugfs_u32_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u32_ro, debugfs_u32_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u32_wo, NULL, debugfs_u32_set, "%llu\n");

struct dentry *debugfs_create_u32(const char *name, mode_t mode,
				 struct dentry *parent, u32 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u32_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u32_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u32);
}
EXPORT_SYMBOL_GPL(debugfs_create_u32);

static int debugfs_u64_set(void *data, u64 val)
{
	*(u64 *)data = val;
	return 0;
}

static int debugfs_u64_get(void *data, u64 *val)
{
	*val = *(u64 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u64, debugfs_u64_get, debugfs_u64_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u64_ro, debugfs_u64_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u64_wo, NULL, debugfs_u64_set, "%llu\n");

struct dentry *debugfs_create_u64(const char *name, mode_t mode,
				 struct dentry *parent, u64 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u64_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u64_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u64);
}
EXPORT_SYMBOL_GPL(debugfs_create_u64);

DEFINE_SIMPLE_ATTRIBUTE(fops_x8, debugfs_u8_get, debugfs_u8_set, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x8_ro, debugfs_u8_get, NULL, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x8_wo, NULL, debugfs_u8_set, "0x%02llx\n");

DEFINE_SIMPLE_ATTRIBUTE(fops_x16, debugfs_u16_get, debugfs_u16_set, "0x%04llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x16_ro, debugfs_u16_get, NULL, "0x%04llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x16_wo, NULL, debugfs_u16_set, "0x%04llx\n");

DEFINE_SIMPLE_ATTRIBUTE(fops_x32, debugfs_u32_get, debugfs_u32_set, "0x%08llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x32_ro, debugfs_u32_get, NULL, "0x%08llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x32_wo, NULL, debugfs_u32_set, "0x%08llx\n");

DEFINE_SIMPLE_ATTRIBUTE(fops_x64, debugfs_u64_get, debugfs_u64_set, "0x%016llx\n");


struct dentry *debugfs_create_x8(const char *name, mode_t mode,
				 struct dentry *parent, u8 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x8_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x8_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_x8);
}
EXPORT_SYMBOL_GPL(debugfs_create_x8);

struct dentry *debugfs_create_x16(const char *name, mode_t mode,
				 struct dentry *parent, u16 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x16_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x16_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_x16);
}
EXPORT_SYMBOL_GPL(debugfs_create_x16);

struct dentry *debugfs_create_x32(const char *name, mode_t mode,
				 struct dentry *parent, u32 *value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x32_ro);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x32_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_x32);
}
EXPORT_SYMBOL_GPL(debugfs_create_x32);

struct dentry *debugfs_create_x64(const char *name, mode_t mode,
				 struct dentry *parent, u64 *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_x64);
}
EXPORT_SYMBOL_GPL(debugfs_create_x64);


static int debugfs_size_t_set(void *data, u64 val)
{
	*(size_t *)data = val;
	return 0;
}
static int debugfs_size_t_get(void *data, u64 *val)
{
	*val = *(size_t *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_size_t, debugfs_size_t_get, debugfs_size_t_set,
			"%llu\n");	/* %llu and %zu are more or less the same */

struct dentry *debugfs_create_size_t(const char *name, mode_t mode,
				     struct dentry *parent, size_t *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_size_t);
}
EXPORT_SYMBOL_GPL(debugfs_create_size_t);


static ssize_t read_file_bool(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	char buf[3];
	u32 *val = file->private_data;
	
	if (*val)
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t write_file_bool(struct file *file, const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	char buf[32];
	int buf_size;
	u32 *val = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	switch (buf[0]) {
	case 'y':
	case 'Y':
	case '1':
		*val = 1;
		break;
	case 'n':
	case 'N':
	case '0':
		*val = 0;
		break;
	}
	
	return count;
}

static const struct file_operations fops_bool = {
	.read =		read_file_bool,
	.write =	write_file_bool,
	.open =		default_open,
};

struct dentry *debugfs_create_bool(const char *name, mode_t mode,
				   struct dentry *parent, u32 *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_bool);
}
EXPORT_SYMBOL_GPL(debugfs_create_bool);

static ssize_t read_file_blob(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct debugfs_blob_wrapper *blob = file->private_data;
	return simple_read_from_buffer(user_buf, count, ppos, blob->data,
			blob->size);
}

static const struct file_operations fops_blob = {
	.read =		read_file_blob,
	.open =		default_open,
};

struct dentry *debugfs_create_blob(const char *name, mode_t mode,
				   struct dentry *parent,
				   struct debugfs_blob_wrapper *blob)
{
	return debugfs_create_file(name, mode, parent, blob, &fops_blob);
}
EXPORT_SYMBOL_GPL(debugfs_create_blob);
