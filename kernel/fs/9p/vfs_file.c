

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/inet.h>
#include <linux/list.h>
#include <linux/pagemap.h>
#include <asm/uaccess.h>
#include <linux/idr.h>
#include <net/9p/9p.h>
#include <net/9p/client.h>

#include "v9fs.h"
#include "v9fs_vfs.h"
#include "fid.h"
#include "cache.h"

static const struct file_operations v9fs_cached_file_operations;


int v9fs_file_open(struct inode *inode, struct file *file)
{
	int err;
	struct v9fs_session_info *v9ses;
	struct p9_fid *fid;
	int omode;

	P9_DPRINTK(P9_DEBUG_VFS, "inode: %p file: %p \n", inode, file);
	v9ses = v9fs_inode2v9ses(inode);
	omode = v9fs_uflags2omode(file->f_flags, v9fs_proto_dotu(v9ses));
	fid = file->private_data;
	if (!fid) {
		fid = v9fs_fid_clone(file->f_path.dentry);
		if (IS_ERR(fid))
			return PTR_ERR(fid);

		err = p9_client_open(fid, omode);
		if (err < 0) {
			p9_client_clunk(fid);
			return err;
		}
		if (omode & P9_OTRUNC) {
			i_size_write(inode, 0);
			inode->i_blocks = 0;
		}
		if ((file->f_flags & O_APPEND) && (!v9fs_proto_dotu(v9ses)))
			generic_file_llseek(file, 0, SEEK_END);
	}

	file->private_data = fid;
	if ((fid->qid.version) && (v9ses->cache)) {
		P9_DPRINTK(P9_DEBUG_VFS, "cached");
		/* enable cached file options */
		if(file->f_op == &v9fs_file_operations)
			file->f_op = &v9fs_cached_file_operations;

#ifdef CONFIG_9P_FSCACHE
		v9fs_cache_inode_set_cookie(inode, file);
#endif
	}

	return 0;
}


static int v9fs_file_lock(struct file *filp, int cmd, struct file_lock *fl)
{
	int res = 0;
	struct inode *inode = filp->f_path.dentry->d_inode;

	P9_DPRINTK(P9_DEBUG_VFS, "filp: %p lock: %p\n", filp, fl);

	/* No mandatory locks */
	if (__mandatory_lock(inode) && fl->fl_type != F_UNLCK)
		return -ENOLCK;

	if ((IS_SETLK(cmd) || IS_SETLKW(cmd)) && fl->fl_type != F_UNLCK) {
		filemap_write_and_wait(inode->i_mapping);
		invalidate_mapping_pages(&inode->i_data, 0, -1);
	}

	return res;
}


ssize_t
v9fs_file_readn(struct file *filp, char *data, char __user *udata, u32 count,
	       u64 offset)
{
	int n, total;
	struct p9_fid *fid = filp->private_data;

	P9_DPRINTK(P9_DEBUG_VFS, "fid %d offset %llu count %d\n", fid->fid,
					(long long unsigned) offset, count);

	n = 0;
	total = 0;
	do {
		n = p9_client_read(fid, data, udata, offset, count);
		if (n <= 0)
			break;

		if (data)
			data += n;
		if (udata)
			udata += n;

		offset += n;
		count -= n;
		total += n;
	} while (count > 0 && n == (fid->clnt->msize - P9_IOHDRSZ));

	if (n < 0)
		total = n;

	return total;
}


static ssize_t
v9fs_file_read(struct file *filp, char __user *udata, size_t count,
	       loff_t * offset)
{
	int ret;
	struct p9_fid *fid;

	P9_DPRINTK(P9_DEBUG_VFS, "count %zu offset %lld\n", count, *offset);
	fid = filp->private_data;

	if (count > (fid->clnt->msize - P9_IOHDRSZ))
		ret = v9fs_file_readn(filp, NULL, udata, count, *offset);
	else
		ret = p9_client_read(fid, NULL, udata, *offset, count);

	if (ret > 0)
		*offset += ret;

	return ret;
}


static ssize_t
v9fs_file_write(struct file *filp, const char __user * data,
		size_t count, loff_t * offset)
{
	int n, rsize, total = 0;
	struct p9_fid *fid;
	struct p9_client *clnt;
	struct inode *inode = filp->f_path.dentry->d_inode;
	loff_t origin = *offset;
	unsigned long pg_start, pg_end;

	P9_DPRINTK(P9_DEBUG_VFS, "data %p count %d offset %x\n", data,
		(int)count, (int)*offset);

	fid = filp->private_data;
	clnt = fid->clnt;

	rsize = fid->iounit;
	if (!rsize || rsize > clnt->msize-P9_IOHDRSZ)
		rsize = clnt->msize - P9_IOHDRSZ;

	do {
		if (count < rsize)
			rsize = count;

		n = p9_client_write(fid, NULL, data+total, origin+total,
									rsize);
		if (n <= 0)
			break;
		count -= n;
		total += n;
	} while (count > 0);

	if (total > 0) {
		pg_start = origin >> PAGE_CACHE_SHIFT;
		pg_end = (origin + total - 1) >> PAGE_CACHE_SHIFT;
		if (inode->i_mapping && inode->i_mapping->nrpages)
			invalidate_inode_pages2_range(inode->i_mapping,
						      pg_start, pg_end);
		*offset += total;
		i_size_write(inode, i_size_read(inode) + total);
		inode->i_blocks = (i_size_read(inode) + 512 - 1) >> 9;
	}

	if (n < 0)
		return n;

	return total;
}

static int v9fs_file_fsync(struct file *filp, int datasync)
{
	struct p9_fid *fid;
	struct p9_wstat wstat;
	int retval;

	P9_DPRINTK(P9_DEBUG_VFS, "filp %p datasync %x\n", filp, datasync);

	fid = filp->private_data;
	v9fs_blank_wstat(&wstat);

	retval = p9_client_wstat(fid, &wstat);
	return retval;
}

static const struct file_operations v9fs_cached_file_operations = {
	.llseek = generic_file_llseek,
	.read = do_sync_read,
	.aio_read = generic_file_aio_read,
	.write = v9fs_file_write,
	.open = v9fs_file_open,
	.release = v9fs_dir_release,
	.lock = v9fs_file_lock,
	.mmap = generic_file_readonly_mmap,
	.fsync = v9fs_file_fsync,
};

const struct file_operations v9fs_file_operations = {
	.llseek = generic_file_llseek,
	.read = v9fs_file_read,
	.write = v9fs_file_write,
	.open = v9fs_file_open,
	.release = v9fs_dir_release,
	.lock = v9fs_file_lock,
	.mmap = generic_file_readonly_mmap,
	.fsync = v9fs_file_fsync,
};

const struct file_operations v9fs_file_operations_dotl = {
	.llseek = generic_file_llseek,
	.read = v9fs_file_read,
	.write = v9fs_file_write,
	.open = v9fs_file_open,
	.release = v9fs_dir_release,
	.lock = v9fs_file_lock,
	.mmap = generic_file_readonly_mmap,
	.fsync = v9fs_file_fsync,
};
