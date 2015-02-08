

 /*
  * See Documentation/filesystems/nfs/Exporting
  * and examples in fs/exportfs
  *
  * Since cifs is a network file system, an "fsid" must be included for
  * any nfs exports file entries which refer to cifs paths.  In addition
  * the cifs mount must be mounted with the "serverino" option (ie use stable
  * server inode numbers instead of locally generated temporary ones).
  * Although cifs inodes do not use generation numbers (have generation number
  * of zero) - the inode number alone should be good enough for simple cases
  * in which users want to export cifs shares with NFS. The decode and encode
  * could be improved by using a new routine which expects 64 bit inode numbers
  * instead of the default 32 bit routines in fs/exportfs
  *
  */

#include <linux/fs.h>
#include <linux/exportfs.h>
#include "cifsglob.h"
#include "cifs_debug.h"
#include "cifsfs.h"

#ifdef CONFIG_CIFS_EXPERIMENTAL
static struct dentry *cifs_get_parent(struct dentry *dentry)
{
	/* BB need to add code here eventually to enable export via NFSD */
	cFYI(1, "get parent for %p", dentry);
	return ERR_PTR(-EACCES);
}

const struct export_operations cifs_export_ops = {
	.get_parent = cifs_get_parent,
};

#endif /* EXPERIMENTAL */

