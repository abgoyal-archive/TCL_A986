

#include <linux/xattr.h>
#include <linux/pagemap.h>
#include <linux/mount.h>
#include <linux/stat.h>
#include <linux/kd.h>
#include <asm/ioctls.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/pipe_fs_i.h>
#include <net/netlabel.h>
#include <net/cipso_ipv4.h>
#include <linux/audit.h>
#include <linux/magic.h>
#include "smack.h"

#define task_security(task)	(task_cred_xxx((task), security))

static char *smk_fetch(struct inode *ip, struct dentry *dp)
{
	int rc;
	char in[SMK_LABELLEN];

	if (ip->i_op->getxattr == NULL)
		return NULL;

	rc = ip->i_op->getxattr(dp, XATTR_NAME_SMACK, in, SMK_LABELLEN);
	if (rc < 0)
		return NULL;

	return smk_import(in, rc);
}

struct inode_smack *new_inode_smack(char *smack)
{
	struct inode_smack *isp;

	isp = kzalloc(sizeof(struct inode_smack), GFP_KERNEL);
	if (isp == NULL)
		return NULL;

	isp->smk_inode = smack;
	isp->smk_flags = 0;
	mutex_init(&isp->smk_lock);

	return isp;
}


static int smack_ptrace_access_check(struct task_struct *ctp, unsigned int mode)
{
	int rc;
	struct smk_audit_info ad;
	char *sp, *tsp;

	rc = cap_ptrace_access_check(ctp, mode);
	if (rc != 0)
		return rc;

	sp = current_security();
	tsp = task_security(ctp);
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_tsk(&ad, ctp);

	/* we won't log here, because rc can be overriden */
	rc = smk_access(sp, tsp, MAY_READWRITE, NULL);
	if (rc != 0 && capable(CAP_MAC_OVERRIDE))
		rc = 0;

	smack_log(sp, tsp, MAY_READWRITE, rc, &ad);
	return rc;
}

static int smack_ptrace_traceme(struct task_struct *ptp)
{
	int rc;
	struct smk_audit_info ad;
	char *sp, *tsp;

	rc = cap_ptrace_traceme(ptp);
	if (rc != 0)
		return rc;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_tsk(&ad, ptp);

	sp = current_security();
	tsp = task_security(ptp);
	/* we won't log here, because rc can be overriden */
	rc = smk_access(tsp, sp, MAY_READWRITE, NULL);
	if (rc != 0 && has_capability(ptp, CAP_MAC_OVERRIDE))
		rc = 0;

	smack_log(tsp, sp, MAY_READWRITE, rc, &ad);
	return rc;
}

static int smack_syslog(int type, bool from_file)
{
	int rc;
	char *sp = current_security();

	rc = cap_syslog(type, from_file);
	if (rc != 0)
		return rc;

	if (capable(CAP_MAC_OVERRIDE))
		return 0;

	 if (sp != smack_known_floor.smk_known)
		rc = -EACCES;

	return rc;
}



static int smack_sb_alloc_security(struct super_block *sb)
{
	struct superblock_smack *sbsp;

	sbsp = kzalloc(sizeof(struct superblock_smack), GFP_KERNEL);

	if (sbsp == NULL)
		return -ENOMEM;

	sbsp->smk_root = smack_known_floor.smk_known;
	sbsp->smk_default = smack_known_floor.smk_known;
	sbsp->smk_floor = smack_known_floor.smk_known;
	sbsp->smk_hat = smack_known_hat.smk_known;
	sbsp->smk_initialized = 0;
	spin_lock_init(&sbsp->smk_sblock);

	sb->s_security = sbsp;

	return 0;
}

static void smack_sb_free_security(struct super_block *sb)
{
	kfree(sb->s_security);
	sb->s_security = NULL;
}

static int smack_sb_copy_data(char *orig, char *smackopts)
{
	char *cp, *commap, *otheropts, *dp;

	otheropts = (char *)get_zeroed_page(GFP_KERNEL);
	if (otheropts == NULL)
		return -ENOMEM;

	for (cp = orig, commap = orig; commap != NULL; cp = commap + 1) {
		if (strstr(cp, SMK_FSDEFAULT) == cp)
			dp = smackopts;
		else if (strstr(cp, SMK_FSFLOOR) == cp)
			dp = smackopts;
		else if (strstr(cp, SMK_FSHAT) == cp)
			dp = smackopts;
		else if (strstr(cp, SMK_FSROOT) == cp)
			dp = smackopts;
		else
			dp = otheropts;

		commap = strchr(cp, ',');
		if (commap != NULL)
			*commap = '\0';

		if (*dp != '\0')
			strcat(dp, ",");
		strcat(dp, cp);
	}

	strcpy(orig, otheropts);
	free_page((unsigned long)otheropts);

	return 0;
}

static int smack_sb_kern_mount(struct super_block *sb, int flags, void *data)
{
	struct dentry *root = sb->s_root;
	struct inode *inode = root->d_inode;
	struct superblock_smack *sp = sb->s_security;
	struct inode_smack *isp;
	char *op;
	char *commap;
	char *nsp;

	spin_lock(&sp->smk_sblock);
	if (sp->smk_initialized != 0) {
		spin_unlock(&sp->smk_sblock);
		return 0;
	}
	sp->smk_initialized = 1;
	spin_unlock(&sp->smk_sblock);

	for (op = data; op != NULL; op = commap) {
		commap = strchr(op, ',');
		if (commap != NULL)
			*commap++ = '\0';

		if (strncmp(op, SMK_FSHAT, strlen(SMK_FSHAT)) == 0) {
			op += strlen(SMK_FSHAT);
			nsp = smk_import(op, 0);
			if (nsp != NULL)
				sp->smk_hat = nsp;
		} else if (strncmp(op, SMK_FSFLOOR, strlen(SMK_FSFLOOR)) == 0) {
			op += strlen(SMK_FSFLOOR);
			nsp = smk_import(op, 0);
			if (nsp != NULL)
				sp->smk_floor = nsp;
		} else if (strncmp(op, SMK_FSDEFAULT,
				   strlen(SMK_FSDEFAULT)) == 0) {
			op += strlen(SMK_FSDEFAULT);
			nsp = smk_import(op, 0);
			if (nsp != NULL)
				sp->smk_default = nsp;
		} else if (strncmp(op, SMK_FSROOT, strlen(SMK_FSROOT)) == 0) {
			op += strlen(SMK_FSROOT);
			nsp = smk_import(op, 0);
			if (nsp != NULL)
				sp->smk_root = nsp;
		}
	}

	/*
	 * Initialize the root inode.
	 */
	isp = inode->i_security;
	if (isp == NULL)
		inode->i_security = new_inode_smack(sp->smk_root);
	else
		isp->smk_inode = sp->smk_root;

	return 0;
}

static int smack_sb_statfs(struct dentry *dentry)
{
	struct superblock_smack *sbp = dentry->d_sb->s_security;
	int rc;
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);

	rc = smk_curacc(sbp->smk_floor, MAY_READ, &ad);
	return rc;
}

static int smack_sb_mount(char *dev_name, struct path *path,
			  char *type, unsigned long flags, void *data)
{
	struct superblock_smack *sbp = path->mnt->mnt_sb->s_security;
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path(&ad, *path);

	return smk_curacc(sbp->smk_floor, MAY_WRITE, &ad);
}

static int smack_sb_umount(struct vfsmount *mnt, int flags)
{
	struct superblock_smack *sbp;
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, mnt->mnt_root);
	smk_ad_setfield_u_fs_path_mnt(&ad, mnt);

	sbp = mnt->mnt_sb->s_security;
	return smk_curacc(sbp->smk_floor, MAY_WRITE, &ad);
}


static int smack_inode_alloc_security(struct inode *inode)
{
	inode->i_security = new_inode_smack(current_security());
	if (inode->i_security == NULL)
		return -ENOMEM;
	return 0;
}

static void smack_inode_free_security(struct inode *inode)
{
	kfree(inode->i_security);
	inode->i_security = NULL;
}

static int smack_inode_init_security(struct inode *inode, struct inode *dir,
				     char **name, void **value, size_t *len)
{
	char *isp = smk_of_inode(inode);

	if (name) {
		*name = kstrdup(XATTR_SMACK_SUFFIX, GFP_KERNEL);
		if (*name == NULL)
			return -ENOMEM;
	}

	if (value) {
		*value = kstrdup(isp, GFP_KERNEL);
		if (*value == NULL)
			return -ENOMEM;
	}

	if (len)
		*len = strlen(isp) + 1;

	return 0;
}

static int smack_inode_link(struct dentry *old_dentry, struct inode *dir,
			    struct dentry *new_dentry)
{
	char *isp;
	struct smk_audit_info ad;
	int rc;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, old_dentry);

	isp = smk_of_inode(old_dentry->d_inode);
	rc = smk_curacc(isp, MAY_WRITE, &ad);

	if (rc == 0 && new_dentry->d_inode != NULL) {
		isp = smk_of_inode(new_dentry->d_inode);
		smk_ad_setfield_u_fs_path_dentry(&ad, new_dentry);
		rc = smk_curacc(isp, MAY_WRITE, &ad);
	}

	return rc;
}

static int smack_inode_unlink(struct inode *dir, struct dentry *dentry)
{
	struct inode *ip = dentry->d_inode;
	struct smk_audit_info ad;
	int rc;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);

	/*
	 * You need write access to the thing you're unlinking
	 */
	rc = smk_curacc(smk_of_inode(ip), MAY_WRITE, &ad);
	if (rc == 0) {
		/*
		 * You also need write access to the containing directory
		 */
		smk_ad_setfield_u_fs_path_dentry(&ad, NULL);
		smk_ad_setfield_u_fs_inode(&ad, dir);
		rc = smk_curacc(smk_of_inode(dir), MAY_WRITE, &ad);
	}
	return rc;
}

static int smack_inode_rmdir(struct inode *dir, struct dentry *dentry)
{
	struct smk_audit_info ad;
	int rc;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);

	/*
	 * You need write access to the thing you're removing
	 */
	rc = smk_curacc(smk_of_inode(dentry->d_inode), MAY_WRITE, &ad);
	if (rc == 0) {
		/*
		 * You also need write access to the containing directory
		 */
		smk_ad_setfield_u_fs_path_dentry(&ad, NULL);
		smk_ad_setfield_u_fs_inode(&ad, dir);
		rc = smk_curacc(smk_of_inode(dir), MAY_WRITE, &ad);
	}

	return rc;
}

static int smack_inode_rename(struct inode *old_inode,
			      struct dentry *old_dentry,
			      struct inode *new_inode,
			      struct dentry *new_dentry)
{
	int rc;
	char *isp;
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, old_dentry);

	isp = smk_of_inode(old_dentry->d_inode);
	rc = smk_curacc(isp, MAY_READWRITE, &ad);

	if (rc == 0 && new_dentry->d_inode != NULL) {
		isp = smk_of_inode(new_dentry->d_inode);
		smk_ad_setfield_u_fs_path_dentry(&ad, new_dentry);
		rc = smk_curacc(isp, MAY_READWRITE, &ad);
	}
	return rc;
}

static int smack_inode_permission(struct inode *inode, int mask)
{
	struct smk_audit_info ad;
	/*
	 * No permission to check. Existence test. Yup, it's there.
	 */
	if (mask == 0)
		return 0;
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_inode(&ad, inode);
	return smk_curacc(smk_of_inode(inode), mask, &ad);
}

static int smack_inode_setattr(struct dentry *dentry, struct iattr *iattr)
{
	struct smk_audit_info ad;
	/*
	 * Need to allow for clearing the setuid bit.
	 */
	if (iattr->ia_valid & ATTR_FORCE)
		return 0;
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);

	return smk_curacc(smk_of_inode(dentry->d_inode), MAY_WRITE, &ad);
}

static int smack_inode_getattr(struct vfsmount *mnt, struct dentry *dentry)
{
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);
	smk_ad_setfield_u_fs_path_mnt(&ad, mnt);
	return smk_curacc(smk_of_inode(dentry->d_inode), MAY_READ, &ad);
}

static int smack_inode_setxattr(struct dentry *dentry, const char *name,
				const void *value, size_t size, int flags)
{
	struct smk_audit_info ad;
	int rc = 0;

	if (strcmp(name, XATTR_NAME_SMACK) == 0 ||
	    strcmp(name, XATTR_NAME_SMACKIPIN) == 0 ||
	    strcmp(name, XATTR_NAME_SMACKIPOUT) == 0) {
		if (!capable(CAP_MAC_ADMIN))
			rc = -EPERM;
		/*
		 * check label validity here so import wont fail on
		 * post_setxattr
		 */
		if (size == 0 || size >= SMK_LABELLEN ||
		    smk_import(value, size) == NULL)
			rc = -EINVAL;
	} else
		rc = cap_inode_setxattr(dentry, name, value, size, flags);

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);

	if (rc == 0)
		rc = smk_curacc(smk_of_inode(dentry->d_inode), MAY_WRITE, &ad);

	return rc;
}

static void smack_inode_post_setxattr(struct dentry *dentry, const char *name,
				      const void *value, size_t size, int flags)
{
	struct inode_smack *isp;
	char *nsp;

	/*
	 * Not SMACK
	 */
	if (strcmp(name, XATTR_NAME_SMACK))
		return;

	isp = dentry->d_inode->i_security;

	/*
	 * No locking is done here. This is a pointer
	 * assignment.
	 */
	nsp = smk_import(value, size);
	if (nsp != NULL)
		isp->smk_inode = nsp;
	else
		isp->smk_inode = smack_known_invalid.smk_known;

	return;
}

static int smack_inode_getxattr(struct dentry *dentry, const char *name)
{
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);

	return smk_curacc(smk_of_inode(dentry->d_inode), MAY_READ, &ad);
}

static int smack_inode_removexattr(struct dentry *dentry, const char *name)
{
	struct smk_audit_info ad;
	int rc = 0;

	if (strcmp(name, XATTR_NAME_SMACK) == 0 ||
	    strcmp(name, XATTR_NAME_SMACKIPIN) == 0 ||
	    strcmp(name, XATTR_NAME_SMACKIPOUT) == 0) {
		if (!capable(CAP_MAC_ADMIN))
			rc = -EPERM;
	} else
		rc = cap_inode_removexattr(dentry, name);

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, dentry);
	if (rc == 0)
		rc = smk_curacc(smk_of_inode(dentry->d_inode), MAY_WRITE, &ad);

	return rc;
}

static int smack_inode_getsecurity(const struct inode *inode,
				   const char *name, void **buffer,
				   bool alloc)
{
	struct socket_smack *ssp;
	struct socket *sock;
	struct super_block *sbp;
	struct inode *ip = (struct inode *)inode;
	char *isp;
	int ilen;
	int rc = 0;

	if (strcmp(name, XATTR_SMACK_SUFFIX) == 0) {
		isp = smk_of_inode(inode);
		ilen = strlen(isp) + 1;
		*buffer = isp;
		return ilen;
	}

	/*
	 * The rest of the Smack xattrs are only on sockets.
	 */
	sbp = ip->i_sb;
	if (sbp->s_magic != SOCKFS_MAGIC)
		return -EOPNOTSUPP;

	sock = SOCKET_I(ip);
	if (sock == NULL || sock->sk == NULL)
		return -EOPNOTSUPP;

	ssp = sock->sk->sk_security;

	if (strcmp(name, XATTR_SMACK_IPIN) == 0)
		isp = ssp->smk_in;
	else if (strcmp(name, XATTR_SMACK_IPOUT) == 0)
		isp = ssp->smk_out;
	else
		return -EOPNOTSUPP;

	ilen = strlen(isp) + 1;
	if (rc == 0) {
		*buffer = isp;
		rc = ilen;
	}

	return rc;
}


static int smack_inode_listsecurity(struct inode *inode, char *buffer,
				    size_t buffer_size)
{
	int len = strlen(XATTR_NAME_SMACK);

	if (buffer != NULL && len <= buffer_size) {
		memcpy(buffer, XATTR_NAME_SMACK, len);
		return len;
	}
	return -EINVAL;
}

static void smack_inode_getsecid(const struct inode *inode, u32 *secid)
{
	struct inode_smack *isp = inode->i_security;

	*secid = smack_to_secid(isp->smk_inode);
}


static int smack_file_permission(struct file *file, int mask)
{
	return 0;
}

static int smack_file_alloc_security(struct file *file)
{
	file->f_security = current_security();
	return 0;
}

static void smack_file_free_security(struct file *file)
{
	file->f_security = NULL;
}

static int smack_file_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	int rc = 0;
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path(&ad, file->f_path);

	if (_IOC_DIR(cmd) & _IOC_WRITE)
		rc = smk_curacc(file->f_security, MAY_WRITE, &ad);

	if (rc == 0 && (_IOC_DIR(cmd) & _IOC_READ))
		rc = smk_curacc(file->f_security, MAY_READ, &ad);

	return rc;
}

static int smack_file_lock(struct file *file, unsigned int cmd)
{
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path_dentry(&ad, file->f_path.dentry);
	return smk_curacc(file->f_security, MAY_WRITE, &ad);
}

static int smack_file_fcntl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct smk_audit_info ad;
	int rc;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_FS);
	smk_ad_setfield_u_fs_path(&ad, file->f_path);

	switch (cmd) {
	case F_DUPFD:
	case F_GETFD:
	case F_GETFL:
	case F_GETLK:
	case F_GETOWN:
	case F_GETSIG:
		rc = smk_curacc(file->f_security, MAY_READ, &ad);
		break;
	case F_SETFD:
	case F_SETFL:
	case F_SETLK:
	case F_SETLKW:
	case F_SETOWN:
	case F_SETSIG:
		rc = smk_curacc(file->f_security, MAY_WRITE, &ad);
		break;
	default:
		rc = smk_curacc(file->f_security, MAY_READWRITE, &ad);
	}

	return rc;
}

static int smack_file_set_fowner(struct file *file)
{
	file->f_security = current_security();
	return 0;
}

static int smack_file_send_sigiotask(struct task_struct *tsk,
				     struct fown_struct *fown, int signum)
{
	struct file *file;
	int rc;
	char *tsp = tsk->cred->security;
	struct smk_audit_info ad;

	/*
	 * struct fown_struct is never outside the context of a struct file
	 */
	file = container_of(fown, struct file, f_owner);
	/* we don't log here as rc can be overriden */
	rc = smk_access(file->f_security, tsp, MAY_WRITE, NULL);
	if (rc != 0 && has_capability(tsk, CAP_MAC_OVERRIDE))
		rc = 0;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_tsk(&ad, tsk);
	smack_log(file->f_security, tsp, MAY_WRITE, rc, &ad);
	return rc;
}

static int smack_file_receive(struct file *file)
{
	int may = 0;
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_fs_path(&ad, file->f_path);
	/*
	 * This code relies on bitmasks.
	 */
	if (file->f_mode & FMODE_READ)
		may = MAY_READ;
	if (file->f_mode & FMODE_WRITE)
		may |= MAY_WRITE;

	return smk_curacc(file->f_security, may, &ad);
}


static int smack_cred_alloc_blank(struct cred *cred, gfp_t gfp)
{
	cred->security = NULL;
	return 0;
}


static void smack_cred_free(struct cred *cred)
{
	cred->security = NULL;
}

static int smack_cred_prepare(struct cred *new, const struct cred *old,
			      gfp_t gfp)
{
	new->security = old->security;
	return 0;
}

static void smack_cred_transfer(struct cred *new, const struct cred *old)
{
	new->security = old->security;
}

static int smack_kernel_act_as(struct cred *new, u32 secid)
{
	char *smack = smack_from_secid(secid);

	if (smack == NULL)
		return -EINVAL;

	new->security = smack;
	return 0;
}

static int smack_kernel_create_files_as(struct cred *new,
					struct inode *inode)
{
	struct inode_smack *isp = inode->i_security;

	new->security = isp->smk_inode;
	return 0;
}

static int smk_curacc_on_task(struct task_struct *p, int access)
{
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_tsk(&ad, p);
	return smk_curacc(task_security(p), access, &ad);
}

static int smack_task_setpgid(struct task_struct *p, pid_t pgid)
{
	return smk_curacc_on_task(p, MAY_WRITE);
}

static int smack_task_getpgid(struct task_struct *p)
{
	return smk_curacc_on_task(p, MAY_READ);
}

static int smack_task_getsid(struct task_struct *p)
{
	return smk_curacc_on_task(p, MAY_READ);
}

static void smack_task_getsecid(struct task_struct *p, u32 *secid)
{
	*secid = smack_to_secid(task_security(p));
}

static int smack_task_setnice(struct task_struct *p, int nice)
{
	int rc;

	rc = cap_task_setnice(p, nice);
	if (rc == 0)
		rc = smk_curacc_on_task(p, MAY_WRITE);
	return rc;
}

static int smack_task_setioprio(struct task_struct *p, int ioprio)
{
	int rc;

	rc = cap_task_setioprio(p, ioprio);
	if (rc == 0)
		rc = smk_curacc_on_task(p, MAY_WRITE);
	return rc;
}

static int smack_task_getioprio(struct task_struct *p)
{
	return smk_curacc_on_task(p, MAY_READ);
}

static int smack_task_setscheduler(struct task_struct *p, int policy,
				   struct sched_param *lp)
{
	int rc;

	rc = cap_task_setscheduler(p, policy, lp);
	if (rc == 0)
		rc = smk_curacc_on_task(p, MAY_WRITE);
	return rc;
}

static int smack_task_getscheduler(struct task_struct *p)
{
	return smk_curacc_on_task(p, MAY_READ);
}

static int smack_task_movememory(struct task_struct *p)
{
	return smk_curacc_on_task(p, MAY_WRITE);
}

static int smack_task_kill(struct task_struct *p, struct siginfo *info,
			   int sig, u32 secid)
{
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_tsk(&ad, p);
	/*
	 * Sending a signal requires that the sender
	 * can write the receiver.
	 */
	if (secid == 0)
		return smk_curacc(task_security(p), MAY_WRITE, &ad);
	/*
	 * If the secid isn't 0 we're dealing with some USB IO
	 * specific behavior. This is not clean. For one thing
	 * we can't take privilege into account.
	 */
	return smk_access(smack_from_secid(secid), task_security(p),
			  MAY_WRITE, &ad);
}

static int smack_task_wait(struct task_struct *p)
{
	struct smk_audit_info ad;
	char *sp = current_security();
	char *tsp = task_security(p);
	int rc;

	/* we don't log here, we can be overriden */
	rc = smk_access(sp, tsp, MAY_WRITE, NULL);
	if (rc == 0)
		goto out_log;

	/*
	 * Allow the operation to succeed if either task
	 * has privilege to perform operations that might
	 * account for the smack labels having gotten to
	 * be different in the first place.
	 *
	 * This breaks the strict subject/object access
	 * control ideal, taking the object's privilege
	 * state into account in the decision as well as
	 * the smack value.
	 */
	if (capable(CAP_MAC_OVERRIDE) || has_capability(p, CAP_MAC_OVERRIDE))
		rc = 0;
	/* we log only if we didn't get overriden */
 out_log:
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_TASK);
	smk_ad_setfield_u_tsk(&ad, p);
	smack_log(sp, tsp, MAY_WRITE, rc, &ad);
	return rc;
}

static void smack_task_to_inode(struct task_struct *p, struct inode *inode)
{
	struct inode_smack *isp = inode->i_security;
	isp->smk_inode = task_security(p);
}


static int smack_sk_alloc_security(struct sock *sk, int family, gfp_t gfp_flags)
{
	char *csp = current_security();
	struct socket_smack *ssp;

	ssp = kzalloc(sizeof(struct socket_smack), gfp_flags);
	if (ssp == NULL)
		return -ENOMEM;

	ssp->smk_in = csp;
	ssp->smk_out = csp;
	ssp->smk_packet[0] = '\0';

	sk->sk_security = ssp;

	return 0;
}

static void smack_sk_free_security(struct sock *sk)
{
	kfree(sk->sk_security);
}

static char *smack_host_label(struct sockaddr_in *sip)
{
	struct smk_netlbladdr *snp;
	struct in_addr *siap = &sip->sin_addr;

	if (siap->s_addr == 0)
		return NULL;

	list_for_each_entry_rcu(snp, &smk_netlbladdr_list, list)
		/*
		* we break after finding the first match because
		* the list is sorted from longest to shortest mask
		* so we have found the most specific match
		*/
		if ((&snp->smk_host.sin_addr)->s_addr ==
		    (siap->s_addr & (&snp->smk_mask)->s_addr)) {
			/* we have found the special CIPSO option */
			if (snp->smk_label == smack_cipso_option)
				return NULL;
			return snp->smk_label;
		}

	return NULL;
}

static void smack_set_catset(char *catset, struct netlbl_lsm_secattr *sap)
{
	unsigned char *cp;
	unsigned char m;
	int cat;
	int rc;
	int byte;

	if (!catset)
		return;

	sap->flags |= NETLBL_SECATTR_MLS_CAT;
	sap->attr.mls.cat = netlbl_secattr_catmap_alloc(GFP_ATOMIC);
	sap->attr.mls.cat->startbit = 0;

	for (cat = 1, cp = catset, byte = 0; byte < SMK_LABELLEN; cp++, byte++)
		for (m = 0x80; m != 0; m >>= 1, cat++) {
			if ((m & *cp) == 0)
				continue;
			rc = netlbl_secattr_catmap_setbit(sap->attr.mls.cat,
							  cat, GFP_ATOMIC);
		}
}

static void smack_to_secattr(char *smack, struct netlbl_lsm_secattr *nlsp)
{
	struct smack_cipso cipso;
	int rc;

	nlsp->domain = smack;
	nlsp->flags = NETLBL_SECATTR_DOMAIN | NETLBL_SECATTR_MLS_LVL;

	rc = smack_to_cipso(smack, &cipso);
	if (rc == 0) {
		nlsp->attr.mls.lvl = cipso.smk_level;
		smack_set_catset(cipso.smk_catset, nlsp);
	} else {
		nlsp->attr.mls.lvl = smack_cipso_direct;
		smack_set_catset(smack, nlsp);
	}
}

static int smack_netlabel(struct sock *sk, int labeled)
{
	struct socket_smack *ssp = sk->sk_security;
	struct netlbl_lsm_secattr secattr;
	int rc = 0;

	/*
	 * Usually the netlabel code will handle changing the
	 * packet labeling based on the label.
	 * The case of a single label host is different, because
	 * a single label host should never get a labeled packet
	 * even though the label is usually associated with a packet
	 * label.
	 */
	local_bh_disable();
	bh_lock_sock_nested(sk);

	if (ssp->smk_out == smack_net_ambient ||
	    labeled == SMACK_UNLABELED_SOCKET)
		netlbl_sock_delattr(sk);
	else {
		netlbl_secattr_init(&secattr);
		smack_to_secattr(ssp->smk_out, &secattr);
		rc = netlbl_sock_setattr(sk, sk->sk_family, &secattr);
		netlbl_secattr_destroy(&secattr);
	}

	bh_unlock_sock(sk);
	local_bh_enable();

	return rc;
}

static int smack_netlabel_send(struct sock *sk, struct sockaddr_in *sap)
{
	int rc;
	int sk_lbl;
	char *hostsp;
	struct socket_smack *ssp = sk->sk_security;
	struct smk_audit_info ad;

	rcu_read_lock();
	hostsp = smack_host_label(sap);
	if (hostsp != NULL) {
		sk_lbl = SMACK_UNLABELED_SOCKET;
#ifdef CONFIG_AUDIT
		smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_NET);
		ad.a.u.net.family = sap->sin_family;
		ad.a.u.net.dport = sap->sin_port;
		ad.a.u.net.v4info.daddr = sap->sin_addr.s_addr;
#endif
		rc = smk_access(ssp->smk_out, hostsp, MAY_WRITE, &ad);
	} else {
		sk_lbl = SMACK_CIPSO_SOCKET;
		rc = 0;
	}
	rcu_read_unlock();
	if (rc != 0)
		return rc;

	return smack_netlabel(sk, sk_lbl);
}

static int smack_inode_setsecurity(struct inode *inode, const char *name,
				   const void *value, size_t size, int flags)
{
	char *sp;
	struct inode_smack *nsp = inode->i_security;
	struct socket_smack *ssp;
	struct socket *sock;
	int rc = 0;

	if (value == NULL || size > SMK_LABELLEN || size == 0)
		return -EACCES;

	sp = smk_import(value, size);
	if (sp == NULL)
		return -EINVAL;

	if (strcmp(name, XATTR_SMACK_SUFFIX) == 0) {
		nsp->smk_inode = sp;
		nsp->smk_flags |= SMK_INODE_INSTANT;
		return 0;
	}
	/*
	 * The rest of the Smack xattrs are only on sockets.
	 */
	if (inode->i_sb->s_magic != SOCKFS_MAGIC)
		return -EOPNOTSUPP;

	sock = SOCKET_I(inode);
	if (sock == NULL || sock->sk == NULL)
		return -EOPNOTSUPP;

	ssp = sock->sk->sk_security;

	if (strcmp(name, XATTR_SMACK_IPIN) == 0)
		ssp->smk_in = sp;
	else if (strcmp(name, XATTR_SMACK_IPOUT) == 0) {
		ssp->smk_out = sp;
		rc = smack_netlabel(sock->sk, SMACK_CIPSO_SOCKET);
		if (rc != 0)
			printk(KERN_WARNING "Smack: \"%s\" netlbl error %d.\n",
			       __func__, -rc);
	} else
		return -EOPNOTSUPP;

	return 0;
}

static int smack_socket_post_create(struct socket *sock, int family,
				    int type, int protocol, int kern)
{
	if (family != PF_INET || sock->sk == NULL)
		return 0;
	/*
	 * Set the outbound netlbl.
	 */
	return smack_netlabel(sock->sk, SMACK_CIPSO_SOCKET);
}

static int smack_socket_connect(struct socket *sock, struct sockaddr *sap,
				int addrlen)
{
	if (sock->sk == NULL || sock->sk->sk_family != PF_INET)
		return 0;
	if (addrlen < sizeof(struct sockaddr_in))
		return -EINVAL;

	return smack_netlabel_send(sock->sk, (struct sockaddr_in *)sap);
}

static int smack_flags_to_may(int flags)
{
	int may = 0;

	if (flags & S_IRUGO)
		may |= MAY_READ;
	if (flags & S_IWUGO)
		may |= MAY_WRITE;
	if (flags & S_IXUGO)
		may |= MAY_EXEC;

	return may;
}

static int smack_msg_msg_alloc_security(struct msg_msg *msg)
{
	msg->security = current_security();
	return 0;
}

static void smack_msg_msg_free_security(struct msg_msg *msg)
{
	msg->security = NULL;
}

static char *smack_of_shm(struct shmid_kernel *shp)
{
	return (char *)shp->shm_perm.security;
}

static int smack_shm_alloc_security(struct shmid_kernel *shp)
{
	struct kern_ipc_perm *isp = &shp->shm_perm;

	isp->security = current_security();
	return 0;
}

static void smack_shm_free_security(struct shmid_kernel *shp)
{
	struct kern_ipc_perm *isp = &shp->shm_perm;

	isp->security = NULL;
}

static int smk_curacc_shm(struct shmid_kernel *shp, int access)
{
	char *ssp = smack_of_shm(shp);
	struct smk_audit_info ad;

#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_IPC);
	ad.a.u.ipc_id = shp->shm_perm.id;
#endif
	return smk_curacc(ssp, access, &ad);
}

static int smack_shm_associate(struct shmid_kernel *shp, int shmflg)
{
	int may;

	may = smack_flags_to_may(shmflg);
	return smk_curacc_shm(shp, may);
}

static int smack_shm_shmctl(struct shmid_kernel *shp, int cmd)
{
	int may;

	switch (cmd) {
	case IPC_STAT:
	case SHM_STAT:
		may = MAY_READ;
		break;
	case IPC_SET:
	case SHM_LOCK:
	case SHM_UNLOCK:
	case IPC_RMID:
		may = MAY_READWRITE;
		break;
	case IPC_INFO:
	case SHM_INFO:
		/*
		 * System level information.
		 */
		return 0;
	default:
		return -EINVAL;
	}
	return smk_curacc_shm(shp, may);
}

static int smack_shm_shmat(struct shmid_kernel *shp, char __user *shmaddr,
			   int shmflg)
{
	int may;

	may = smack_flags_to_may(shmflg);
	return smk_curacc_shm(shp, may);
}

static char *smack_of_sem(struct sem_array *sma)
{
	return (char *)sma->sem_perm.security;
}

static int smack_sem_alloc_security(struct sem_array *sma)
{
	struct kern_ipc_perm *isp = &sma->sem_perm;

	isp->security = current_security();
	return 0;
}

static void smack_sem_free_security(struct sem_array *sma)
{
	struct kern_ipc_perm *isp = &sma->sem_perm;

	isp->security = NULL;
}

static int smk_curacc_sem(struct sem_array *sma, int access)
{
	char *ssp = smack_of_sem(sma);
	struct smk_audit_info ad;

#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_IPC);
	ad.a.u.ipc_id = sma->sem_perm.id;
#endif
	return smk_curacc(ssp, access, &ad);
}

static int smack_sem_associate(struct sem_array *sma, int semflg)
{
	int may;

	may = smack_flags_to_may(semflg);
	return smk_curacc_sem(sma, may);
}

static int smack_sem_semctl(struct sem_array *sma, int cmd)
{
	int may;

	switch (cmd) {
	case GETPID:
	case GETNCNT:
	case GETZCNT:
	case GETVAL:
	case GETALL:
	case IPC_STAT:
	case SEM_STAT:
		may = MAY_READ;
		break;
	case SETVAL:
	case SETALL:
	case IPC_RMID:
	case IPC_SET:
		may = MAY_READWRITE;
		break;
	case IPC_INFO:
	case SEM_INFO:
		/*
		 * System level information
		 */
		return 0;
	default:
		return -EINVAL;
	}

	return smk_curacc_sem(sma, may);
}

static int smack_sem_semop(struct sem_array *sma, struct sembuf *sops,
			   unsigned nsops, int alter)
{
	return smk_curacc_sem(sma, MAY_READWRITE);
}

static int smack_msg_queue_alloc_security(struct msg_queue *msq)
{
	struct kern_ipc_perm *kisp = &msq->q_perm;

	kisp->security = current_security();
	return 0;
}

static void smack_msg_queue_free_security(struct msg_queue *msq)
{
	struct kern_ipc_perm *kisp = &msq->q_perm;

	kisp->security = NULL;
}

static char *smack_of_msq(struct msg_queue *msq)
{
	return (char *)msq->q_perm.security;
}

static int smk_curacc_msq(struct msg_queue *msq, int access)
{
	char *msp = smack_of_msq(msq);
	struct smk_audit_info ad;

#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_IPC);
	ad.a.u.ipc_id = msq->q_perm.id;
#endif
	return smk_curacc(msp, access, &ad);
}

static int smack_msg_queue_associate(struct msg_queue *msq, int msqflg)
{
	int may;

	may = smack_flags_to_may(msqflg);
	return smk_curacc_msq(msq, may);
}

static int smack_msg_queue_msgctl(struct msg_queue *msq, int cmd)
{
	int may;

	switch (cmd) {
	case IPC_STAT:
	case MSG_STAT:
		may = MAY_READ;
		break;
	case IPC_SET:
	case IPC_RMID:
		may = MAY_READWRITE;
		break;
	case IPC_INFO:
	case MSG_INFO:
		/*
		 * System level information
		 */
		return 0;
	default:
		return -EINVAL;
	}

	return smk_curacc_msq(msq, may);
}

static int smack_msg_queue_msgsnd(struct msg_queue *msq, struct msg_msg *msg,
				  int msqflg)
{
	int may;

	may = smack_flags_to_may(msqflg);
	return smk_curacc_msq(msq, may);
}

static int smack_msg_queue_msgrcv(struct msg_queue *msq, struct msg_msg *msg,
			struct task_struct *target, long type, int mode)
{
	return smk_curacc_msq(msq, MAY_READWRITE);
}

static int smack_ipc_permission(struct kern_ipc_perm *ipp, short flag)
{
	char *isp = ipp->security;
	int may = smack_flags_to_may(flag);
	struct smk_audit_info ad;

#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_IPC);
	ad.a.u.ipc_id = ipp->id;
#endif
	return smk_curacc(isp, may, &ad);
}

static void smack_ipc_getsecid(struct kern_ipc_perm *ipp, u32 *secid)
{
	char *smack = ipp->security;

	*secid = smack_to_secid(smack);
}

static void smack_d_instantiate(struct dentry *opt_dentry, struct inode *inode)
{
	struct super_block *sbp;
	struct superblock_smack *sbsp;
	struct inode_smack *isp;
	char *csp = current_security();
	char *fetched;
	char *final;
	struct dentry *dp;

	if (inode == NULL)
		return;

	isp = inode->i_security;

	mutex_lock(&isp->smk_lock);
	/*
	 * If the inode is already instantiated
	 * take the quick way out
	 */
	if (isp->smk_flags & SMK_INODE_INSTANT)
		goto unlockandout;

	sbp = inode->i_sb;
	sbsp = sbp->s_security;
	/*
	 * We're going to use the superblock default label
	 * if there's no label on the file.
	 */
	final = sbsp->smk_default;

	/*
	 * If this is the root inode the superblock
	 * may be in the process of initialization.
	 * If that is the case use the root value out
	 * of the superblock.
	 */
	if (opt_dentry->d_parent == opt_dentry) {
		isp->smk_inode = sbsp->smk_root;
		isp->smk_flags |= SMK_INODE_INSTANT;
		goto unlockandout;
	}

	/*
	 * This is pretty hackish.
	 * Casey says that we shouldn't have to do
	 * file system specific code, but it does help
	 * with keeping it simple.
	 */
	switch (sbp->s_magic) {
	case SMACK_MAGIC:
		/*
		 * Casey says that it's a little embarassing
		 * that the smack file system doesn't do
		 * extended attributes.
		 */
		final = smack_known_star.smk_known;
		break;
	case PIPEFS_MAGIC:
		/*
		 * Casey says pipes are easy (?)
		 */
		final = smack_known_star.smk_known;
		break;
	case DEVPTS_SUPER_MAGIC:
		/*
		 * devpts seems content with the label of the task.
		 * Programs that change smack have to treat the
		 * pty with respect.
		 */
		final = csp;
		break;
	case SOCKFS_MAGIC:
		/*
		 * Casey says sockets get the smack of the task.
		 */
		final = csp;
		break;
	case PROC_SUPER_MAGIC:
		/*
		 * Casey says procfs appears not to care.
		 * The superblock default suffices.
		 */
		break;
	case TMPFS_MAGIC:
		/*
		 * Device labels should come from the filesystem,
		 * but watch out, because they're volitile,
		 * getting recreated on every reboot.
		 */
		final = smack_known_star.smk_known;
		/*
		 * No break.
		 *
		 * If a smack value has been set we want to use it,
		 * but since tmpfs isn't giving us the opportunity
		 * to set mount options simulate setting the
		 * superblock default.
		 */
	default:
		/*
		 * This isn't an understood special case.
		 * Get the value from the xattr.
		 *
		 * No xattr support means, alas, no SMACK label.
		 * Use the aforeapplied default.
		 * It would be curious if the label of the task
		 * does not match that assigned.
		 */
		if (inode->i_op->getxattr == NULL)
			break;
		/*
		 * Get the dentry for xattr.
		 */
		if (opt_dentry == NULL) {
			dp = d_find_alias(inode);
			if (dp == NULL)
				break;
		} else {
			dp = dget(opt_dentry);
			if (dp == NULL)
				break;
		}

		fetched = smk_fetch(inode, dp);
		if (fetched != NULL)
			final = fetched;

		dput(dp);
		break;
	}

	if (final == NULL)
		isp->smk_inode = csp;
	else
		isp->smk_inode = final;

	isp->smk_flags |= SMK_INODE_INSTANT;

unlockandout:
	mutex_unlock(&isp->smk_lock);
	return;
}

static int smack_getprocattr(struct task_struct *p, char *name, char **value)
{
	char *cp;
	int slen;

	if (strcmp(name, "current") != 0)
		return -EINVAL;

	cp = kstrdup(task_security(p), GFP_KERNEL);
	if (cp == NULL)
		return -ENOMEM;

	slen = strlen(cp);
	*value = cp;
	return slen;
}

static int smack_setprocattr(struct task_struct *p, char *name,
			     void *value, size_t size)
{
	struct cred *new;
	char *newsmack;

	/*
	 * Changing another process' Smack value is too dangerous
	 * and supports no sane use case.
	 */
	if (p != current)
		return -EPERM;

	if (!capable(CAP_MAC_ADMIN))
		return -EPERM;

	if (value == NULL || size == 0 || size >= SMK_LABELLEN)
		return -EINVAL;

	if (strcmp(name, "current") != 0)
		return -EINVAL;

	newsmack = smk_import(value, size);
	if (newsmack == NULL)
		return -EINVAL;

	/*
	 * No process is ever allowed the web ("@") label.
	 */
	if (newsmack == smack_known_web.smk_known)
		return -EPERM;

	new = prepare_creds();
	if (new == NULL)
		return -ENOMEM;
	new->security = newsmack;
	commit_creds(new);
	return size;
}

static int smack_unix_stream_connect(struct socket *sock,
				     struct socket *other, struct sock *newsk)
{
	struct inode *sp = SOCK_INODE(sock);
	struct inode *op = SOCK_INODE(other);
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_NET);
	smk_ad_setfield_u_net_sk(&ad, other->sk);
	return smk_access(smk_of_inode(sp), smk_of_inode(op),
				 MAY_READWRITE, &ad);
}

static int smack_unix_may_send(struct socket *sock, struct socket *other)
{
	struct inode *sp = SOCK_INODE(sock);
	struct inode *op = SOCK_INODE(other);
	struct smk_audit_info ad;

	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_NET);
	smk_ad_setfield_u_net_sk(&ad, other->sk);
	return smk_access(smk_of_inode(sp), smk_of_inode(op), MAY_WRITE, &ad);
}

static int smack_socket_sendmsg(struct socket *sock, struct msghdr *msg,
				int size)
{
	struct sockaddr_in *sip = (struct sockaddr_in *) msg->msg_name;

	/*
	 * Perfectly reasonable for this to be NULL
	 */
	if (sip == NULL || sip->sin_family != AF_INET)
		return 0;

	return smack_netlabel_send(sock->sk, sip);
}


static void smack_from_secattr(struct netlbl_lsm_secattr *sap, char *sip)
{
	char smack[SMK_LABELLEN];
	char *sp;
	int pcat;

	if ((sap->flags & NETLBL_SECATTR_MLS_LVL) != 0) {
		/*
		 * Looks like a CIPSO packet.
		 * If there are flags but no level netlabel isn't
		 * behaving the way we expect it to.
		 *
		 * Get the categories, if any
		 * Without guidance regarding the smack value
		 * for the packet fall back on the network
		 * ambient value.
		 */
		memset(smack, '\0', SMK_LABELLEN);
		if ((sap->flags & NETLBL_SECATTR_MLS_CAT) != 0)
			for (pcat = -1;;) {
				pcat = netlbl_secattr_catmap_walk(
					sap->attr.mls.cat, pcat + 1);
				if (pcat < 0)
					break;
				smack_catset_bit(pcat, smack);
			}
		/*
		 * If it is CIPSO using smack direct mapping
		 * we are already done. WeeHee.
		 */
		if (sap->attr.mls.lvl == smack_cipso_direct) {
			memcpy(sip, smack, SMK_MAXLEN);
			return;
		}
		/*
		 * Look it up in the supplied table if it is not
		 * a direct mapping.
		 */
		smack_from_cipso(sap->attr.mls.lvl, smack, sip);
		return;
	}
	if ((sap->flags & NETLBL_SECATTR_SECID) != 0) {
		/*
		 * Looks like a fallback, which gives us a secid.
		 */
		sp = smack_from_secid(sap->attr.secid);
		/*
		 * This has got to be a bug because it is
		 * impossible to specify a fallback without
		 * specifying the label, which will ensure
		 * it has a secid, and the only way to get a
		 * secid is from a fallback.
		 */
		BUG_ON(sp == NULL);
		strncpy(sip, sp, SMK_MAXLEN);
		return;
	}
	/*
	 * Without guidance regarding the smack value
	 * for the packet fall back on the network
	 * ambient value.
	 */
	strncpy(sip, smack_net_ambient, SMK_MAXLEN);
	return;
}

static int smack_socket_sock_rcv_skb(struct sock *sk, struct sk_buff *skb)
{
	struct netlbl_lsm_secattr secattr;
	struct socket_smack *ssp = sk->sk_security;
	char smack[SMK_LABELLEN];
	char *csp;
	int rc;
	struct smk_audit_info ad;
	if (sk->sk_family != PF_INET && sk->sk_family != PF_INET6)
		return 0;

	/*
	 * Translate what netlabel gave us.
	 */
	netlbl_secattr_init(&secattr);

	rc = netlbl_skbuff_getattr(skb, sk->sk_family, &secattr);
	if (rc == 0) {
		smack_from_secattr(&secattr, smack);
		csp = smack;
	} else
		csp = smack_net_ambient;

	netlbl_secattr_destroy(&secattr);

#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_NET);
	ad.a.u.net.family = sk->sk_family;
	ad.a.u.net.netif = skb->skb_iif;
	ipv4_skb_to_auditdata(skb, &ad.a, NULL);
#endif
	/*
	 * Receiving a packet requires that the other end
	 * be able to write here. Read access is not required.
	 * This is the simplist possible security model
	 * for networking.
	 */
	rc = smk_access(csp, ssp->smk_in, MAY_WRITE, &ad);
	if (rc != 0)
		netlbl_skbuff_err(skb, rc, 0);
	return rc;
}

static int smack_socket_getpeersec_stream(struct socket *sock,
					  char __user *optval,
					  int __user *optlen, unsigned len)
{
	struct socket_smack *ssp;
	int slen;
	int rc = 0;

	ssp = sock->sk->sk_security;
	slen = strlen(ssp->smk_packet) + 1;

	if (slen > len)
		rc = -ERANGE;
	else if (copy_to_user(optval, ssp->smk_packet, slen) != 0)
		rc = -EFAULT;

	if (put_user(slen, optlen) != 0)
		rc = -EFAULT;

	return rc;
}


static int smack_socket_getpeersec_dgram(struct socket *sock,
					 struct sk_buff *skb, u32 *secid)

{
	struct netlbl_lsm_secattr secattr;
	struct sock *sk;
	char smack[SMK_LABELLEN];
	int family = PF_INET;
	u32 s;
	int rc;

	/*
	 * Only works for families with packets.
	 */
	if (sock != NULL) {
		sk = sock->sk;
		if (sk->sk_family != PF_INET && sk->sk_family != PF_INET6)
			return 0;
		family = sk->sk_family;
	}
	/*
	 * Translate what netlabel gave us.
	 */
	netlbl_secattr_init(&secattr);
	rc = netlbl_skbuff_getattr(skb, family, &secattr);
	if (rc == 0)
		smack_from_secattr(&secattr, smack);
	netlbl_secattr_destroy(&secattr);

	/*
	 * Give up if we couldn't get anything
	 */
	if (rc != 0)
		return rc;

	s = smack_to_secid(smack);
	if (s == 0)
		return -EINVAL;

	*secid = s;
	return 0;
}

static void smack_sock_graft(struct sock *sk, struct socket *parent)
{
	struct socket_smack *ssp;

	if (sk == NULL ||
	    (sk->sk_family != PF_INET && sk->sk_family != PF_INET6))
		return;

	ssp = sk->sk_security;
	ssp->smk_in = ssp->smk_out = current_security();
	/* cssp->smk_packet is already set in smack_inet_csk_clone() */
}

static int smack_inet_conn_request(struct sock *sk, struct sk_buff *skb,
				   struct request_sock *req)
{
	u16 family = sk->sk_family;
	struct socket_smack *ssp = sk->sk_security;
	struct netlbl_lsm_secattr secattr;
	struct sockaddr_in addr;
	struct iphdr *hdr;
	char smack[SMK_LABELLEN];
	int rc;
	struct smk_audit_info ad;

	/* handle mapped IPv4 packets arriving via IPv6 sockets */
	if (family == PF_INET6 && skb->protocol == htons(ETH_P_IP))
		family = PF_INET;

	netlbl_secattr_init(&secattr);
	rc = netlbl_skbuff_getattr(skb, family, &secattr);
	if (rc == 0)
		smack_from_secattr(&secattr, smack);
	else
		strncpy(smack, smack_known_huh.smk_known, SMK_MAXLEN);
	netlbl_secattr_destroy(&secattr);

#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_NET);
	ad.a.u.net.family = family;
	ad.a.u.net.netif = skb->skb_iif;
	ipv4_skb_to_auditdata(skb, &ad.a, NULL);
#endif
	/*
	 * Receiving a packet requires that the other end be able to write
	 * here. Read access is not required.
	 */
	rc = smk_access(smack, ssp->smk_in, MAY_WRITE, &ad);
	if (rc != 0)
		return rc;

	/*
	 * Save the peer's label in the request_sock so we can later setup
	 * smk_packet in the child socket so that SO_PEERCRED can report it.
	 */
	req->peer_secid = smack_to_secid(smack);

	/*
	 * We need to decide if we want to label the incoming connection here
	 * if we do we only need to label the request_sock and the stack will
	 * propogate the wire-label to the sock when it is created.
	 */
	hdr = ip_hdr(skb);
	addr.sin_addr.s_addr = hdr->saddr;
	rcu_read_lock();
	if (smack_host_label(&addr) == NULL) {
		rcu_read_unlock();
		netlbl_secattr_init(&secattr);
		smack_to_secattr(smack, &secattr);
		rc = netlbl_req_setattr(req, &secattr);
		netlbl_secattr_destroy(&secattr);
	} else {
		rcu_read_unlock();
		netlbl_req_delattr(req);
	}

	return rc;
}

static void smack_inet_csk_clone(struct sock *sk,
				 const struct request_sock *req)
{
	struct socket_smack *ssp = sk->sk_security;
	char *smack;

	if (req->peer_secid != 0) {
		smack = smack_from_secid(req->peer_secid);
		strncpy(ssp->smk_packet, smack, SMK_MAXLEN);
	} else
		ssp->smk_packet[0] = '\0';
}

#ifdef CONFIG_KEYS

static int smack_key_alloc(struct key *key, const struct cred *cred,
			   unsigned long flags)
{
	key->security = cred->security;
	return 0;
}

static void smack_key_free(struct key *key)
{
	key->security = NULL;
}

static int smack_key_permission(key_ref_t key_ref,
				const struct cred *cred, key_perm_t perm)
{
	struct key *keyp;
	struct smk_audit_info ad;

	keyp = key_ref_to_ptr(key_ref);
	if (keyp == NULL)
		return -EINVAL;
	/*
	 * If the key hasn't been initialized give it access so that
	 * it may do so.
	 */
	if (keyp->security == NULL)
		return 0;
	/*
	 * This should not occur
	 */
	if (cred->security == NULL)
		return -EACCES;
#ifdef CONFIG_AUDIT
	smk_ad_init(&ad, __func__, LSM_AUDIT_DATA_KEY);
	ad.a.u.key_struct.key = keyp->serial;
	ad.a.u.key_struct.key_desc = keyp->description;
#endif
	return smk_access(cred->security, keyp->security,
				 MAY_READWRITE, &ad);
}
#endif /* CONFIG_KEYS */

#ifdef CONFIG_AUDIT

static int smack_audit_rule_init(u32 field, u32 op, char *rulestr, void **vrule)
{
	char **rule = (char **)vrule;
	*rule = NULL;

	if (field != AUDIT_SUBJ_USER && field != AUDIT_OBJ_USER)
		return -EINVAL;

	if (op != Audit_equal && op != Audit_not_equal)
		return -EINVAL;

	*rule = smk_import(rulestr, 0);

	return 0;
}

static int smack_audit_rule_known(struct audit_krule *krule)
{
	struct audit_field *f;
	int i;

	for (i = 0; i < krule->field_count; i++) {
		f = &krule->fields[i];

		if (f->type == AUDIT_SUBJ_USER || f->type == AUDIT_OBJ_USER)
			return 1;
	}

	return 0;
}

static int smack_audit_rule_match(u32 secid, u32 field, u32 op, void *vrule,
				  struct audit_context *actx)
{
	char *smack;
	char *rule = vrule;

	if (!rule) {
		audit_log(actx, GFP_KERNEL, AUDIT_SELINUX_ERR,
			  "Smack: missing rule\n");
		return -ENOENT;
	}

	if (field != AUDIT_SUBJ_USER && field != AUDIT_OBJ_USER)
		return 0;

	smack = smack_from_secid(secid);

	/*
	 * No need to do string comparisons. If a match occurs,
	 * both pointers will point to the same smack_known
	 * label.
	 */
	if (op == Audit_equal)
		return (rule == smack);
	if (op == Audit_not_equal)
		return (rule != smack);

	return 0;
}

static void smack_audit_rule_free(void *vrule)
{
	/* No-op */
}

#endif /* CONFIG_AUDIT */

static int smack_secid_to_secctx(u32 secid, char **secdata, u32 *seclen)
{
	char *sp = smack_from_secid(secid);

	*secdata = sp;
	*seclen = strlen(sp);
	return 0;
}

static int smack_secctx_to_secid(const char *secdata, u32 seclen, u32 *secid)
{
	*secid = smack_to_secid(secdata);
	return 0;
}

static void smack_release_secctx(char *secdata, u32 seclen)
{
}

static int smack_inode_notifysecctx(struct inode *inode, void *ctx, u32 ctxlen)
{
	return smack_inode_setsecurity(inode, XATTR_SMACK_SUFFIX, ctx, ctxlen, 0);
}

static int smack_inode_setsecctx(struct dentry *dentry, void *ctx, u32 ctxlen)
{
	return __vfs_setxattr_noperm(dentry, XATTR_NAME_SMACK, ctx, ctxlen, 0);
}

static int smack_inode_getsecctx(struct inode *inode, void **ctx, u32 *ctxlen)
{
	int len = 0;
	len = smack_inode_getsecurity(inode, XATTR_SMACK_SUFFIX, ctx, true);

	if (len < 0)
		return len;
	*ctxlen = len;
	return 0;
}

struct security_operations smack_ops = {
	.name =				"smack",

	.ptrace_access_check =		smack_ptrace_access_check,
	.ptrace_traceme =		smack_ptrace_traceme,
	.syslog = 			smack_syslog,

	.sb_alloc_security = 		smack_sb_alloc_security,
	.sb_free_security = 		smack_sb_free_security,
	.sb_copy_data = 		smack_sb_copy_data,
	.sb_kern_mount = 		smack_sb_kern_mount,
	.sb_statfs = 			smack_sb_statfs,
	.sb_mount = 			smack_sb_mount,
	.sb_umount = 			smack_sb_umount,

	.inode_alloc_security = 	smack_inode_alloc_security,
	.inode_free_security = 		smack_inode_free_security,
	.inode_init_security = 		smack_inode_init_security,
	.inode_link = 			smack_inode_link,
	.inode_unlink = 		smack_inode_unlink,
	.inode_rmdir = 			smack_inode_rmdir,
	.inode_rename = 		smack_inode_rename,
	.inode_permission = 		smack_inode_permission,
	.inode_setattr = 		smack_inode_setattr,
	.inode_getattr = 		smack_inode_getattr,
	.inode_setxattr = 		smack_inode_setxattr,
	.inode_post_setxattr = 		smack_inode_post_setxattr,
	.inode_getxattr = 		smack_inode_getxattr,
	.inode_removexattr = 		smack_inode_removexattr,
	.inode_getsecurity = 		smack_inode_getsecurity,
	.inode_setsecurity = 		smack_inode_setsecurity,
	.inode_listsecurity = 		smack_inode_listsecurity,
	.inode_getsecid =		smack_inode_getsecid,

	.file_permission = 		smack_file_permission,
	.file_alloc_security = 		smack_file_alloc_security,
	.file_free_security = 		smack_file_free_security,
	.file_ioctl = 			smack_file_ioctl,
	.file_lock = 			smack_file_lock,
	.file_fcntl = 			smack_file_fcntl,
	.file_set_fowner = 		smack_file_set_fowner,
	.file_send_sigiotask = 		smack_file_send_sigiotask,
	.file_receive = 		smack_file_receive,

	.cred_alloc_blank =		smack_cred_alloc_blank,
	.cred_free =			smack_cred_free,
	.cred_prepare =			smack_cred_prepare,
	.cred_transfer =		smack_cred_transfer,
	.kernel_act_as =		smack_kernel_act_as,
	.kernel_create_files_as =	smack_kernel_create_files_as,
	.task_setpgid = 		smack_task_setpgid,
	.task_getpgid = 		smack_task_getpgid,
	.task_getsid = 			smack_task_getsid,
	.task_getsecid = 		smack_task_getsecid,
	.task_setnice = 		smack_task_setnice,
	.task_setioprio = 		smack_task_setioprio,
	.task_getioprio = 		smack_task_getioprio,
	.task_setscheduler = 		smack_task_setscheduler,
	.task_getscheduler = 		smack_task_getscheduler,
	.task_movememory = 		smack_task_movememory,
	.task_kill = 			smack_task_kill,
	.task_wait = 			smack_task_wait,
	.task_to_inode = 		smack_task_to_inode,

	.ipc_permission = 		smack_ipc_permission,
	.ipc_getsecid =			smack_ipc_getsecid,

	.msg_msg_alloc_security = 	smack_msg_msg_alloc_security,
	.msg_msg_free_security = 	smack_msg_msg_free_security,

	.msg_queue_alloc_security = 	smack_msg_queue_alloc_security,
	.msg_queue_free_security = 	smack_msg_queue_free_security,
	.msg_queue_associate = 		smack_msg_queue_associate,
	.msg_queue_msgctl = 		smack_msg_queue_msgctl,
	.msg_queue_msgsnd = 		smack_msg_queue_msgsnd,
	.msg_queue_msgrcv = 		smack_msg_queue_msgrcv,

	.shm_alloc_security = 		smack_shm_alloc_security,
	.shm_free_security = 		smack_shm_free_security,
	.shm_associate = 		smack_shm_associate,
	.shm_shmctl = 			smack_shm_shmctl,
	.shm_shmat = 			smack_shm_shmat,

	.sem_alloc_security = 		smack_sem_alloc_security,
	.sem_free_security = 		smack_sem_free_security,
	.sem_associate = 		smack_sem_associate,
	.sem_semctl = 			smack_sem_semctl,
	.sem_semop = 			smack_sem_semop,

	.d_instantiate = 		smack_d_instantiate,

	.getprocattr = 			smack_getprocattr,
	.setprocattr = 			smack_setprocattr,

	.unix_stream_connect = 		smack_unix_stream_connect,
	.unix_may_send = 		smack_unix_may_send,

	.socket_post_create = 		smack_socket_post_create,
	.socket_connect =		smack_socket_connect,
	.socket_sendmsg =		smack_socket_sendmsg,
	.socket_sock_rcv_skb = 		smack_socket_sock_rcv_skb,
	.socket_getpeersec_stream =	smack_socket_getpeersec_stream,
	.socket_getpeersec_dgram =	smack_socket_getpeersec_dgram,
	.sk_alloc_security = 		smack_sk_alloc_security,
	.sk_free_security = 		smack_sk_free_security,
	.sock_graft = 			smack_sock_graft,
	.inet_conn_request = 		smack_inet_conn_request,
	.inet_csk_clone =		smack_inet_csk_clone,

 /* key management security hooks */
#ifdef CONFIG_KEYS
	.key_alloc = 			smack_key_alloc,
	.key_free = 			smack_key_free,
	.key_permission = 		smack_key_permission,
#endif /* CONFIG_KEYS */

 /* Audit hooks */
#ifdef CONFIG_AUDIT
	.audit_rule_init =		smack_audit_rule_init,
	.audit_rule_known =		smack_audit_rule_known,
	.audit_rule_match =		smack_audit_rule_match,
	.audit_rule_free =		smack_audit_rule_free,
#endif /* CONFIG_AUDIT */

	.secid_to_secctx = 		smack_secid_to_secctx,
	.secctx_to_secid = 		smack_secctx_to_secid,
	.release_secctx = 		smack_release_secctx,
	.inode_notifysecctx =		smack_inode_notifysecctx,
	.inode_setsecctx =		smack_inode_setsecctx,
	.inode_getsecctx =		smack_inode_getsecctx,
};


static __init void init_smack_know_list(void)
{
	list_add(&smack_known_huh.list, &smack_known_list);
	list_add(&smack_known_hat.list, &smack_known_list);
	list_add(&smack_known_star.list, &smack_known_list);
	list_add(&smack_known_floor.list, &smack_known_list);
	list_add(&smack_known_invalid.list, &smack_known_list);
	list_add(&smack_known_web.list, &smack_known_list);
}

static __init int smack_init(void)
{
	struct cred *cred;

	if (!security_module_enable(&smack_ops))
		return 0;

	printk(KERN_INFO "Smack:  Initializing.\n");

	/*
	 * Set the security state for the initial task.
	 */
	cred = (struct cred *) current->cred;
	cred->security = &smack_known_floor.smk_known;

	/* initilize the smack_know_list */
	init_smack_know_list();
	/*
	 * Initialize locks
	 */
	spin_lock_init(&smack_known_huh.smk_cipsolock);
	spin_lock_init(&smack_known_hat.smk_cipsolock);
	spin_lock_init(&smack_known_star.smk_cipsolock);
	spin_lock_init(&smack_known_floor.smk_cipsolock);
	spin_lock_init(&smack_known_invalid.smk_cipsolock);

	/*
	 * Register with LSM
	 */
	if (register_security(&smack_ops))
		panic("smack: Unable to register with kernel.\n");

	return 0;
}

security_initcall(smack_init);
