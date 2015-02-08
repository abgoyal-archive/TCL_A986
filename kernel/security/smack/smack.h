

#ifndef _SECURITY_SMACK_H
#define _SECURITY_SMACK_H

#include <linux/capability.h>
#include <linux/spinlock.h>
#include <linux/security.h>
#include <linux/in.h>
#include <net/netlabel.h>
#include <linux/list.h>
#include <linux/rculist.h>
#include <linux/lsm_audit.h>

#define SMK_MAXLEN	23
#define SMK_LABELLEN	(SMK_MAXLEN+1)

struct superblock_smack {
	char		*smk_root;
	char		*smk_floor;
	char		*smk_hat;
	char		*smk_default;
	int		smk_initialized;
	spinlock_t	smk_sblock;	/* for initialization */
};

struct socket_smack {
	char		*smk_out;			/* outbound label */
	char		*smk_in;			/* inbound label */
	char		smk_packet[SMK_LABELLEN];	/* TCP peer label */
};

struct inode_smack {
	char		*smk_inode;	/* label of the fso */
	struct mutex	smk_lock;	/* initialization lock */
	int		smk_flags;	/* smack inode flags */
};

#define	SMK_INODE_INSTANT	0x01	/* inode is instantiated */

struct smack_rule {
	struct list_head	list;
	char			*smk_subject;
	char			*smk_object;
	int			smk_access;
};

struct smack_cipso {
	int	smk_level;
	char	smk_catset[SMK_LABELLEN];
};

struct smk_netlbladdr {
	struct list_head	list;
	struct sockaddr_in	smk_host;	/* network address */
	struct in_addr		smk_mask;	/* network mask */
	char			*smk_label;	/* label */
};

struct smack_known {
	struct list_head	list;
	char			smk_known[SMK_LABELLEN];
	u32			smk_secid;
	struct smack_cipso	*smk_cipso;
	spinlock_t		smk_cipsolock; /* for changing cipso map */
};

#define SMK_FSDEFAULT	"smackfsdef="
#define SMK_FSFLOOR	"smackfsfloor="
#define SMK_FSHAT	"smackfshat="
#define SMK_FSROOT	"smackfsroot="

#define XATTR_SMACK_SUFFIX	"SMACK64"
#define XATTR_SMACK_IPIN	"SMACK64IPIN"
#define XATTR_SMACK_IPOUT	"SMACK64IPOUT"
#define XATTR_NAME_SMACK	XATTR_SECURITY_PREFIX XATTR_SMACK_SUFFIX
#define XATTR_NAME_SMACKIPIN	XATTR_SECURITY_PREFIX XATTR_SMACK_IPIN
#define XATTR_NAME_SMACKIPOUT	XATTR_SECURITY_PREFIX XATTR_SMACK_IPOUT

#define SMACK_CIPSO_OPTION 	"-CIPSO"

#define SMACK_UNLABELED_SOCKET	0
#define SMACK_CIPSO_SOCKET	1

#define SMACK_MAGIC	0x43415d53 /* "SMAC" */

#define SMACK_LIST_MAX	10000

#define SMACK_CIPSO_DOI_DEFAULT		3	/* Historical */
#define SMACK_CIPSO_DOI_INVALID		-1	/* Not a DOI */
#define SMACK_CIPSO_DIRECT_DEFAULT	250	/* Arbitrary */
#define SMACK_CIPSO_MAXCATVAL		63	/* Bigger gets harder */
#define SMACK_CIPSO_MAXLEVEL            255     /* CIPSO 2.2 standard */
#define SMACK_CIPSO_MAXCATNUM           239     /* CIPSO 2.2 standard */

#define MAY_ANY		(MAY_READ | MAY_WRITE | MAY_APPEND | MAY_EXEC)
#define MAY_ANYREAD	(MAY_READ | MAY_EXEC)
#define MAY_ANYWRITE	(MAY_WRITE | MAY_APPEND)
#define MAY_READWRITE	(MAY_READ | MAY_WRITE)
#define MAY_NOT		0

#define SMK_NUM_ACCESS_TYPE 4

struct smk_audit_info {
#ifdef CONFIG_AUDIT
	struct common_audit_data a;
#endif
};
struct inode_smack *new_inode_smack(char *);

int smk_access(char *, char *, int, struct smk_audit_info *);
int smk_curacc(char *, u32, struct smk_audit_info *);
int smack_to_cipso(const char *, struct smack_cipso *);
void smack_from_cipso(u32, char *, char *);
char *smack_from_secid(const u32);
char *smk_import(const char *, int);
struct smack_known *smk_import_entry(const char *, int);
u32 smack_to_secid(const char *);

extern int smack_cipso_direct;
extern char *smack_net_ambient;
extern char *smack_onlycap;
extern const char *smack_cipso_option;

extern struct smack_known smack_known_floor;
extern struct smack_known smack_known_hat;
extern struct smack_known smack_known_huh;
extern struct smack_known smack_known_invalid;
extern struct smack_known smack_known_star;
extern struct smack_known smack_known_web;

extern struct list_head smack_known_list;
extern struct list_head smack_rule_list;
extern struct list_head smk_netlbladdr_list;

extern struct security_operations smack_ops;

static inline void smack_catset_bit(int cat, char *catsetp)
{
	if (cat > SMK_LABELLEN * 8)
		return;

	catsetp[(cat - 1) / 8] |= 0x80 >> ((cat - 1) % 8);
}

static inline char *smk_of_inode(const struct inode *isp)
{
	struct inode_smack *sip = isp->i_security;
	return sip->smk_inode;
}

#define SMACK_AUDIT_DENIED 0x1
#define SMACK_AUDIT_ACCEPT 0x2
extern int log_policy;

void smack_log(char *subject_label, char *object_label,
		int request,
		int result, struct smk_audit_info *auditdata);

#ifdef CONFIG_AUDIT

static inline void smk_ad_init(struct smk_audit_info *a, const char *func,
			       char type)
{
	memset(a, 0, sizeof(*a));
	a->a.type = type;
	a->a.smack_audit_data.function = func;
}

static inline void smk_ad_setfield_u_tsk(struct smk_audit_info *a,
					 struct task_struct *t)
{
	a->a.u.tsk = t;
}
static inline void smk_ad_setfield_u_fs_path_dentry(struct smk_audit_info *a,
						    struct dentry *d)
{
	a->a.u.fs.path.dentry = d;
}
static inline void smk_ad_setfield_u_fs_path_mnt(struct smk_audit_info *a,
						 struct vfsmount *m)
{
	a->a.u.fs.path.mnt = m;
}
static inline void smk_ad_setfield_u_fs_inode(struct smk_audit_info *a,
					      struct inode *i)
{
	a->a.u.fs.inode = i;
}
static inline void smk_ad_setfield_u_fs_path(struct smk_audit_info *a,
					     struct path p)
{
	a->a.u.fs.path = p;
}
static inline void smk_ad_setfield_u_net_sk(struct smk_audit_info *a,
					    struct sock *sk)
{
	a->a.u.net.sk = sk;
}

#else /* no AUDIT */

static inline void smk_ad_init(struct smk_audit_info *a, const char *func,
			       char type)
{
}
static inline void smk_ad_setfield_u_tsk(struct smk_audit_info *a,
					 struct task_struct *t)
{
}
static inline void smk_ad_setfield_u_fs_path_dentry(struct smk_audit_info *a,
						    struct dentry *d)
{
}
static inline void smk_ad_setfield_u_fs_path_mnt(struct smk_audit_info *a,
						 struct vfsmount *m)
{
}
static inline void smk_ad_setfield_u_fs_inode(struct smk_audit_info *a,
					      struct inode *i)
{
}
static inline void smk_ad_setfield_u_fs_path(struct smk_audit_info *a,
					     struct path p)
{
}
static inline void smk_ad_setfield_u_net_sk(struct smk_audit_info *a,
					    struct sock *sk)
{
}
#endif

#endif  /* _SECURITY_SMACK_H */
