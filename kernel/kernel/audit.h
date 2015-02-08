

#include <linux/fs.h>
#include <linux/audit.h>
#include <linux/skbuff.h>

#define AUDIT_DEBUG 0

enum audit_state {
	AUDIT_DISABLED,		/* Do not create per-task audit_context.
				 * No syscall-specific audit records can
				 * be generated. */
	AUDIT_SETUP_CONTEXT,	/* Create the per-task audit_context,
				 * but don't necessarily fill it in at
				 * syscall entry time (i.e., filter
				 * instead). */
	AUDIT_BUILD_CONTEXT,	/* Create the per-task audit_context,
				 * and always fill it in at syscall
				 * entry time.  This makes a full
				 * syscall record available if some
				 * other part of the kernel decides it
				 * should be recorded. */
	AUDIT_RECORD_CONTEXT	/* Create the per-task audit_context,
				 * always fill it in at syscall entry
				 * time, and always write out the audit
				 * record at syscall exit time.  */
};

/* Rule lists */
struct audit_watch;
struct audit_tree;
struct audit_chunk;

struct audit_entry {
	struct list_head	list;
	struct rcu_head		rcu;
	struct audit_krule	rule;
};

#ifdef CONFIG_AUDIT
extern int audit_enabled;
extern int audit_ever_enabled;
#endif

extern int audit_pid;

#define AUDIT_INODE_BUCKETS	32
extern struct list_head audit_inode_hash[AUDIT_INODE_BUCKETS];

static inline int audit_hash_ino(u32 ino)
{
	return (ino & (AUDIT_INODE_BUCKETS-1));
}

extern int audit_match_class(int class, unsigned syscall);
extern int audit_comparator(const u32 left, const u32 op, const u32 right);
extern int audit_compare_dname_path(const char *dname, const char *path,
				    int *dirlen);
extern struct sk_buff *	    audit_make_reply(int pid, int seq, int type,
					     int done, int multi,
					     void *payload, int size);
extern void		    audit_send_reply(int pid, int seq, int type,
					     int done, int multi,
					     void *payload, int size);
extern void		    audit_panic(const char *message);

struct audit_netlink_list {
	int pid;
	struct sk_buff_head q;
};

int audit_send_list(void *);

extern int selinux_audit_rule_update(void);

extern struct mutex audit_filter_mutex;
extern void audit_free_rule_rcu(struct rcu_head *);
extern struct list_head audit_filter_list[];

/* audit watch functions */
extern unsigned long audit_watch_inode(struct audit_watch *watch);
extern dev_t audit_watch_dev(struct audit_watch *watch);
extern void audit_put_watch(struct audit_watch *watch);
extern void audit_get_watch(struct audit_watch *watch);
extern int audit_to_watch(struct audit_krule *krule, char *path, int len, u32 op);
extern int audit_add_watch(struct audit_krule *krule);
extern void audit_remove_watch(struct audit_watch *watch);
extern void audit_remove_watch_rule(struct audit_krule *krule, struct list_head *list);
extern void audit_inotify_unregister(struct list_head *in_list);
extern char *audit_watch_path(struct audit_watch *watch);
extern struct list_head *audit_watch_rules(struct audit_watch *watch);

extern struct audit_entry *audit_dupe_rule(struct audit_krule *old,
					   struct audit_watch *watch);

#ifdef CONFIG_AUDIT_TREE
extern struct audit_chunk *audit_tree_lookup(const struct inode *);
extern void audit_put_chunk(struct audit_chunk *);
extern int audit_tree_match(struct audit_chunk *, struct audit_tree *);
extern int audit_make_tree(struct audit_krule *, char *, u32);
extern int audit_add_tree_rule(struct audit_krule *);
extern int audit_remove_tree_rule(struct audit_krule *);
extern void audit_trim_trees(void);
extern int audit_tag_tree(char *old, char *new);
extern const char *audit_tree_path(struct audit_tree *);
extern void audit_put_tree(struct audit_tree *);
extern void audit_kill_trees(struct list_head *);
#else
#define audit_remove_tree_rule(rule) BUG()
#define audit_add_tree_rule(rule) -EINVAL
#define audit_make_tree(rule, str, op) -EINVAL
#define audit_trim_trees() (void)0
#define audit_put_tree(tree) (void)0
#define audit_tag_tree(old, new) -EINVAL
#define audit_tree_path(rule) ""	/* never called */
#define audit_kill_trees(list) BUG()
#endif

extern char *audit_unpack_string(void **, size_t *, size_t);

extern pid_t audit_sig_pid;
extern uid_t audit_sig_uid;
extern u32 audit_sig_sid;

#ifdef CONFIG_AUDITSYSCALL
extern int __audit_signal_info(int sig, struct task_struct *t);
static inline int audit_signal_info(int sig, struct task_struct *t)
{
	if (unlikely((audit_pid && t->tgid == audit_pid) ||
		     (audit_signals && !audit_dummy_context())))
		return __audit_signal_info(sig, t);
	return 0;
}
extern void audit_filter_inodes(struct task_struct *, struct audit_context *);
extern struct list_head *audit_killed_trees(void);
#else
#define audit_signal_info(s,t) AUDIT_DISABLED
#define audit_filter_inodes(t,c) AUDIT_DISABLED
#endif

extern struct mutex audit_cmd_mutex;
