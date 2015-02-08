


#ifndef STACKGLUE_H
#define STACKGLUE_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/dlmconstants.h>

#include "dlm/dlmapi.h"
#include <linux/dlm.h>

/* Needed for plock-related prototypes */
struct file;
struct file_lock;

#define DLM_LKF_LOCAL		0x00100000

#define GROUP_NAME_MAX		64


struct ocfs2_protocol_version {
	u8 pv_major;
	u8 pv_minor;
};

struct fsdlm_lksb_plus_lvb {
	struct dlm_lksb lksb;
	char lvb[DLM_LVB_LEN];
};

struct ocfs2_cluster_connection;
struct ocfs2_dlm_lksb {
	 union {
		 struct dlm_lockstatus lksb_o2dlm;
		 struct dlm_lksb lksb_fsdlm;
		 struct fsdlm_lksb_plus_lvb padding;
	 };
	 struct ocfs2_cluster_connection *lksb_conn;
};

struct ocfs2_locking_protocol {
	struct ocfs2_protocol_version lp_max_version;
	void (*lp_lock_ast)(struct ocfs2_dlm_lksb *lksb);
	void (*lp_blocking_ast)(struct ocfs2_dlm_lksb *lksb, int level);
	void (*lp_unlock_ast)(struct ocfs2_dlm_lksb *lksb, int error);
};


struct ocfs2_cluster_connection {
	char cc_name[GROUP_NAME_MAX];
	int cc_namelen;
	struct ocfs2_protocol_version cc_version;
	struct ocfs2_locking_protocol *cc_proto;
	void (*cc_recovery_handler)(int node_num, void *recovery_data);
	void *cc_recovery_data;
	void *cc_lockspace;
	void *cc_private;
};

struct ocfs2_stack_operations {
	/*
	 * The fs code calls ocfs2_cluster_connect() to attach a new
	 * filesystem to the cluster stack.  The ->connect() op is passed
	 * an ocfs2_cluster_connection with the name and recovery field
	 * filled in.
	 *
	 * The stack must set up any notification mechanisms and create
	 * the filesystem lockspace in the DLM.  The lockspace should be
	 * stored on cc_lockspace.  Any other information can be stored on
	 * cc_private.
	 *
	 * ->connect() must not return until it is guaranteed that
	 *
	 *  - Node down notifications for the filesystem will be recieved
	 *    and passed to conn->cc_recovery_handler().
	 *  - Locking requests for the filesystem will be processed.
	 */
	int (*connect)(struct ocfs2_cluster_connection *conn);

	/*
	 * The fs code calls ocfs2_cluster_disconnect() when a filesystem
	 * no longer needs cluster services.  All DLM locks have been
	 * dropped, and recovery notification is being ignored by the
	 * fs code.  The stack must disengage from the DLM and discontinue
	 * recovery notification.
	 *
	 * Once ->disconnect() has returned, the connection structure will
	 * be freed.  Thus, a stack must not return from ->disconnect()
	 * until it will no longer reference the conn pointer.
	 *
	 * Once this call returns, the stack glue will be dropping this
	 * connection's reference on the module.
	 */
	int (*disconnect)(struct ocfs2_cluster_connection *conn);

	/*
	 * ->this_node() returns the cluster's unique identifier for the
	 * local node.
	 */
	int (*this_node)(unsigned int *node);

	/*
	 * Call the underlying dlm lock function.  The ->dlm_lock()
	 * callback should convert the flags and mode as appropriate.
	 *
	 * ast and bast functions are not part of the call because the
	 * stack will likely want to wrap ast and bast calls before passing
	 * them to stack->sp_proto.  There is no astarg.  The lksb will
	 * be passed back to the ast and bast functions.  The caller can
	 * use this to find their object.
	 */
	int (*dlm_lock)(struct ocfs2_cluster_connection *conn,
			int mode,
			struct ocfs2_dlm_lksb *lksb,
			u32 flags,
			void *name,
			unsigned int namelen);

	/*
	 * Call the underlying dlm unlock function.  The ->dlm_unlock()
	 * function should convert the flags as appropriate.
	 *
	 * The unlock ast is not passed, as the stack will want to wrap
	 * it before calling stack->sp_proto->lp_unlock_ast().  There is
	 * no astarg.  The lksb will be passed back to the unlock ast
	 * function.  The caller can use this to find their object.
	 */
	int (*dlm_unlock)(struct ocfs2_cluster_connection *conn,
			  struct ocfs2_dlm_lksb *lksb,
			  u32 flags);

	/*
	 * Return the status of the current lock status block.  The fs
	 * code should never dereference the union.  The ->lock_status()
	 * callback pulls out the stack-specific lksb, converts the status
	 * to a proper errno, and returns it.
	 */
	int (*lock_status)(struct ocfs2_dlm_lksb *lksb);

	/*
	 * Return non-zero if the LVB is valid.
	 */
	int (*lvb_valid)(struct ocfs2_dlm_lksb *lksb);

	/*
	 * Pull the lvb pointer off of the stack-specific lksb.
	 */
	void *(*lock_lvb)(struct ocfs2_dlm_lksb *lksb);

	/*
	 * Cluster-aware posix locks
	 *
	 * This is NULL for stacks which do not support posix locks.
	 */
	int (*plock)(struct ocfs2_cluster_connection *conn,
		     u64 ino,
		     struct file *file,
		     int cmd,
		     struct file_lock *fl);

	/*
	 * This is an optoinal debugging hook.  If provided, the
	 * stack can dump debugging information about this lock.
	 */
	void (*dump_lksb)(struct ocfs2_dlm_lksb *lksb);
};

struct ocfs2_stack_plugin {
	char *sp_name;
	struct ocfs2_stack_operations *sp_ops;
	struct module *sp_owner;

	/* These are managed by the stackglue code. */
	struct list_head sp_list;
	unsigned int sp_count;
	struct ocfs2_protocol_version sp_max_proto;
};


/* Used by the filesystem */
int ocfs2_cluster_connect(const char *stack_name,
			  const char *group,
			  int grouplen,
			  struct ocfs2_locking_protocol *lproto,
			  void (*recovery_handler)(int node_num,
						   void *recovery_data),
			  void *recovery_data,
			  struct ocfs2_cluster_connection **conn);
int ocfs2_cluster_connect_agnostic(const char *group,
				   int grouplen,
				   struct ocfs2_locking_protocol *lproto,
				   void (*recovery_handler)(int node_num,
							    void *recovery_data),
				   void *recovery_data,
				   struct ocfs2_cluster_connection **conn);
int ocfs2_cluster_disconnect(struct ocfs2_cluster_connection *conn,
			     int hangup_pending);
void ocfs2_cluster_hangup(const char *group, int grouplen);
int ocfs2_cluster_this_node(unsigned int *node);

struct ocfs2_lock_res;
int ocfs2_dlm_lock(struct ocfs2_cluster_connection *conn,
		   int mode,
		   struct ocfs2_dlm_lksb *lksb,
		   u32 flags,
		   void *name,
		   unsigned int namelen);
int ocfs2_dlm_unlock(struct ocfs2_cluster_connection *conn,
		     struct ocfs2_dlm_lksb *lksb,
		     u32 flags);

int ocfs2_dlm_lock_status(struct ocfs2_dlm_lksb *lksb);
int ocfs2_dlm_lvb_valid(struct ocfs2_dlm_lksb *lksb);
void *ocfs2_dlm_lvb(struct ocfs2_dlm_lksb *lksb);
void ocfs2_dlm_dump_lksb(struct ocfs2_dlm_lksb *lksb);

int ocfs2_stack_supports_plocks(void);
int ocfs2_plock(struct ocfs2_cluster_connection *conn, u64 ino,
		struct file *file, int cmd, struct file_lock *fl);

void ocfs2_stack_glue_set_max_proto_version(struct ocfs2_protocol_version *max_proto);


/* Used by stack plugins */
int ocfs2_stack_glue_register(struct ocfs2_stack_plugin *plugin);
void ocfs2_stack_glue_unregister(struct ocfs2_stack_plugin *plugin);

#endif  /* STACKGLUE_H */
