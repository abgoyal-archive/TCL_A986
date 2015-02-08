
#ifndef _LINUX_NSPROXY_H
#define _LINUX_NSPROXY_H

#include <linux/spinlock.h>
#include <linux/sched.h>

struct mnt_namespace;
struct uts_namespace;
struct ipc_namespace;
struct pid_namespace;
struct fs_struct;

struct nsproxy {
	atomic_t count;
	struct uts_namespace *uts_ns;
	struct ipc_namespace *ipc_ns;
	struct mnt_namespace *mnt_ns;
	struct pid_namespace *pid_ns;
	struct net 	     *net_ns;
};
extern struct nsproxy init_nsproxy;


static inline struct nsproxy *task_nsproxy(struct task_struct *tsk)
{
	return rcu_dereference(tsk->nsproxy);
}

int copy_namespaces(unsigned long flags, struct task_struct *tsk);
void exit_task_namespaces(struct task_struct *tsk);
void switch_task_namespaces(struct task_struct *tsk, struct nsproxy *new);
void free_nsproxy(struct nsproxy *ns);
int unshare_nsproxy_namespaces(unsigned long, struct nsproxy **,
	struct fs_struct *);

static inline void put_nsproxy(struct nsproxy *ns)
{
	if (atomic_dec_and_test(&ns->count)) {
		free_nsproxy(ns);
	}
}

static inline void get_nsproxy(struct nsproxy *ns)
{
	atomic_inc(&ns->count);
}

#ifdef CONFIG_CGROUP_NS
int ns_cgroup_clone(struct task_struct *tsk, struct pid *pid);
#else
static inline int ns_cgroup_clone(struct task_struct *tsk, struct pid *pid)
{
	return 0;
}
#endif

#endif
