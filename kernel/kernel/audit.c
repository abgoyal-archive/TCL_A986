

#include <linux/init.h>
#include <asm/types.h>
#include <asm/atomic.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/kthread.h>

#include <linux/audit.h>

#include <net/sock.h>
#include <net/netlink.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/inotify.h>
#include <linux/freezer.h>
#include <linux/tty.h>

#include "audit.h"

#define AUDIT_DISABLED		-1
#define AUDIT_UNINITIALIZED	0
#define AUDIT_INITIALIZED	1
static int	audit_initialized;

#define AUDIT_OFF	0
#define AUDIT_ON	1
#define AUDIT_LOCKED	2
int		audit_enabled;
int		audit_ever_enabled;

/* Default state when kernel boots without any parameters. */
static int	audit_default;

/* If auditing cannot proceed, audit_failure selects what happens. */
static int	audit_failure = AUDIT_FAIL_PRINTK;

int		audit_pid;
static int	audit_nlk_pid;

static int	audit_rate_limit;

/* Number of outstanding audit_buffers allowed. */
static int	audit_backlog_limit = 64;
static int	audit_backlog_wait_time = 60 * HZ;
static int	audit_backlog_wait_overflow = 0;

/* The identity of the user shutting down the audit system. */
uid_t		audit_sig_uid = -1;
pid_t		audit_sig_pid = -1;
u32		audit_sig_sid = 0;

static atomic_t    audit_lost = ATOMIC_INIT(0);

/* The netlink socket. */
static struct sock *audit_sock;

/* Hash for inode-based rules */
struct list_head audit_inode_hash[AUDIT_INODE_BUCKETS];

static DEFINE_SPINLOCK(audit_freelist_lock);
static int	   audit_freelist_count;
static LIST_HEAD(audit_freelist);

static struct sk_buff_head audit_skb_queue;
/* queue of skbs to send to auditd when/if it comes back */
static struct sk_buff_head audit_skb_hold_queue;
static struct task_struct *kauditd_task;
static DECLARE_WAIT_QUEUE_HEAD(kauditd_wait);
static DECLARE_WAIT_QUEUE_HEAD(audit_backlog_wait);

/* Serialize requests from userspace. */
DEFINE_MUTEX(audit_cmd_mutex);

#define AUDIT_BUFSIZ 1024

#define AUDIT_MAXFREE  (2*NR_CPUS)

struct audit_buffer {
	struct list_head     list;
	struct sk_buff       *skb;	/* formatted skb ready to send */
	struct audit_context *ctx;	/* NULL or associated context */
	gfp_t		     gfp_mask;
};

struct audit_reply {
	int pid;
	struct sk_buff *skb;
};

static void audit_set_pid(struct audit_buffer *ab, pid_t pid)
{
	if (ab) {
		struct nlmsghdr *nlh = nlmsg_hdr(ab->skb);
		nlh->nlmsg_pid = pid;
	}
}

void audit_panic(const char *message)
{
	switch (audit_failure)
	{
	case AUDIT_FAIL_SILENT:
		break;
	case AUDIT_FAIL_PRINTK:
		if (printk_ratelimit())
			printk(KERN_ERR "audit: %s\n", message);
		break;
	case AUDIT_FAIL_PANIC:
		/* test audit_pid since printk is always losey, why bother? */
		if (audit_pid)
			panic("audit: %s\n", message);
		break;
	}
}

static inline int audit_rate_check(void)
{
	static unsigned long	last_check = 0;
	static int		messages   = 0;
	static DEFINE_SPINLOCK(lock);
	unsigned long		flags;
	unsigned long		now;
	unsigned long		elapsed;
	int			retval	   = 0;

	if (!audit_rate_limit) return 1;

	spin_lock_irqsave(&lock, flags);
	if (++messages < audit_rate_limit) {
		retval = 1;
	} else {
		now     = jiffies;
		elapsed = now - last_check;
		if (elapsed > HZ) {
			last_check = now;
			messages   = 0;
			retval     = 1;
		}
	}
	spin_unlock_irqrestore(&lock, flags);

	return retval;
}

void audit_log_lost(const char *message)
{
	static unsigned long	last_msg = 0;
	static DEFINE_SPINLOCK(lock);
	unsigned long		flags;
	unsigned long		now;
	int			print;

	atomic_inc(&audit_lost);

	print = (audit_failure == AUDIT_FAIL_PANIC || !audit_rate_limit);

	if (!print) {
		spin_lock_irqsave(&lock, flags);
		now = jiffies;
		if (now - last_msg > HZ) {
			print = 1;
			last_msg = now;
		}
		spin_unlock_irqrestore(&lock, flags);
	}

	if (print) {
		if (printk_ratelimit())
			printk(KERN_WARNING
				"audit: audit_lost=%d audit_rate_limit=%d "
				"audit_backlog_limit=%d\n",
				atomic_read(&audit_lost),
				audit_rate_limit,
				audit_backlog_limit);
		audit_panic(message);
	}
}

static int audit_log_config_change(char *function_name, int new, int old,
				   uid_t loginuid, u32 sessionid, u32 sid,
				   int allow_changes)
{
	struct audit_buffer *ab;
	int rc = 0;

	ab = audit_log_start(NULL, GFP_KERNEL, AUDIT_CONFIG_CHANGE);
	audit_log_format(ab, "%s=%d old=%d auid=%u ses=%u", function_name, new,
			 old, loginuid, sessionid);
	if (sid) {
		char *ctx = NULL;
		u32 len;

		rc = security_secid_to_secctx(sid, &ctx, &len);
		if (rc) {
			audit_log_format(ab, " sid=%u", sid);
			allow_changes = 0; /* Something weird, deny request */
		} else {
			audit_log_format(ab, " subj=%s", ctx);
			security_release_secctx(ctx, len);
		}
	}
	audit_log_format(ab, " res=%d", allow_changes);
	audit_log_end(ab);
	return rc;
}

static int audit_do_config_change(char *function_name, int *to_change,
				  int new, uid_t loginuid, u32 sessionid,
				  u32 sid)
{
	int allow_changes, rc = 0, old = *to_change;

	/* check if we are locked */
	if (audit_enabled == AUDIT_LOCKED)
		allow_changes = 0;
	else
		allow_changes = 1;

	if (audit_enabled != AUDIT_OFF) {
		rc = audit_log_config_change(function_name, new, old, loginuid,
					     sessionid, sid, allow_changes);
		if (rc)
			allow_changes = 0;
	}

	/* If we are allowed, make the change */
	if (allow_changes == 1)
		*to_change = new;
	/* Not allowed, update reason */
	else if (rc == 0)
		rc = -EPERM;
	return rc;
}

static int audit_set_rate_limit(int limit, uid_t loginuid, u32 sessionid,
				u32 sid)
{
	return audit_do_config_change("audit_rate_limit", &audit_rate_limit,
				      limit, loginuid, sessionid, sid);
}

static int audit_set_backlog_limit(int limit, uid_t loginuid, u32 sessionid,
				   u32 sid)
{
	return audit_do_config_change("audit_backlog_limit", &audit_backlog_limit,
				      limit, loginuid, sessionid, sid);
}

static int audit_set_enabled(int state, uid_t loginuid, u32 sessionid, u32 sid)
{
	int rc;
	if (state < AUDIT_OFF || state > AUDIT_LOCKED)
		return -EINVAL;

	rc =  audit_do_config_change("audit_enabled", &audit_enabled, state,
				     loginuid, sessionid, sid);

	if (!rc)
		audit_ever_enabled |= !!state;

	return rc;
}

static int audit_set_failure(int state, uid_t loginuid, u32 sessionid, u32 sid)
{
	if (state != AUDIT_FAIL_SILENT
	    && state != AUDIT_FAIL_PRINTK
	    && state != AUDIT_FAIL_PANIC)
		return -EINVAL;

	return audit_do_config_change("audit_failure", &audit_failure, state,
				      loginuid, sessionid, sid);
}

static void audit_hold_skb(struct sk_buff *skb)
{
	if (audit_default &&
	    skb_queue_len(&audit_skb_hold_queue) < audit_backlog_limit)
		skb_queue_tail(&audit_skb_hold_queue, skb);
	else
		kfree_skb(skb);
}

static void audit_printk_skb(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = nlmsg_hdr(skb);
	char *data = NLMSG_DATA(nlh);

	if (nlh->nlmsg_type != AUDIT_EOE) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "type=%d %s\n", nlh->nlmsg_type, data);
		else
			audit_log_lost("printk limit exceeded\n");
	}

	audit_hold_skb(skb);
}

static void kauditd_send_skb(struct sk_buff *skb)
{
	int err;
	/* take a reference in case we can't send it and we want to hold it */
	skb_get(skb);
	err = netlink_unicast(audit_sock, skb, audit_nlk_pid, 0);
	if (err < 0) {
		BUG_ON(err != -ECONNREFUSED); /* Shouldn't happen */
		printk(KERN_ERR "audit: *NO* daemon at audit_pid=%d\n", audit_pid);
		audit_log_lost("auditd dissapeared\n");
		audit_pid = 0;
		/* we might get lucky and get this in the next auditd */
		audit_hold_skb(skb);
	} else
		/* drop the extra reference if sent ok */
		kfree_skb(skb);
}

static int kauditd_thread(void *dummy)
{
	struct sk_buff *skb;

	set_freezable();
	while (!kthread_should_stop()) {
		/*
		 * if auditd just started drain the queue of messages already
		 * sent to syslog/printk.  remember loss here is ok.  we already
		 * called audit_log_lost() if it didn't go out normally.  so the
		 * race between the skb_dequeue and the next check for audit_pid
		 * doesn't matter.
		 *
		 * if you ever find kauditd to be too slow we can get a perf win
		 * by doing our own locking and keeping better track if there
		 * are messages in this queue.  I don't see the need now, but
		 * in 5 years when I want to play with this again I'll see this
		 * note and still have no friggin idea what i'm thinking today.
		 */
		if (audit_default && audit_pid) {
			skb = skb_dequeue(&audit_skb_hold_queue);
			if (unlikely(skb)) {
				while (skb && audit_pid) {
					kauditd_send_skb(skb);
					skb = skb_dequeue(&audit_skb_hold_queue);
				}
			}
		}

		skb = skb_dequeue(&audit_skb_queue);
		wake_up(&audit_backlog_wait);
		if (skb) {
			if (audit_pid)
				kauditd_send_skb(skb);
			else
				audit_printk_skb(skb);
		} else {
			DECLARE_WAITQUEUE(wait, current);
			set_current_state(TASK_INTERRUPTIBLE);
			add_wait_queue(&kauditd_wait, &wait);

			if (!skb_queue_len(&audit_skb_queue)) {
				try_to_freeze();
				schedule();
			}

			__set_current_state(TASK_RUNNING);
			remove_wait_queue(&kauditd_wait, &wait);
		}
	}
	return 0;
}

static int audit_prepare_user_tty(pid_t pid, uid_t loginuid, u32 sessionid)
{
	struct task_struct *tsk;
	int err;

	read_lock(&tasklist_lock);
	tsk = find_task_by_vpid(pid);
	err = -ESRCH;
	if (!tsk)
		goto out;
	err = 0;

	spin_lock_irq(&tsk->sighand->siglock);
	if (!tsk->signal->audit_tty)
		err = -EPERM;
	spin_unlock_irq(&tsk->sighand->siglock);
	if (err)
		goto out;

	tty_audit_push_task(tsk, loginuid, sessionid);
out:
	read_unlock(&tasklist_lock);
	return err;
}

int audit_send_list(void *_dest)
{
	struct audit_netlink_list *dest = _dest;
	int pid = dest->pid;
	struct sk_buff *skb;

	/* wait for parent to finish and send an ACK */
	mutex_lock(&audit_cmd_mutex);
	mutex_unlock(&audit_cmd_mutex);

	while ((skb = __skb_dequeue(&dest->q)) != NULL)
		netlink_unicast(audit_sock, skb, pid, 0);

	kfree(dest);

	return 0;
}

struct sk_buff *audit_make_reply(int pid, int seq, int type, int done,
				 int multi, void *payload, int size)
{
	struct sk_buff	*skb;
	struct nlmsghdr	*nlh;
	void		*data;
	int		flags = multi ? NLM_F_MULTI : 0;
	int		t     = done  ? NLMSG_DONE  : type;

	skb = nlmsg_new(size, GFP_KERNEL);
	if (!skb)
		return NULL;

	nlh	= NLMSG_NEW(skb, pid, seq, t, size, flags);
	data	= NLMSG_DATA(nlh);
	memcpy(data, payload, size);
	return skb;

nlmsg_failure:			/* Used by NLMSG_NEW */
	if (skb)
		kfree_skb(skb);
	return NULL;
}

static int audit_send_reply_thread(void *arg)
{
	struct audit_reply *reply = (struct audit_reply *)arg;

	mutex_lock(&audit_cmd_mutex);
	mutex_unlock(&audit_cmd_mutex);

	/* Ignore failure. It'll only happen if the sender goes away,
	   because our timeout is set to infinite. */
	netlink_unicast(audit_sock, reply->skb, reply->pid, 0);
	kfree(reply);
	return 0;
}
void audit_send_reply(int pid, int seq, int type, int done, int multi,
		      void *payload, int size)
{
	struct sk_buff *skb;
	struct task_struct *tsk;
	struct audit_reply *reply = kmalloc(sizeof(struct audit_reply),
					    GFP_KERNEL);

	if (!reply)
		return;

	skb = audit_make_reply(pid, seq, type, done, multi, payload, size);
	if (!skb)
		goto out;

	reply->pid = pid;
	reply->skb = skb;

	tsk = kthread_run(audit_send_reply_thread, reply, "audit_send_reply");
	if (!IS_ERR(tsk))
		return;
	kfree_skb(skb);
out:
	kfree(reply);
}

static int audit_netlink_ok(struct sk_buff *skb, u16 msg_type)
{
	int err = 0;

	switch (msg_type) {
	case AUDIT_GET:
	case AUDIT_LIST:
	case AUDIT_LIST_RULES:
	case AUDIT_SET:
	case AUDIT_ADD:
	case AUDIT_ADD_RULE:
	case AUDIT_DEL:
	case AUDIT_DEL_RULE:
	case AUDIT_SIGNAL_INFO:
	case AUDIT_TTY_GET:
	case AUDIT_TTY_SET:
	case AUDIT_TRIM:
	case AUDIT_MAKE_EQUIV:
		if (security_netlink_recv(skb, CAP_AUDIT_CONTROL))
			err = -EPERM;
		break;
	case AUDIT_USER:
	case AUDIT_FIRST_USER_MSG ... AUDIT_LAST_USER_MSG:
	case AUDIT_FIRST_USER_MSG2 ... AUDIT_LAST_USER_MSG2:
		if (security_netlink_recv(skb, CAP_AUDIT_WRITE))
			err = -EPERM;
		break;
	default:  /* bad msg */
		err = -EINVAL;
	}

	return err;
}

static int audit_log_common_recv_msg(struct audit_buffer **ab, u16 msg_type,
				     u32 pid, u32 uid, uid_t auid, u32 ses,
				     u32 sid)
{
	int rc = 0;
	char *ctx = NULL;
	u32 len;

	if (!audit_enabled) {
		*ab = NULL;
		return rc;
	}

	*ab = audit_log_start(NULL, GFP_KERNEL, msg_type);
	audit_log_format(*ab, "user pid=%d uid=%u auid=%u ses=%u",
			 pid, uid, auid, ses);
	if (sid) {
		rc = security_secid_to_secctx(sid, &ctx, &len);
		if (rc)
			audit_log_format(*ab, " ssid=%u", sid);
		else {
			audit_log_format(*ab, " subj=%s", ctx);
			security_release_secctx(ctx, len);
		}
	}

	return rc;
}

static int audit_receive_msg(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	u32			uid, pid, seq, sid;
	void			*data;
	struct audit_status	*status_get, status_set;
	int			err;
	struct audit_buffer	*ab;
	u16			msg_type = nlh->nlmsg_type;
	uid_t			loginuid; /* loginuid of sender */
	u32			sessionid;
	struct audit_sig_info   *sig_data;
	char			*ctx = NULL;
	u32			len;

	err = audit_netlink_ok(skb, msg_type);
	if (err)
		return err;

	/* As soon as there's any sign of userspace auditd,
	 * start kauditd to talk to it */
	if (!kauditd_task)
		kauditd_task = kthread_run(kauditd_thread, NULL, "kauditd");
	if (IS_ERR(kauditd_task)) {
		err = PTR_ERR(kauditd_task);
		kauditd_task = NULL;
		return err;
	}

	pid  = NETLINK_CREDS(skb)->pid;
	uid  = NETLINK_CREDS(skb)->uid;
	loginuid = NETLINK_CB(skb).loginuid;
	sessionid = NETLINK_CB(skb).sessionid;
	sid  = NETLINK_CB(skb).sid;
	seq  = nlh->nlmsg_seq;
	data = NLMSG_DATA(nlh);

	switch (msg_type) {
	case AUDIT_GET:
		status_set.enabled	 = audit_enabled;
		status_set.failure	 = audit_failure;
		status_set.pid		 = audit_pid;
		status_set.rate_limit	 = audit_rate_limit;
		status_set.backlog_limit = audit_backlog_limit;
		status_set.lost		 = atomic_read(&audit_lost);
		status_set.backlog	 = skb_queue_len(&audit_skb_queue);
		audit_send_reply(NETLINK_CB(skb).pid, seq, AUDIT_GET, 0, 0,
				 &status_set, sizeof(status_set));
		break;
	case AUDIT_SET:
		if (nlh->nlmsg_len < sizeof(struct audit_status))
			return -EINVAL;
		status_get   = (struct audit_status *)data;
		if (status_get->mask & AUDIT_STATUS_ENABLED) {
			err = audit_set_enabled(status_get->enabled,
						loginuid, sessionid, sid);
			if (err < 0)
				return err;
		}
		if (status_get->mask & AUDIT_STATUS_FAILURE) {
			err = audit_set_failure(status_get->failure,
						loginuid, sessionid, sid);
			if (err < 0)
				return err;
		}
		if (status_get->mask & AUDIT_STATUS_PID) {
			int new_pid = status_get->pid;

			if (audit_enabled != AUDIT_OFF)
				audit_log_config_change("audit_pid", new_pid,
							audit_pid, loginuid,
							sessionid, sid, 1);

			audit_pid = new_pid;
			audit_nlk_pid = NETLINK_CB(skb).pid;
		}
		if (status_get->mask & AUDIT_STATUS_RATE_LIMIT) {
			err = audit_set_rate_limit(status_get->rate_limit,
						   loginuid, sessionid, sid);
			if (err < 0)
				return err;
		}
		if (status_get->mask & AUDIT_STATUS_BACKLOG_LIMIT)
			err = audit_set_backlog_limit(status_get->backlog_limit,
						      loginuid, sessionid, sid);
		break;
	case AUDIT_USER:
	case AUDIT_FIRST_USER_MSG ... AUDIT_LAST_USER_MSG:
	case AUDIT_FIRST_USER_MSG2 ... AUDIT_LAST_USER_MSG2:
		if (!audit_enabled && msg_type != AUDIT_USER_AVC)
			return 0;

		err = audit_filter_user(&NETLINK_CB(skb));
		if (err == 1) {
			err = 0;
			if (msg_type == AUDIT_USER_TTY) {
				err = audit_prepare_user_tty(pid, loginuid,
							     sessionid);
				if (err)
					break;
			}
			audit_log_common_recv_msg(&ab, msg_type, pid, uid,
						  loginuid, sessionid, sid);

			if (msg_type != AUDIT_USER_TTY)
				audit_log_format(ab, " msg='%.1024s'",
						 (char *)data);
			else {
				int size;

				audit_log_format(ab, " msg=");
				size = nlmsg_len(nlh);
				if (size > 0 &&
				    ((unsigned char *)data)[size - 1] == '\0')
					size--;
				audit_log_n_untrustedstring(ab, data, size);
			}
			audit_set_pid(ab, pid);
			audit_log_end(ab);
		}
		break;
	case AUDIT_ADD:
	case AUDIT_DEL:
		if (nlmsg_len(nlh) < sizeof(struct audit_rule))
			return -EINVAL;
		if (audit_enabled == AUDIT_LOCKED) {
			audit_log_common_recv_msg(&ab, AUDIT_CONFIG_CHANGE, pid,
						  uid, loginuid, sessionid, sid);

			audit_log_format(ab, " audit_enabled=%d res=0",
					 audit_enabled);
			audit_log_end(ab);
			return -EPERM;
		}
		/* fallthrough */
	case AUDIT_LIST:
		err = audit_receive_filter(msg_type, NETLINK_CB(skb).pid,
					   uid, seq, data, nlmsg_len(nlh),
					   loginuid, sessionid, sid);
		break;
	case AUDIT_ADD_RULE:
	case AUDIT_DEL_RULE:
		if (nlmsg_len(nlh) < sizeof(struct audit_rule_data))
			return -EINVAL;
		if (audit_enabled == AUDIT_LOCKED) {
			audit_log_common_recv_msg(&ab, AUDIT_CONFIG_CHANGE, pid,
						  uid, loginuid, sessionid, sid);

			audit_log_format(ab, " audit_enabled=%d res=0",
					 audit_enabled);
			audit_log_end(ab);
			return -EPERM;
		}
		/* fallthrough */
	case AUDIT_LIST_RULES:
		err = audit_receive_filter(msg_type, NETLINK_CB(skb).pid,
					   uid, seq, data, nlmsg_len(nlh),
					   loginuid, sessionid, sid);
		break;
	case AUDIT_TRIM:
		audit_trim_trees();

		audit_log_common_recv_msg(&ab, AUDIT_CONFIG_CHANGE, pid,
					  uid, loginuid, sessionid, sid);

		audit_log_format(ab, " op=trim res=1");
		audit_log_end(ab);
		break;
	case AUDIT_MAKE_EQUIV: {
		void *bufp = data;
		u32 sizes[2];
		size_t msglen = nlmsg_len(nlh);
		char *old, *new;

		err = -EINVAL;
		if (msglen < 2 * sizeof(u32))
			break;
		memcpy(sizes, bufp, 2 * sizeof(u32));
		bufp += 2 * sizeof(u32);
		msglen -= 2 * sizeof(u32);
		old = audit_unpack_string(&bufp, &msglen, sizes[0]);
		if (IS_ERR(old)) {
			err = PTR_ERR(old);
			break;
		}
		new = audit_unpack_string(&bufp, &msglen, sizes[1]);
		if (IS_ERR(new)) {
			err = PTR_ERR(new);
			kfree(old);
			break;
		}
		/* OK, here comes... */
		err = audit_tag_tree(old, new);

		audit_log_common_recv_msg(&ab, AUDIT_CONFIG_CHANGE, pid,
					  uid, loginuid, sessionid, sid);

		audit_log_format(ab, " op=make_equiv old=");
		audit_log_untrustedstring(ab, old);
		audit_log_format(ab, " new=");
		audit_log_untrustedstring(ab, new);
		audit_log_format(ab, " res=%d", !err);
		audit_log_end(ab);
		kfree(old);
		kfree(new);
		break;
	}
	case AUDIT_SIGNAL_INFO:
		len = 0;
		if (audit_sig_sid) {
			err = security_secid_to_secctx(audit_sig_sid, &ctx, &len);
			if (err)
				return err;
		}
		sig_data = kmalloc(sizeof(*sig_data) + len, GFP_KERNEL);
		if (!sig_data) {
			if (audit_sig_sid)
				security_release_secctx(ctx, len);
			return -ENOMEM;
		}
		sig_data->uid = audit_sig_uid;
		sig_data->pid = audit_sig_pid;
		if (audit_sig_sid) {
			memcpy(sig_data->ctx, ctx, len);
			security_release_secctx(ctx, len);
		}
		audit_send_reply(NETLINK_CB(skb).pid, seq, AUDIT_SIGNAL_INFO,
				0, 0, sig_data, sizeof(*sig_data) + len);
		kfree(sig_data);
		break;
	case AUDIT_TTY_GET: {
		struct audit_tty_status s;
		struct task_struct *tsk;

		read_lock(&tasklist_lock);
		tsk = find_task_by_vpid(pid);
		if (!tsk)
			err = -ESRCH;
		else {
			spin_lock_irq(&tsk->sighand->siglock);
			s.enabled = tsk->signal->audit_tty != 0;
			spin_unlock_irq(&tsk->sighand->siglock);
		}
		read_unlock(&tasklist_lock);
		audit_send_reply(NETLINK_CB(skb).pid, seq, AUDIT_TTY_GET, 0, 0,
				 &s, sizeof(s));
		break;
	}
	case AUDIT_TTY_SET: {
		struct audit_tty_status *s;
		struct task_struct *tsk;

		if (nlh->nlmsg_len < sizeof(struct audit_tty_status))
			return -EINVAL;
		s = data;
		if (s->enabled != 0 && s->enabled != 1)
			return -EINVAL;
		read_lock(&tasklist_lock);
		tsk = find_task_by_vpid(pid);
		if (!tsk)
			err = -ESRCH;
		else {
			spin_lock_irq(&tsk->sighand->siglock);
			tsk->signal->audit_tty = s->enabled != 0;
			spin_unlock_irq(&tsk->sighand->siglock);
		}
		read_unlock(&tasklist_lock);
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

	return err < 0 ? err : 0;
}

static void audit_receive_skb(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;
	/*
	 * len MUST be signed for NLMSG_NEXT to be able to dec it below 0
	 * if the nlmsg_len was not aligned
	 */
	int len;
	int err;

	nlh = nlmsg_hdr(skb);
	len = skb->len;

	while (NLMSG_OK(nlh, len)) {
		err = audit_receive_msg(skb, nlh);
		/* if err or if this message says it wants a response */
		if (err || (nlh->nlmsg_flags & NLM_F_ACK))
			netlink_ack(skb, nlh, err);

		nlh = NLMSG_NEXT(nlh, len);
	}
}

/* Receive messages from netlink socket. */
static void audit_receive(struct sk_buff  *skb)
{
	mutex_lock(&audit_cmd_mutex);
	audit_receive_skb(skb);
	mutex_unlock(&audit_cmd_mutex);
}

/* Initialize audit support at boot time. */
static int __init audit_init(void)
{
	int i;

	if (audit_initialized == AUDIT_DISABLED)
		return 0;

	printk(KERN_INFO "audit: initializing netlink socket (%s)\n",
	       audit_default ? "enabled" : "disabled");
	audit_sock = netlink_kernel_create(&init_net, NETLINK_AUDIT, 0,
					   audit_receive, NULL, THIS_MODULE);
	if (!audit_sock)
		audit_panic("cannot initialize netlink socket");
	else
		audit_sock->sk_sndtimeo = MAX_SCHEDULE_TIMEOUT;

	skb_queue_head_init(&audit_skb_queue);
	skb_queue_head_init(&audit_skb_hold_queue);
	audit_initialized = AUDIT_INITIALIZED;
	audit_enabled = audit_default;
	audit_ever_enabled |= !!audit_default;

	audit_log(NULL, GFP_KERNEL, AUDIT_KERNEL, "initialized");

	for (i = 0; i < AUDIT_INODE_BUCKETS; i++)
		INIT_LIST_HEAD(&audit_inode_hash[i]);

	return 0;
}
__initcall(audit_init);

/* Process kernel command-line parameter at boot time.  audit=0 or audit=1. */
static int __init audit_enable(char *str)
{
	audit_default = !!simple_strtol(str, NULL, 0);
	if (!audit_default)
		audit_initialized = AUDIT_DISABLED;

	printk(KERN_INFO "audit: %s", audit_default ? "enabled" : "disabled");

	if (audit_initialized == AUDIT_INITIALIZED) {
		audit_enabled = audit_default;
		audit_ever_enabled |= !!audit_default;
	} else if (audit_initialized == AUDIT_UNINITIALIZED) {
		printk(" (after initialization)");
	} else {
		printk(" (until reboot)");
	}
	printk("\n");

	return 1;
}

__setup("audit=", audit_enable);

static void audit_buffer_free(struct audit_buffer *ab)
{
	unsigned long flags;

	if (!ab)
		return;

	if (ab->skb)
		kfree_skb(ab->skb);

	spin_lock_irqsave(&audit_freelist_lock, flags);
	if (audit_freelist_count > AUDIT_MAXFREE)
		kfree(ab);
	else {
		audit_freelist_count++;
		list_add(&ab->list, &audit_freelist);
	}
	spin_unlock_irqrestore(&audit_freelist_lock, flags);
}

static struct audit_buffer * audit_buffer_alloc(struct audit_context *ctx,
						gfp_t gfp_mask, int type)
{
	unsigned long flags;
	struct audit_buffer *ab = NULL;
	struct nlmsghdr *nlh;

	spin_lock_irqsave(&audit_freelist_lock, flags);
	if (!list_empty(&audit_freelist)) {
		ab = list_entry(audit_freelist.next,
				struct audit_buffer, list);
		list_del(&ab->list);
		--audit_freelist_count;
	}
	spin_unlock_irqrestore(&audit_freelist_lock, flags);

	if (!ab) {
		ab = kmalloc(sizeof(*ab), gfp_mask);
		if (!ab)
			goto err;
	}

	ab->ctx = ctx;
	ab->gfp_mask = gfp_mask;

	ab->skb = nlmsg_new(AUDIT_BUFSIZ, gfp_mask);
	if (!ab->skb)
		goto nlmsg_failure;

	nlh = NLMSG_NEW(ab->skb, 0, 0, type, 0, 0);

	return ab;

nlmsg_failure:                  /* Used by NLMSG_NEW */
	kfree_skb(ab->skb);
	ab->skb = NULL;
err:
	audit_buffer_free(ab);
	return NULL;
}

unsigned int audit_serial(void)
{
	static DEFINE_SPINLOCK(serial_lock);
	static unsigned int serial = 0;

	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&serial_lock, flags);
	do {
		ret = ++serial;
	} while (unlikely(!ret));
	spin_unlock_irqrestore(&serial_lock, flags);

	return ret;
}

static inline void audit_get_stamp(struct audit_context *ctx,
				   struct timespec *t, unsigned int *serial)
{
	if (!ctx || !auditsc_get_stamp(ctx, t, serial)) {
		*t = CURRENT_TIME;
		*serial = audit_serial();
	}
}


struct audit_buffer *audit_log_start(struct audit_context *ctx, gfp_t gfp_mask,
				     int type)
{
	struct audit_buffer	*ab	= NULL;
	struct timespec		t;
	unsigned int		uninitialized_var(serial);
	int reserve;
	unsigned long timeout_start = jiffies;

	if (audit_initialized != AUDIT_INITIALIZED)
		return NULL;

	if (unlikely(audit_filter_type(type)))
		return NULL;

	if (gfp_mask & __GFP_WAIT)
		reserve = 0;
	else
		reserve = 5; /* Allow atomic callers to go up to five
				entries over the normal backlog limit */

	while (audit_backlog_limit
	       && skb_queue_len(&audit_skb_queue) > audit_backlog_limit + reserve) {
		if (gfp_mask & __GFP_WAIT && audit_backlog_wait_time
		    && time_before(jiffies, timeout_start + audit_backlog_wait_time)) {

			/* Wait for auditd to drain the queue a little */
			DECLARE_WAITQUEUE(wait, current);
			set_current_state(TASK_INTERRUPTIBLE);
			add_wait_queue(&audit_backlog_wait, &wait);

			if (audit_backlog_limit &&
			    skb_queue_len(&audit_skb_queue) > audit_backlog_limit)
				schedule_timeout(timeout_start + audit_backlog_wait_time - jiffies);

			__set_current_state(TASK_RUNNING);
			remove_wait_queue(&audit_backlog_wait, &wait);
			continue;
		}
		if (audit_rate_check() && printk_ratelimit())
			printk(KERN_WARNING
			       "audit: audit_backlog=%d > "
			       "audit_backlog_limit=%d\n",
			       skb_queue_len(&audit_skb_queue),
			       audit_backlog_limit);
		audit_log_lost("backlog limit exceeded");
		audit_backlog_wait_time = audit_backlog_wait_overflow;
		wake_up(&audit_backlog_wait);
		return NULL;
	}

	ab = audit_buffer_alloc(ctx, gfp_mask, type);
	if (!ab) {
		audit_log_lost("out of memory in audit_log_start");
		return NULL;
	}

	audit_get_stamp(ab->ctx, &t, &serial);

	audit_log_format(ab, "audit(%lu.%03lu:%u): ",
			 t.tv_sec, t.tv_nsec/1000000, serial);
	return ab;
}

static inline int audit_expand(struct audit_buffer *ab, int extra)
{
	struct sk_buff *skb = ab->skb;
	int oldtail = skb_tailroom(skb);
	int ret = pskb_expand_head(skb, 0, extra, ab->gfp_mask);
	int newtail = skb_tailroom(skb);

	if (ret < 0) {
		audit_log_lost("out of memory in audit_expand");
		return 0;
	}

	skb->truesize += newtail - oldtail;
	return newtail;
}

static void audit_log_vformat(struct audit_buffer *ab, const char *fmt,
			      va_list args)
{
	int len, avail;
	struct sk_buff *skb;
	va_list args2;

	if (!ab)
		return;

	BUG_ON(!ab->skb);
	skb = ab->skb;
	avail = skb_tailroom(skb);
	if (avail == 0) {
		avail = audit_expand(ab, AUDIT_BUFSIZ);
		if (!avail)
			goto out;
	}
	va_copy(args2, args);
	len = vsnprintf(skb_tail_pointer(skb), avail, fmt, args);
	if (len >= avail) {
		/* The printk buffer is 1024 bytes long, so if we get
		 * here and AUDIT_BUFSIZ is at least 1024, then we can
		 * log everything that printk could have logged. */
		avail = audit_expand(ab,
			max_t(unsigned, AUDIT_BUFSIZ, 1+len-avail));
		if (!avail)
			goto out;
		len = vsnprintf(skb_tail_pointer(skb), avail, fmt, args2);
	}
	va_end(args2);
	if (len > 0)
		skb_put(skb, len);
out:
	return;
}

void audit_log_format(struct audit_buffer *ab, const char *fmt, ...)
{
	va_list args;

	if (!ab)
		return;
	va_start(args, fmt);
	audit_log_vformat(ab, fmt, args);
	va_end(args);
}

void audit_log_n_hex(struct audit_buffer *ab, const unsigned char *buf,
		size_t len)
{
	int i, avail, new_len;
	unsigned char *ptr;
	struct sk_buff *skb;
	static const unsigned char *hex = "0123456789ABCDEF";

	if (!ab)
		return;

	BUG_ON(!ab->skb);
	skb = ab->skb;
	avail = skb_tailroom(skb);
	new_len = len<<1;
	if (new_len >= avail) {
		/* Round the buffer request up to the next multiple */
		new_len = AUDIT_BUFSIZ*(((new_len-avail)/AUDIT_BUFSIZ) + 1);
		avail = audit_expand(ab, new_len);
		if (!avail)
			return;
	}

	ptr = skb_tail_pointer(skb);
	for (i=0; i<len; i++) {
		*ptr++ = hex[(buf[i] & 0xF0)>>4]; /* Upper nibble */
		*ptr++ = hex[buf[i] & 0x0F];	  /* Lower nibble */
	}
	*ptr = 0;
	skb_put(skb, len << 1); /* new string is twice the old string */
}

void audit_log_n_string(struct audit_buffer *ab, const char *string,
			size_t slen)
{
	int avail, new_len;
	unsigned char *ptr;
	struct sk_buff *skb;

	if (!ab)
		return;

	BUG_ON(!ab->skb);
	skb = ab->skb;
	avail = skb_tailroom(skb);
	new_len = slen + 3;	/* enclosing quotes + null terminator */
	if (new_len > avail) {
		avail = audit_expand(ab, new_len);
		if (!avail)
			return;
	}
	ptr = skb_tail_pointer(skb);
	*ptr++ = '"';
	memcpy(ptr, string, slen);
	ptr += slen;
	*ptr++ = '"';
	*ptr = 0;
	skb_put(skb, slen + 2);	/* don't include null terminator */
}

int audit_string_contains_control(const char *string, size_t len)
{
	const unsigned char *p;
	for (p = string; p < (const unsigned char *)string + len; p++) {
		if (*p == '"' || *p < 0x21 || *p > 0x7e)
			return 1;
	}
	return 0;
}

void audit_log_n_untrustedstring(struct audit_buffer *ab, const char *string,
				 size_t len)
{
	if (audit_string_contains_control(string, len))
		audit_log_n_hex(ab, string, len);
	else
		audit_log_n_string(ab, string, len);
}

void audit_log_untrustedstring(struct audit_buffer *ab, const char *string)
{
	audit_log_n_untrustedstring(ab, string, strlen(string));
}

/* This is a helper-function to print the escaped d_path */
void audit_log_d_path(struct audit_buffer *ab, const char *prefix,
		      struct path *path)
{
	char *p, *pathname;

	if (prefix)
		audit_log_format(ab, " %s", prefix);

	/* We will allow 11 spaces for ' (deleted)' to be appended */
	pathname = kmalloc(PATH_MAX+11, ab->gfp_mask);
	if (!pathname) {
		audit_log_string(ab, "<no_memory>");
		return;
	}
	p = d_path(path, pathname, PATH_MAX+11);
	if (IS_ERR(p)) { /* Should never happen since we send PATH_MAX */
		/* FIXME: can we save some information here? */
		audit_log_string(ab, "<too_long>");
	} else
		audit_log_untrustedstring(ab, p);
	kfree(pathname);
}

void audit_log_key(struct audit_buffer *ab, char *key)
{
	audit_log_format(ab, " key=");
	if (key)
		audit_log_untrustedstring(ab, key);
	else
		audit_log_format(ab, "(null)");
}

void audit_log_end(struct audit_buffer *ab)
{
	if (!ab)
		return;
	if (!audit_rate_check()) {
		audit_log_lost("rate limit exceeded");
	} else {
		struct nlmsghdr *nlh = nlmsg_hdr(ab->skb);
		nlh->nlmsg_len = ab->skb->len - NLMSG_SPACE(0);

		if (audit_pid) {
			skb_queue_tail(&audit_skb_queue, ab->skb);
			wake_up_interruptible(&kauditd_wait);
		} else {
			audit_printk_skb(ab->skb);
		}
		ab->skb = NULL;
	}
	audit_buffer_free(ab);
}

void audit_log(struct audit_context *ctx, gfp_t gfp_mask, int type,
	       const char *fmt, ...)
{
	struct audit_buffer *ab;
	va_list args;

	ab = audit_log_start(ctx, gfp_mask, type);
	if (ab) {
		va_start(args, fmt);
		audit_log_vformat(ab, fmt, args);
		va_end(args);
		audit_log_end(ab);
	}
}

EXPORT_SYMBOL(audit_log_start);
EXPORT_SYMBOL(audit_log_end);
EXPORT_SYMBOL(audit_log_format);
EXPORT_SYMBOL(audit_log);
