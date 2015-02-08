
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/cpuset.h>
#include <linux/unistd.h>
#include <linux/file.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <trace/events/sched.h>

static DEFINE_SPINLOCK(kthread_create_lock);
static LIST_HEAD(kthread_create_list);
struct task_struct *kthreadd_task;

struct kthread_create_info
{
	/* Information passed to kthread() from kthreadd. */
	int (*threadfn)(void *data);
	void *data;

	/* Result passed back to kthread_create() from kthreadd. */
	struct task_struct *result;
	struct completion done;

	struct list_head list;
};

struct kthread {
	int should_stop;
	struct completion exited;
};

#define to_kthread(tsk)	\
	container_of((tsk)->vfork_done, struct kthread, exited)

int kthread_should_stop(void)
{
	return to_kthread(current)->should_stop;
}
EXPORT_SYMBOL(kthread_should_stop);

static int kthread(void *_create)
{
	/* Copy data: it's on kthread's stack */
	struct kthread_create_info *create = _create;
	int (*threadfn)(void *data) = create->threadfn;
	void *data = create->data;
	struct kthread self;
	int ret;

	self.should_stop = 0;
	init_completion(&self.exited);
	current->vfork_done = &self.exited;

	/* OK, tell user we're spawned, wait for stop or wakeup */
	__set_current_state(TASK_UNINTERRUPTIBLE);
	create->result = current;
	complete(&create->done);
	schedule();

	ret = -EINTR;
	if (!self.should_stop)
		ret = threadfn(data);

	/* we can't just return, we must preserve "self" on stack */
	do_exit(ret);
}

static void create_kthread(struct kthread_create_info *create)
{
	int pid;

	/* We want our own signal handler (we take no signals by default). */
	pid = kernel_thread(kthread, create, CLONE_FS | CLONE_FILES | SIGCHLD);
	if (pid < 0) {
		create->result = ERR_PTR(pid);
		complete(&create->done);
	}
}

struct task_struct *kthread_create(int (*threadfn)(void *data),
				   void *data,
				   const char namefmt[],
				   ...)
{
	struct kthread_create_info create;

	create.threadfn = threadfn;
	create.data = data;
	init_completion(&create.done);

	spin_lock(&kthread_create_lock);
	list_add_tail(&create.list, &kthread_create_list);
	spin_unlock(&kthread_create_lock);

	wake_up_process(kthreadd_task);
	wait_for_completion(&create.done);

	if (!IS_ERR(create.result)) {
		struct sched_param param = { .sched_priority = 0 };
		va_list args;

		va_start(args, namefmt);
		vsnprintf(create.result->comm, sizeof(create.result->comm),
			  namefmt, args);
		va_end(args);
		/*
		 * root may have changed our (kthreadd's) priority or CPU mask.
		 * The kernel thread should not inherit these properties.
		 */
		sched_setscheduler_nocheck(create.result, SCHED_NORMAL, &param);
		set_cpus_allowed_ptr(create.result, cpu_all_mask);
	}
	return create.result;
}
EXPORT_SYMBOL(kthread_create);

void kthread_bind(struct task_struct *p, unsigned int cpu)
{
	/* Must have done schedule() in kthread() before we set_task_cpu */
	if (!wait_task_inactive(p, TASK_UNINTERRUPTIBLE)) {
		WARN_ON(1);
		return;
	}

	p->cpus_allowed = cpumask_of_cpu(cpu);
	p->rt.nr_cpus_allowed = 1;
	p->flags |= PF_THREAD_BOUND;
}
EXPORT_SYMBOL(kthread_bind);

int kthread_stop(struct task_struct *k)
{
	struct kthread *kthread;
	int ret;

	trace_sched_kthread_stop(k);
	get_task_struct(k);

	kthread = to_kthread(k);
	barrier(); /* it might have exited */
	if (k->vfork_done != NULL) {
		kthread->should_stop = 1;
		wake_up_process(k);
		wait_for_completion(&kthread->exited);
	}
	ret = k->exit_code;

	put_task_struct(k);
	trace_sched_kthread_stop_ret(ret);

	return ret;
}
EXPORT_SYMBOL(kthread_stop);

int kthreadd(void *unused)
{
	struct task_struct *tsk = current;

	/* Setup a clean context for our children to inherit. */
	set_task_comm(tsk, "kthreadd");
	ignore_signals(tsk);
	set_cpus_allowed_ptr(tsk, cpu_all_mask);
	set_mems_allowed(node_states[N_HIGH_MEMORY]);

	current->flags |= PF_NOFREEZE | PF_FREEZER_NOSIG;

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (list_empty(&kthread_create_list))
			schedule();
		__set_current_state(TASK_RUNNING);

		spin_lock(&kthread_create_lock);
		while (!list_empty(&kthread_create_list)) {
			struct kthread_create_info *create;

			create = list_entry(kthread_create_list.next,
					    struct kthread_create_info, list);
			list_del_init(&create->list);
			spin_unlock(&kthread_create_lock);

			create_kthread(create);

			spin_lock(&kthread_create_lock);
		}
		spin_unlock(&kthread_create_lock);
	}

	return 0;
}
