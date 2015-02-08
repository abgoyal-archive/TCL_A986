

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpuset.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/list.h>
#include <linux/mempolicy.h>
#include <linux/mm.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/pagemap.h>
#include <linux/proc_fs.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/security.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/backing-dev.h>
#include <linux/sort.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/cgroup.h>

static struct workqueue_struct *cpuset_wq;

int number_of_cpusets __read_mostly;

/* Forward declare cgroup structures */
struct cgroup_subsys cpuset_subsys;
struct cpuset;

/* See "Frequency meter" comments, below. */

struct fmeter {
	int cnt;		/* unprocessed events count */
	int val;		/* most recent output value */
	time_t time;		/* clock (secs) when val computed */
	spinlock_t lock;	/* guards read or write of above */
};

struct cpuset {
	struct cgroup_subsys_state css;

	unsigned long flags;		/* "unsigned long" so bitops work */
	cpumask_var_t cpus_allowed;	/* CPUs allowed to tasks in cpuset */
	nodemask_t mems_allowed;	/* Memory Nodes allowed to tasks */

	struct cpuset *parent;		/* my parent */

	struct fmeter fmeter;		/* memory_pressure filter */

	/* partition number for rebuild_sched_domains() */
	int pn;

	/* for custom sched domain */
	int relax_domain_level;

	/* used for walking a cpuset heirarchy */
	struct list_head stack_list;
};

/* Retrieve the cpuset for a cgroup */
static inline struct cpuset *cgroup_cs(struct cgroup *cont)
{
	return container_of(cgroup_subsys_state(cont, cpuset_subsys_id),
			    struct cpuset, css);
}

/* Retrieve the cpuset for a task */
static inline struct cpuset *task_cs(struct task_struct *task)
{
	return container_of(task_subsys_state(task, cpuset_subsys_id),
			    struct cpuset, css);
}

/* bits in struct cpuset flags field */
typedef enum {
	CS_CPU_EXCLUSIVE,
	CS_MEM_EXCLUSIVE,
	CS_MEM_HARDWALL,
	CS_MEMORY_MIGRATE,
	CS_SCHED_LOAD_BALANCE,
	CS_SPREAD_PAGE,
	CS_SPREAD_SLAB,
} cpuset_flagbits_t;

/* convenient tests for these bits */
static inline int is_cpu_exclusive(const struct cpuset *cs)
{
	return test_bit(CS_CPU_EXCLUSIVE, &cs->flags);
}

static inline int is_mem_exclusive(const struct cpuset *cs)
{
	return test_bit(CS_MEM_EXCLUSIVE, &cs->flags);
}

static inline int is_mem_hardwall(const struct cpuset *cs)
{
	return test_bit(CS_MEM_HARDWALL, &cs->flags);
}

static inline int is_sched_load_balance(const struct cpuset *cs)
{
	return test_bit(CS_SCHED_LOAD_BALANCE, &cs->flags);
}

static inline int is_memory_migrate(const struct cpuset *cs)
{
	return test_bit(CS_MEMORY_MIGRATE, &cs->flags);
}

static inline int is_spread_page(const struct cpuset *cs)
{
	return test_bit(CS_SPREAD_PAGE, &cs->flags);
}

static inline int is_spread_slab(const struct cpuset *cs)
{
	return test_bit(CS_SPREAD_SLAB, &cs->flags);
}

static struct cpuset top_cpuset = {
	.flags = ((1 << CS_CPU_EXCLUSIVE) | (1 << CS_MEM_EXCLUSIVE)),
};


static DEFINE_MUTEX(callback_mutex);

#define CPUSET_NAME_LEN		(128)
#define	CPUSET_NODELIST_LEN	(256)
static char cpuset_name[CPUSET_NAME_LEN];
static char cpuset_nodelist[CPUSET_NODELIST_LEN];
static DEFINE_SPINLOCK(cpuset_buffer_lock);

static int cpuset_get_sb(struct file_system_type *fs_type,
			 int flags, const char *unused_dev_name,
			 void *data, struct vfsmount *mnt)
{
	struct file_system_type *cgroup_fs = get_fs_type("cgroup");
	int ret = -ENODEV;
	if (cgroup_fs) {
		char mountopts[] =
			"cpuset,noprefix,"
			"release_agent=/sbin/cpuset_release_agent";
		ret = cgroup_fs->get_sb(cgroup_fs, flags,
					   unused_dev_name, mountopts, mnt);
		put_filesystem(cgroup_fs);
	}
	return ret;
}

static struct file_system_type cpuset_fs_type = {
	.name = "cpuset",
	.get_sb = cpuset_get_sb,
};


static void guarantee_online_cpus(const struct cpuset *cs,
				  struct cpumask *pmask)
{
	while (cs && !cpumask_intersects(cs->cpus_allowed, cpu_online_mask))
		cs = cs->parent;
	if (cs)
		cpumask_and(pmask, cs->cpus_allowed, cpu_online_mask);
	else
		cpumask_copy(pmask, cpu_online_mask);
	BUG_ON(!cpumask_intersects(pmask, cpu_online_mask));
}


static void guarantee_online_mems(const struct cpuset *cs, nodemask_t *pmask)
{
	while (cs && !nodes_intersects(cs->mems_allowed,
					node_states[N_HIGH_MEMORY]))
		cs = cs->parent;
	if (cs)
		nodes_and(*pmask, cs->mems_allowed,
					node_states[N_HIGH_MEMORY]);
	else
		*pmask = node_states[N_HIGH_MEMORY];
	BUG_ON(!nodes_intersects(*pmask, node_states[N_HIGH_MEMORY]));
}

static void cpuset_update_task_spread_flag(struct cpuset *cs,
					struct task_struct *tsk)
{
	if (is_spread_page(cs))
		tsk->flags |= PF_SPREAD_PAGE;
	else
		tsk->flags &= ~PF_SPREAD_PAGE;
	if (is_spread_slab(cs))
		tsk->flags |= PF_SPREAD_SLAB;
	else
		tsk->flags &= ~PF_SPREAD_SLAB;
}


static int is_cpuset_subset(const struct cpuset *p, const struct cpuset *q)
{
	return	cpumask_subset(p->cpus_allowed, q->cpus_allowed) &&
		nodes_subset(p->mems_allowed, q->mems_allowed) &&
		is_cpu_exclusive(p) <= is_cpu_exclusive(q) &&
		is_mem_exclusive(p) <= is_mem_exclusive(q);
}

static struct cpuset *alloc_trial_cpuset(const struct cpuset *cs)
{
	struct cpuset *trial;

	trial = kmemdup(cs, sizeof(*cs), GFP_KERNEL);
	if (!trial)
		return NULL;

	if (!alloc_cpumask_var(&trial->cpus_allowed, GFP_KERNEL)) {
		kfree(trial);
		return NULL;
	}
	cpumask_copy(trial->cpus_allowed, cs->cpus_allowed);

	return trial;
}

static void free_trial_cpuset(struct cpuset *trial)
{
	free_cpumask_var(trial->cpus_allowed);
	kfree(trial);
}


static int validate_change(const struct cpuset *cur, const struct cpuset *trial)
{
	struct cgroup *cont;
	struct cpuset *c, *par;

	/* Each of our child cpusets must be a subset of us */
	list_for_each_entry(cont, &cur->css.cgroup->children, sibling) {
		if (!is_cpuset_subset(cgroup_cs(cont), trial))
			return -EBUSY;
	}

	/* Remaining checks don't apply to root cpuset */
	if (cur == &top_cpuset)
		return 0;

	par = cur->parent;

	/* We must be a subset of our parent cpuset */
	if (!is_cpuset_subset(trial, par))
		return -EACCES;

	/*
	 * If either I or some sibling (!= me) is exclusive, we can't
	 * overlap
	 */
	list_for_each_entry(cont, &par->css.cgroup->children, sibling) {
		c = cgroup_cs(cont);
		if ((is_cpu_exclusive(trial) || is_cpu_exclusive(c)) &&
		    c != cur &&
		    cpumask_intersects(trial->cpus_allowed, c->cpus_allowed))
			return -EINVAL;
		if ((is_mem_exclusive(trial) || is_mem_exclusive(c)) &&
		    c != cur &&
		    nodes_intersects(trial->mems_allowed, c->mems_allowed))
			return -EINVAL;
	}

	/* Cpusets with tasks can't have empty cpus_allowed or mems_allowed */
	if (cgroup_task_count(cur->css.cgroup)) {
		if (cpumask_empty(trial->cpus_allowed) ||
		    nodes_empty(trial->mems_allowed)) {
			return -ENOSPC;
		}
	}

	return 0;
}

#ifdef CONFIG_SMP
static int cpusets_overlap(struct cpuset *a, struct cpuset *b)
{
	return cpumask_intersects(a->cpus_allowed, b->cpus_allowed);
}

static void
update_domain_attr(struct sched_domain_attr *dattr, struct cpuset *c)
{
	if (dattr->relax_domain_level < c->relax_domain_level)
		dattr->relax_domain_level = c->relax_domain_level;
	return;
}

static void
update_domain_attr_tree(struct sched_domain_attr *dattr, struct cpuset *c)
{
	LIST_HEAD(q);

	list_add(&c->stack_list, &q);
	while (!list_empty(&q)) {
		struct cpuset *cp;
		struct cgroup *cont;
		struct cpuset *child;

		cp = list_first_entry(&q, struct cpuset, stack_list);
		list_del(q.next);

		if (cpumask_empty(cp->cpus_allowed))
			continue;

		if (is_sched_load_balance(cp))
			update_domain_attr(dattr, cp);

		list_for_each_entry(cont, &cp->css.cgroup->children, sibling) {
			child = cgroup_cs(cont);
			list_add_tail(&child->stack_list, &q);
		}
	}
}

static int generate_sched_domains(cpumask_var_t **domains,
			struct sched_domain_attr **attributes)
{
	LIST_HEAD(q);		/* queue of cpusets to be scanned */
	struct cpuset *cp;	/* scans q */
	struct cpuset **csa;	/* array of all cpuset ptrs */
	int csn;		/* how many cpuset ptrs in csa so far */
	int i, j, k;		/* indices for partition finding loops */
	cpumask_var_t *doms;	/* resulting partition; i.e. sched domains */
	struct sched_domain_attr *dattr;  /* attributes for custom domains */
	int ndoms = 0;		/* number of sched domains in result */
	int nslot;		/* next empty doms[] struct cpumask slot */

	doms = NULL;
	dattr = NULL;
	csa = NULL;

	/* Special case for the 99% of systems with one, full, sched domain */
	if (is_sched_load_balance(&top_cpuset)) {
		ndoms = 1;
		doms = alloc_sched_domains(ndoms);
		if (!doms)
			goto done;

		dattr = kmalloc(sizeof(struct sched_domain_attr), GFP_KERNEL);
		if (dattr) {
			*dattr = SD_ATTR_INIT;
			update_domain_attr_tree(dattr, &top_cpuset);
		}
		cpumask_copy(doms[0], top_cpuset.cpus_allowed);

		goto done;
	}

	csa = kmalloc(number_of_cpusets * sizeof(cp), GFP_KERNEL);
	if (!csa)
		goto done;
	csn = 0;

	list_add(&top_cpuset.stack_list, &q);
	while (!list_empty(&q)) {
		struct cgroup *cont;
		struct cpuset *child;   /* scans child cpusets of cp */

		cp = list_first_entry(&q, struct cpuset, stack_list);
		list_del(q.next);

		if (cpumask_empty(cp->cpus_allowed))
			continue;

		/*
		 * All child cpusets contain a subset of the parent's cpus, so
		 * just skip them, and then we call update_domain_attr_tree()
		 * to calc relax_domain_level of the corresponding sched
		 * domain.
		 */
		if (is_sched_load_balance(cp)) {
			csa[csn++] = cp;
			continue;
		}

		list_for_each_entry(cont, &cp->css.cgroup->children, sibling) {
			child = cgroup_cs(cont);
			list_add_tail(&child->stack_list, &q);
		}
  	}

	for (i = 0; i < csn; i++)
		csa[i]->pn = i;
	ndoms = csn;

restart:
	/* Find the best partition (set of sched domains) */
	for (i = 0; i < csn; i++) {
		struct cpuset *a = csa[i];
		int apn = a->pn;

		for (j = 0; j < csn; j++) {
			struct cpuset *b = csa[j];
			int bpn = b->pn;

			if (apn != bpn && cpusets_overlap(a, b)) {
				for (k = 0; k < csn; k++) {
					struct cpuset *c = csa[k];

					if (c->pn == bpn)
						c->pn = apn;
				}
				ndoms--;	/* one less element */
				goto restart;
			}
		}
	}

	/*
	 * Now we know how many domains to create.
	 * Convert <csn, csa> to <ndoms, doms> and populate cpu masks.
	 */
	doms = alloc_sched_domains(ndoms);
	if (!doms)
		goto done;

	/*
	 * The rest of the code, including the scheduler, can deal with
	 * dattr==NULL case. No need to abort if alloc fails.
	 */
	dattr = kmalloc(ndoms * sizeof(struct sched_domain_attr), GFP_KERNEL);

	for (nslot = 0, i = 0; i < csn; i++) {
		struct cpuset *a = csa[i];
		struct cpumask *dp;
		int apn = a->pn;

		if (apn < 0) {
			/* Skip completed partitions */
			continue;
		}

		dp = doms[nslot];

		if (nslot == ndoms) {
			static int warnings = 10;
			if (warnings) {
				printk(KERN_WARNING
				 "rebuild_sched_domains confused:"
				  " nslot %d, ndoms %d, csn %d, i %d,"
				  " apn %d\n",
				  nslot, ndoms, csn, i, apn);
				warnings--;
			}
			continue;
		}

		cpumask_clear(dp);
		if (dattr)
			*(dattr + nslot) = SD_ATTR_INIT;
		for (j = i; j < csn; j++) {
			struct cpuset *b = csa[j];

			if (apn == b->pn) {
				cpumask_or(dp, dp, b->cpus_allowed);
				if (dattr)
					update_domain_attr_tree(dattr + nslot, b);

				/* Done with this partition */
				b->pn = -1;
			}
		}
		nslot++;
	}
	BUG_ON(nslot != ndoms);

done:
	kfree(csa);

	/*
	 * Fallback to the default domain if kmalloc() failed.
	 * See comments in partition_sched_domains().
	 */
	if (doms == NULL)
		ndoms = 1;

	*domains    = doms;
	*attributes = dattr;
	return ndoms;
}

static void do_rebuild_sched_domains(struct work_struct *unused)
{
	struct sched_domain_attr *attr;
	cpumask_var_t *doms;
	int ndoms;

	get_online_cpus();

	/* Generate domain masks and attrs */
	cgroup_lock();
	ndoms = generate_sched_domains(&doms, &attr);
	cgroup_unlock();

	/* Have scheduler rebuild the domains */
	partition_sched_domains(ndoms, doms, attr);

	put_online_cpus();
}
#else /* !CONFIG_SMP */
static void do_rebuild_sched_domains(struct work_struct *unused)
{
}

static int generate_sched_domains(cpumask_var_t **domains,
			struct sched_domain_attr **attributes)
{
	*domains = NULL;
	return 1;
}
#endif /* CONFIG_SMP */

static DECLARE_WORK(rebuild_sched_domains_work, do_rebuild_sched_domains);

static void async_rebuild_sched_domains(void)
{
	queue_work(cpuset_wq, &rebuild_sched_domains_work);
}

void rebuild_sched_domains(void)
{
	do_rebuild_sched_domains(NULL);
}

static int cpuset_test_cpumask(struct task_struct *tsk,
			       struct cgroup_scanner *scan)
{
	return !cpumask_equal(&tsk->cpus_allowed,
			(cgroup_cs(scan->cg))->cpus_allowed);
}

static void cpuset_change_cpumask(struct task_struct *tsk,
				  struct cgroup_scanner *scan)
{
	set_cpus_allowed_ptr(tsk, ((cgroup_cs(scan->cg))->cpus_allowed));
}

static void update_tasks_cpumask(struct cpuset *cs, struct ptr_heap *heap)
{
	struct cgroup_scanner scan;

	scan.cg = cs->css.cgroup;
	scan.test_task = cpuset_test_cpumask;
	scan.process_task = cpuset_change_cpumask;
	scan.heap = heap;
	cgroup_scan_tasks(&scan);
}

static int update_cpumask(struct cpuset *cs, struct cpuset *trialcs,
			  const char *buf)
{
	struct ptr_heap heap;
	int retval;
	int is_load_balanced;

	/* top_cpuset.cpus_allowed tracks cpu_online_map; it's read-only */
	if (cs == &top_cpuset)
		return -EACCES;

	/*
	 * An empty cpus_allowed is ok only if the cpuset has no tasks.
	 * Since cpulist_parse() fails on an empty mask, we special case
	 * that parsing.  The validate_change() call ensures that cpusets
	 * with tasks have cpus.
	 */
	if (!*buf) {
		cpumask_clear(trialcs->cpus_allowed);
	} else {
		retval = cpulist_parse(buf, trialcs->cpus_allowed);
		if (retval < 0)
			return retval;

		if (!cpumask_subset(trialcs->cpus_allowed, cpu_active_mask))
			return -EINVAL;
	}
	retval = validate_change(cs, trialcs);
	if (retval < 0)
		return retval;

	/* Nothing to do if the cpus didn't change */
	if (cpumask_equal(cs->cpus_allowed, trialcs->cpus_allowed))
		return 0;

	retval = heap_init(&heap, PAGE_SIZE, GFP_KERNEL, NULL);
	if (retval)
		return retval;

	is_load_balanced = is_sched_load_balance(trialcs);

	mutex_lock(&callback_mutex);
	cpumask_copy(cs->cpus_allowed, trialcs->cpus_allowed);
	mutex_unlock(&callback_mutex);

	/*
	 * Scan tasks in the cpuset, and update the cpumasks of any
	 * that need an update.
	 */
	update_tasks_cpumask(cs, &heap);

	heap_free(&heap);

	if (is_load_balanced)
		async_rebuild_sched_domains();
	return 0;
}


static void cpuset_migrate_mm(struct mm_struct *mm, const nodemask_t *from,
							const nodemask_t *to)
{
	struct task_struct *tsk = current;

	tsk->mems_allowed = *to;

	do_migrate_pages(mm, from, to, MPOL_MF_MOVE_ALL);

	guarantee_online_mems(task_cs(tsk),&tsk->mems_allowed);
}

static void cpuset_change_task_nodemask(struct task_struct *tsk,
					nodemask_t *newmems)
{
repeat:
	/*
	 * Allow tasks that have access to memory reserves because they have
	 * been OOM killed to get memory anywhere.
	 */
	if (unlikely(test_thread_flag(TIF_MEMDIE)))
		return;
	if (current->flags & PF_EXITING) /* Let dying task have memory */
		return;

	task_lock(tsk);
	nodes_or(tsk->mems_allowed, tsk->mems_allowed, *newmems);
	mpol_rebind_task(tsk, newmems, MPOL_REBIND_STEP1);


	/*
	 * ensure checking ->mems_allowed_change_disable after setting all new
	 * allowed nodes.
	 *
	 * the read-side task can see an nodemask with new allowed nodes and
	 * old allowed nodes. and if it allocates page when cpuset clears newly
	 * disallowed ones continuous, it can see the new allowed bits.
	 *
	 * And if setting all new allowed nodes is after the checking, setting
	 * all new allowed nodes and clearing newly disallowed ones will be done
	 * continuous, and the read-side task may find no node to alloc page.
	 */
	smp_mb();

	/*
	 * Allocation of memory is very fast, we needn't sleep when waiting
	 * for the read-side.
	 */
	while (ACCESS_ONCE(tsk->mems_allowed_change_disable)) {
		task_unlock(tsk);
		if (!task_curr(tsk))
			yield();
		goto repeat;
	}

	/*
	 * ensure checking ->mems_allowed_change_disable before clearing all new
	 * disallowed nodes.
	 *
	 * if clearing newly disallowed bits before the checking, the read-side
	 * task may find no node to alloc page.
	 */
	smp_mb();

	mpol_rebind_task(tsk, newmems, MPOL_REBIND_STEP2);
	tsk->mems_allowed = *newmems;
	task_unlock(tsk);
}

static void cpuset_change_nodemask(struct task_struct *p,
				   struct cgroup_scanner *scan)
{
	struct mm_struct *mm;
	struct cpuset *cs;
	int migrate;
	const nodemask_t *oldmem = scan->data;
	NODEMASK_ALLOC(nodemask_t, newmems, GFP_KERNEL);

	if (!newmems)
		return;

	cs = cgroup_cs(scan->cg);
	guarantee_online_mems(cs, newmems);

	cpuset_change_task_nodemask(p, newmems);

	NODEMASK_FREE(newmems);

	mm = get_task_mm(p);
	if (!mm)
		return;

	migrate = is_memory_migrate(cs);

	mpol_rebind_mm(mm, &cs->mems_allowed);
	if (migrate)
		cpuset_migrate_mm(mm, oldmem, &cs->mems_allowed);
	mmput(mm);
}

static void *cpuset_being_rebound;

static void update_tasks_nodemask(struct cpuset *cs, const nodemask_t *oldmem,
				 struct ptr_heap *heap)
{
	struct cgroup_scanner scan;

	cpuset_being_rebound = cs;		/* causes mpol_dup() rebind */

	scan.cg = cs->css.cgroup;
	scan.test_task = NULL;
	scan.process_task = cpuset_change_nodemask;
	scan.heap = heap;
	scan.data = (nodemask_t *)oldmem;

	/*
	 * The mpol_rebind_mm() call takes mmap_sem, which we couldn't
	 * take while holding tasklist_lock.  Forks can happen - the
	 * mpol_dup() cpuset_being_rebound check will catch such forks,
	 * and rebind their vma mempolicies too.  Because we still hold
	 * the global cgroup_mutex, we know that no other rebind effort
	 * will be contending for the global variable cpuset_being_rebound.
	 * It's ok if we rebind the same mm twice; mpol_rebind_mm()
	 * is idempotent.  Also migrate pages in each mm to new nodes.
	 */
	cgroup_scan_tasks(&scan);

	/* We're done rebinding vmas to this cpuset's new mems_allowed. */
	cpuset_being_rebound = NULL;
}

static int update_nodemask(struct cpuset *cs, struct cpuset *trialcs,
			   const char *buf)
{
	NODEMASK_ALLOC(nodemask_t, oldmem, GFP_KERNEL);
	int retval;
	struct ptr_heap heap;

	if (!oldmem)
		return -ENOMEM;

	/*
	 * top_cpuset.mems_allowed tracks node_stats[N_HIGH_MEMORY];
	 * it's read-only
	 */
	if (cs == &top_cpuset) {
		retval = -EACCES;
		goto done;
	}

	/*
	 * An empty mems_allowed is ok iff there are no tasks in the cpuset.
	 * Since nodelist_parse() fails on an empty mask, we special case
	 * that parsing.  The validate_change() call ensures that cpusets
	 * with tasks have memory.
	 */
	if (!*buf) {
		nodes_clear(trialcs->mems_allowed);
	} else {
		retval = nodelist_parse(buf, trialcs->mems_allowed);
		if (retval < 0)
			goto done;

		if (!nodes_subset(trialcs->mems_allowed,
				node_states[N_HIGH_MEMORY])) {
			retval =  -EINVAL;
			goto done;
		}
	}
	*oldmem = cs->mems_allowed;
	if (nodes_equal(*oldmem, trialcs->mems_allowed)) {
		retval = 0;		/* Too easy - nothing to do */
		goto done;
	}
	retval = validate_change(cs, trialcs);
	if (retval < 0)
		goto done;

	retval = heap_init(&heap, PAGE_SIZE, GFP_KERNEL, NULL);
	if (retval < 0)
		goto done;

	mutex_lock(&callback_mutex);
	cs->mems_allowed = trialcs->mems_allowed;
	mutex_unlock(&callback_mutex);

	update_tasks_nodemask(cs, oldmem, &heap);

	heap_free(&heap);
done:
	NODEMASK_FREE(oldmem);
	return retval;
}

int current_cpuset_is_being_rebound(void)
{
	return task_cs(current) == cpuset_being_rebound;
}

static int update_relax_domain_level(struct cpuset *cs, s64 val)
{
#ifdef CONFIG_SMP
	if (val < -1 || val >= SD_LV_MAX)
		return -EINVAL;
#endif

	if (val != cs->relax_domain_level) {
		cs->relax_domain_level = val;
		if (!cpumask_empty(cs->cpus_allowed) &&
		    is_sched_load_balance(cs))
			async_rebuild_sched_domains();
	}

	return 0;
}

static void cpuset_change_flag(struct task_struct *tsk,
				struct cgroup_scanner *scan)
{
	cpuset_update_task_spread_flag(cgroup_cs(scan->cg), tsk);
}

static void update_tasks_flags(struct cpuset *cs, struct ptr_heap *heap)
{
	struct cgroup_scanner scan;

	scan.cg = cs->css.cgroup;
	scan.test_task = NULL;
	scan.process_task = cpuset_change_flag;
	scan.heap = heap;
	cgroup_scan_tasks(&scan);
}


static int update_flag(cpuset_flagbits_t bit, struct cpuset *cs,
		       int turning_on)
{
	struct cpuset *trialcs;
	int balance_flag_changed;
	int spread_flag_changed;
	struct ptr_heap heap;
	int err;

	trialcs = alloc_trial_cpuset(cs);
	if (!trialcs)
		return -ENOMEM;

	if (turning_on)
		set_bit(bit, &trialcs->flags);
	else
		clear_bit(bit, &trialcs->flags);

	err = validate_change(cs, trialcs);
	if (err < 0)
		goto out;

	err = heap_init(&heap, PAGE_SIZE, GFP_KERNEL, NULL);
	if (err < 0)
		goto out;

	balance_flag_changed = (is_sched_load_balance(cs) !=
				is_sched_load_balance(trialcs));

	spread_flag_changed = ((is_spread_slab(cs) != is_spread_slab(trialcs))
			|| (is_spread_page(cs) != is_spread_page(trialcs)));

	mutex_lock(&callback_mutex);
	cs->flags = trialcs->flags;
	mutex_unlock(&callback_mutex);

	if (!cpumask_empty(trialcs->cpus_allowed) && balance_flag_changed)
		async_rebuild_sched_domains();

	if (spread_flag_changed)
		update_tasks_flags(cs, &heap);
	heap_free(&heap);
out:
	free_trial_cpuset(trialcs);
	return err;
}


#define FM_COEF 933		/* coefficient for half-life of 10 secs */
#define FM_MAXTICKS ((time_t)99) /* useless computing more ticks than this */
#define FM_MAXCNT 1000000	/* limit cnt to avoid overflow */
#define FM_SCALE 1000		/* faux fixed point scale */

/* Initialize a frequency meter */
static void fmeter_init(struct fmeter *fmp)
{
	fmp->cnt = 0;
	fmp->val = 0;
	fmp->time = 0;
	spin_lock_init(&fmp->lock);
}

/* Internal meter update - process cnt events and update value */
static void fmeter_update(struct fmeter *fmp)
{
	time_t now = get_seconds();
	time_t ticks = now - fmp->time;

	if (ticks == 0)
		return;

	ticks = min(FM_MAXTICKS, ticks);
	while (ticks-- > 0)
		fmp->val = (FM_COEF * fmp->val) / FM_SCALE;
	fmp->time = now;

	fmp->val += ((FM_SCALE - FM_COEF) * fmp->cnt) / FM_SCALE;
	fmp->cnt = 0;
}

/* Process any previous ticks, then bump cnt by one (times scale). */
static void fmeter_markevent(struct fmeter *fmp)
{
	spin_lock(&fmp->lock);
	fmeter_update(fmp);
	fmp->cnt = min(FM_MAXCNT, fmp->cnt + FM_SCALE);
	spin_unlock(&fmp->lock);
}

/* Process any previous ticks, then return current value. */
static int fmeter_getrate(struct fmeter *fmp)
{
	int val;

	spin_lock(&fmp->lock);
	fmeter_update(fmp);
	val = fmp->val;
	spin_unlock(&fmp->lock);
	return val;
}

/* Protected by cgroup_lock */
static cpumask_var_t cpus_attach;

/* Called by cgroups to determine if a cpuset is usable; cgroup_mutex held */
static int cpuset_can_attach(struct cgroup_subsys *ss, struct cgroup *cont,
			     struct task_struct *tsk, bool threadgroup)
{
	int ret;
	struct cpuset *cs = cgroup_cs(cont);

	if ((current != task) && (!capable(CAP_SYS_ADMIN))) {
		const struct cred *cred = current_cred(), *tcred;

		if (cred->euid != tcred->uid && cred->euid != tcred->suid)
			return -EPERM;
	}
 
	if (cpumask_empty(cs->cpus_allowed) || nodes_empty(cs->mems_allowed))
		return -ENOSPC;

	/*
	 * Kthreads bound to specific cpus cannot be moved to a new cpuset; we
	 * cannot change their cpu affinity and isolating such threads by their
	 * set of allowed nodes is unnecessary.  Thus, cpusets are not
	 * applicable for such threads.  This prevents checking for success of
	 * set_cpus_allowed_ptr() on all attached tasks before cpus_allowed may
	 * be changed.
	 */
	if (tsk->flags & PF_THREAD_BOUND)
		return -EINVAL;

	ret = security_task_setscheduler(tsk, 0, NULL);
	if (ret)
		return ret;
	if (threadgroup) {
		struct task_struct *c;

		rcu_read_lock();
		list_for_each_entry_rcu(c, &tsk->thread_group, thread_group) {
			ret = security_task_setscheduler(c, 0, NULL);
			if (ret) {
				rcu_read_unlock();
				return ret;
			}
		}
		rcu_read_unlock();
	}
	return 0;
}

static void cpuset_attach_task(struct task_struct *tsk, nodemask_t *to,
			       struct cpuset *cs)
{
	int err;
	/*
	 * can_attach beforehand should guarantee that this doesn't fail.
	 * TODO: have a better way to handle failure here
	 */
	err = set_cpus_allowed_ptr(tsk, cpus_attach);
	WARN_ON_ONCE(err);

	cpuset_change_task_nodemask(tsk, to);
	cpuset_update_task_spread_flag(cs, tsk);

}

static void cpuset_attach(struct cgroup_subsys *ss, struct cgroup *cont,
			  struct cgroup *oldcont, struct task_struct *tsk,
			  bool threadgroup)
{
	struct mm_struct *mm;
	struct cpuset *cs = cgroup_cs(cont);
	struct cpuset *oldcs = cgroup_cs(oldcont);
	NODEMASK_ALLOC(nodemask_t, from, GFP_KERNEL);
	NODEMASK_ALLOC(nodemask_t, to, GFP_KERNEL);

	if (from == NULL || to == NULL)
		goto alloc_fail;

	if (cs == &top_cpuset) {
		cpumask_copy(cpus_attach, cpu_possible_mask);
	} else {
		guarantee_online_cpus(cs, cpus_attach);
	}
	guarantee_online_mems(cs, to);

	/* do per-task migration stuff possibly for each in the threadgroup */
	cpuset_attach_task(tsk, to, cs);
	if (threadgroup) {
		struct task_struct *c;
		rcu_read_lock();
		list_for_each_entry_rcu(c, &tsk->thread_group, thread_group) {
			cpuset_attach_task(c, to, cs);
		}
		rcu_read_unlock();
	}

	/* change mm; only needs to be done once even if threadgroup */
	*from = oldcs->mems_allowed;
	*to = cs->mems_allowed;
	mm = get_task_mm(tsk);
	if (mm) {
		mpol_rebind_mm(mm, to);
		if (is_memory_migrate(cs))
			cpuset_migrate_mm(mm, from, to);
		mmput(mm);
	}

alloc_fail:
	NODEMASK_FREE(from);
	NODEMASK_FREE(to);
}

/* The various types of files and directories in a cpuset file system */

typedef enum {
	FILE_MEMORY_MIGRATE,
	FILE_CPULIST,
	FILE_MEMLIST,
	FILE_CPU_EXCLUSIVE,
	FILE_MEM_EXCLUSIVE,
	FILE_MEM_HARDWALL,
	FILE_SCHED_LOAD_BALANCE,
	FILE_SCHED_RELAX_DOMAIN_LEVEL,
	FILE_MEMORY_PRESSURE_ENABLED,
	FILE_MEMORY_PRESSURE,
	FILE_SPREAD_PAGE,
	FILE_SPREAD_SLAB,
} cpuset_filetype_t;

static int cpuset_write_u64(struct cgroup *cgrp, struct cftype *cft, u64 val)
{
	int retval = 0;
	struct cpuset *cs = cgroup_cs(cgrp);
	cpuset_filetype_t type = cft->private;

	if (!cgroup_lock_live_group(cgrp))
		return -ENODEV;

	switch (type) {
	case FILE_CPU_EXCLUSIVE:
		retval = update_flag(CS_CPU_EXCLUSIVE, cs, val);
		break;
	case FILE_MEM_EXCLUSIVE:
		retval = update_flag(CS_MEM_EXCLUSIVE, cs, val);
		break;
	case FILE_MEM_HARDWALL:
		retval = update_flag(CS_MEM_HARDWALL, cs, val);
		break;
	case FILE_SCHED_LOAD_BALANCE:
		retval = update_flag(CS_SCHED_LOAD_BALANCE, cs, val);
		break;
	case FILE_MEMORY_MIGRATE:
		retval = update_flag(CS_MEMORY_MIGRATE, cs, val);
		break;
	case FILE_MEMORY_PRESSURE_ENABLED:
		cpuset_memory_pressure_enabled = !!val;
		break;
	case FILE_MEMORY_PRESSURE:
		retval = -EACCES;
		break;
	case FILE_SPREAD_PAGE:
		retval = update_flag(CS_SPREAD_PAGE, cs, val);
		break;
	case FILE_SPREAD_SLAB:
		retval = update_flag(CS_SPREAD_SLAB, cs, val);
		break;
	default:
		retval = -EINVAL;
		break;
	}
	cgroup_unlock();
	return retval;
}

static int cpuset_write_s64(struct cgroup *cgrp, struct cftype *cft, s64 val)
{
	int retval = 0;
	struct cpuset *cs = cgroup_cs(cgrp);
	cpuset_filetype_t type = cft->private;

	if (!cgroup_lock_live_group(cgrp))
		return -ENODEV;

	switch (type) {
	case FILE_SCHED_RELAX_DOMAIN_LEVEL:
		retval = update_relax_domain_level(cs, val);
		break;
	default:
		retval = -EINVAL;
		break;
	}
	cgroup_unlock();
	return retval;
}

static int cpuset_write_resmask(struct cgroup *cgrp, struct cftype *cft,
				const char *buf)
{
	int retval = 0;
	struct cpuset *cs = cgroup_cs(cgrp);
	struct cpuset *trialcs;

	if (!cgroup_lock_live_group(cgrp))
		return -ENODEV;

	trialcs = alloc_trial_cpuset(cs);
	if (!trialcs)
		return -ENOMEM;

	switch (cft->private) {
	case FILE_CPULIST:
		retval = update_cpumask(cs, trialcs, buf);
		break;
	case FILE_MEMLIST:
		retval = update_nodemask(cs, trialcs, buf);
		break;
	default:
		retval = -EINVAL;
		break;
	}

	free_trial_cpuset(trialcs);
	cgroup_unlock();
	return retval;
}


static int cpuset_sprintf_cpulist(char *page, struct cpuset *cs)
{
	int ret;

	mutex_lock(&callback_mutex);
	ret = cpulist_scnprintf(page, PAGE_SIZE, cs->cpus_allowed);
	mutex_unlock(&callback_mutex);

	return ret;
}

static int cpuset_sprintf_memlist(char *page, struct cpuset *cs)
{
	NODEMASK_ALLOC(nodemask_t, mask, GFP_KERNEL);
	int retval;

	if (mask == NULL)
		return -ENOMEM;

	mutex_lock(&callback_mutex);
	*mask = cs->mems_allowed;
	mutex_unlock(&callback_mutex);

	retval = nodelist_scnprintf(page, PAGE_SIZE, *mask);

	NODEMASK_FREE(mask);

	return retval;
}

static ssize_t cpuset_common_file_read(struct cgroup *cont,
				       struct cftype *cft,
				       struct file *file,
				       char __user *buf,
				       size_t nbytes, loff_t *ppos)
{
	struct cpuset *cs = cgroup_cs(cont);
	cpuset_filetype_t type = cft->private;
	char *page;
	ssize_t retval = 0;
	char *s;

	if (!(page = (char *)__get_free_page(GFP_TEMPORARY)))
		return -ENOMEM;

	s = page;

	switch (type) {
	case FILE_CPULIST:
		s += cpuset_sprintf_cpulist(s, cs);
		break;
	case FILE_MEMLIST:
		s += cpuset_sprintf_memlist(s, cs);
		break;
	default:
		retval = -EINVAL;
		goto out;
	}
	*s++ = '\n';

	retval = simple_read_from_buffer(buf, nbytes, ppos, page, s - page);
out:
	free_page((unsigned long)page);
	return retval;
}

static u64 cpuset_read_u64(struct cgroup *cont, struct cftype *cft)
{
	struct cpuset *cs = cgroup_cs(cont);
	cpuset_filetype_t type = cft->private;
	switch (type) {
	case FILE_CPU_EXCLUSIVE:
		return is_cpu_exclusive(cs);
	case FILE_MEM_EXCLUSIVE:
		return is_mem_exclusive(cs);
	case FILE_MEM_HARDWALL:
		return is_mem_hardwall(cs);
	case FILE_SCHED_LOAD_BALANCE:
		return is_sched_load_balance(cs);
	case FILE_MEMORY_MIGRATE:
		return is_memory_migrate(cs);
	case FILE_MEMORY_PRESSURE_ENABLED:
		return cpuset_memory_pressure_enabled;
	case FILE_MEMORY_PRESSURE:
		return fmeter_getrate(&cs->fmeter);
	case FILE_SPREAD_PAGE:
		return is_spread_page(cs);
	case FILE_SPREAD_SLAB:
		return is_spread_slab(cs);
	default:
		BUG();
	}

	/* Unreachable but makes gcc happy */
	return 0;
}

static s64 cpuset_read_s64(struct cgroup *cont, struct cftype *cft)
{
	struct cpuset *cs = cgroup_cs(cont);
	cpuset_filetype_t type = cft->private;
	switch (type) {
	case FILE_SCHED_RELAX_DOMAIN_LEVEL:
		return cs->relax_domain_level;
	default:
		BUG();
	}

	/* Unrechable but makes gcc happy */
	return 0;
}



static struct cftype files[] = {
	{
		.name = "cpus",
		.read = cpuset_common_file_read,
		.write_string = cpuset_write_resmask,
		.max_write_len = (100U + 6 * NR_CPUS),
		.private = FILE_CPULIST,
	},

	{
		.name = "mems",
		.read = cpuset_common_file_read,
		.write_string = cpuset_write_resmask,
		.max_write_len = (100U + 6 * MAX_NUMNODES),
		.private = FILE_MEMLIST,
	},

	{
		.name = "cpu_exclusive",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_CPU_EXCLUSIVE,
	},

	{
		.name = "mem_exclusive",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_MEM_EXCLUSIVE,
	},

	{
		.name = "mem_hardwall",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_MEM_HARDWALL,
	},

	{
		.name = "sched_load_balance",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_SCHED_LOAD_BALANCE,
	},

	{
		.name = "sched_relax_domain_level",
		.read_s64 = cpuset_read_s64,
		.write_s64 = cpuset_write_s64,
		.private = FILE_SCHED_RELAX_DOMAIN_LEVEL,
	},

	{
		.name = "memory_migrate",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_MEMORY_MIGRATE,
	},

	{
		.name = "memory_pressure",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_MEMORY_PRESSURE,
		.mode = S_IRUGO,
	},

	{
		.name = "memory_spread_page",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_SPREAD_PAGE,
	},

	{
		.name = "memory_spread_slab",
		.read_u64 = cpuset_read_u64,
		.write_u64 = cpuset_write_u64,
		.private = FILE_SPREAD_SLAB,
	},
};

static struct cftype cft_memory_pressure_enabled = {
	.name = "memory_pressure_enabled",
	.read_u64 = cpuset_read_u64,
	.write_u64 = cpuset_write_u64,
	.private = FILE_MEMORY_PRESSURE_ENABLED,
};

static int cpuset_populate(struct cgroup_subsys *ss, struct cgroup *cont)
{
	int err;

	err = cgroup_add_files(cont, ss, files, ARRAY_SIZE(files));
	if (err)
		return err;
	/* memory_pressure_enabled is in root cpuset only */
	if (!cont->parent)
		err = cgroup_add_file(cont, ss,
				      &cft_memory_pressure_enabled);
	return err;
}

static void cpuset_post_clone(struct cgroup_subsys *ss,
			      struct cgroup *cgroup)
{
	struct cgroup *parent, *child;
	struct cpuset *cs, *parent_cs;

	parent = cgroup->parent;
	list_for_each_entry(child, &parent->children, sibling) {
		cs = cgroup_cs(child);
		if (is_mem_exclusive(cs) || is_cpu_exclusive(cs))
			return;
	}
	cs = cgroup_cs(cgroup);
	parent_cs = cgroup_cs(parent);

	cs->mems_allowed = parent_cs->mems_allowed;
	cpumask_copy(cs->cpus_allowed, parent_cs->cpus_allowed);
	return;
}


static struct cgroup_subsys_state *cpuset_create(
	struct cgroup_subsys *ss,
	struct cgroup *cont)
{
	struct cpuset *cs;
	struct cpuset *parent;

	if (!cont->parent) {
		return &top_cpuset.css;
	}
	parent = cgroup_cs(cont->parent);
	cs = kmalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return ERR_PTR(-ENOMEM);
	if (!alloc_cpumask_var(&cs->cpus_allowed, GFP_KERNEL)) {
		kfree(cs);
		return ERR_PTR(-ENOMEM);
	}

	cs->flags = 0;
	if (is_spread_page(parent))
		set_bit(CS_SPREAD_PAGE, &cs->flags);
	if (is_spread_slab(parent))
		set_bit(CS_SPREAD_SLAB, &cs->flags);
	set_bit(CS_SCHED_LOAD_BALANCE, &cs->flags);
	cpumask_clear(cs->cpus_allowed);
	nodes_clear(cs->mems_allowed);
	fmeter_init(&cs->fmeter);
	cs->relax_domain_level = -1;

	cs->parent = parent;
	number_of_cpusets++;
	return &cs->css ;
}


static void cpuset_destroy(struct cgroup_subsys *ss, struct cgroup *cont)
{
	struct cpuset *cs = cgroup_cs(cont);

	if (is_sched_load_balance(cs))
		update_flag(CS_SCHED_LOAD_BALANCE, cs, 0);

	number_of_cpusets--;
	free_cpumask_var(cs->cpus_allowed);
	kfree(cs);
}

struct cgroup_subsys cpuset_subsys = {
	.name = "cpuset",
	.create = cpuset_create,
	.destroy = cpuset_destroy,
	.can_attach = cpuset_can_attach,
	.attach = cpuset_attach,
	.populate = cpuset_populate,
	.post_clone = cpuset_post_clone,
	.subsys_id = cpuset_subsys_id,
	.early_init = 1,
};


int __init cpuset_init(void)
{
	int err = 0;

	if (!alloc_cpumask_var(&top_cpuset.cpus_allowed, GFP_KERNEL))
		BUG();

	cpumask_setall(top_cpuset.cpus_allowed);
	nodes_setall(top_cpuset.mems_allowed);

	fmeter_init(&top_cpuset.fmeter);
	set_bit(CS_SCHED_LOAD_BALANCE, &top_cpuset.flags);
	top_cpuset.relax_domain_level = -1;

	err = register_filesystem(&cpuset_fs_type);
	if (err < 0)
		return err;

	if (!alloc_cpumask_var(&cpus_attach, GFP_KERNEL))
		BUG();

	number_of_cpusets = 1;
	return 0;
}

static void cpuset_do_move_task(struct task_struct *tsk,
				struct cgroup_scanner *scan)
{
	struct cgroup *new_cgroup = scan->data;

	cgroup_attach_task(new_cgroup, tsk);
}

static void move_member_tasks_to_cpuset(struct cpuset *from, struct cpuset *to)
{
	struct cgroup_scanner scan;

	scan.cg = from->css.cgroup;
	scan.test_task = NULL; /* select all tasks in cgroup */
	scan.process_task = cpuset_do_move_task;
	scan.heap = NULL;
	scan.data = to->css.cgroup;

	if (cgroup_scan_tasks(&scan))
		printk(KERN_ERR "move_member_tasks_to_cpuset: "
				"cgroup_scan_tasks failed\n");
}

static void remove_tasks_in_empty_cpuset(struct cpuset *cs)
{
	struct cpuset *parent;

	/*
	 * The cgroup's css_sets list is in use if there are tasks
	 * in the cpuset; the list is empty if there are none;
	 * the cs->css.refcnt seems always 0.
	 */
	if (list_empty(&cs->css.cgroup->css_sets))
		return;

	/*
	 * Find its next-highest non-empty parent, (top cpuset
	 * has online cpus, so can't be empty).
	 */
	parent = cs->parent;
	while (cpumask_empty(parent->cpus_allowed) ||
			nodes_empty(parent->mems_allowed))
		parent = parent->parent;

	move_member_tasks_to_cpuset(cs, parent);
}

static void scan_for_empty_cpusets(struct cpuset *root)
{
	LIST_HEAD(queue);
	struct cpuset *cp;	/* scans cpusets being updated */
	struct cpuset *child;	/* scans child cpusets of cp */
	struct cgroup *cont;
	NODEMASK_ALLOC(nodemask_t, oldmems, GFP_KERNEL);

	if (oldmems == NULL)
		return;

	list_add_tail((struct list_head *)&root->stack_list, &queue);

	while (!list_empty(&queue)) {
		cp = list_first_entry(&queue, struct cpuset, stack_list);
		list_del(queue.next);
		list_for_each_entry(cont, &cp->css.cgroup->children, sibling) {
			child = cgroup_cs(cont);
			list_add_tail(&child->stack_list, &queue);
		}

		/* Continue past cpusets with all cpus, mems online */
		if (cpumask_subset(cp->cpus_allowed, cpu_active_mask) &&
		    nodes_subset(cp->mems_allowed, node_states[N_HIGH_MEMORY]))
			continue;

		*oldmems = cp->mems_allowed;

		/* Remove offline cpus and mems from this cpuset. */
		mutex_lock(&callback_mutex);
		cpumask_and(cp->cpus_allowed, cp->cpus_allowed,
			    cpu_active_mask);
		nodes_and(cp->mems_allowed, cp->mems_allowed,
						node_states[N_HIGH_MEMORY]);
		mutex_unlock(&callback_mutex);

		/* Move tasks from the empty cpuset to a parent */
		if (cpumask_empty(cp->cpus_allowed) ||
		     nodes_empty(cp->mems_allowed))
			remove_tasks_in_empty_cpuset(cp);
		else {
			update_tasks_cpumask(cp, NULL);
			update_tasks_nodemask(cp, oldmems, NULL);
		}
	}
	NODEMASK_FREE(oldmems);
}

static int cpuset_track_online_cpus(struct notifier_block *unused_nb,
				unsigned long phase, void *unused_cpu)
{
	struct sched_domain_attr *attr;
	cpumask_var_t *doms;
	int ndoms;

	switch (phase) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
	case CPU_DOWN_FAILED:
	case CPU_DOWN_FAILED_FROZEN:
		break;

	default:
		return NOTIFY_DONE;
	}

	cgroup_lock();
	mutex_lock(&callback_mutex);
	cpumask_copy(top_cpuset.cpus_allowed, cpu_active_mask);
	mutex_unlock(&callback_mutex);
	scan_for_empty_cpusets(&top_cpuset);
	ndoms = generate_sched_domains(&doms, &attr);
	cgroup_unlock();

	/* Have scheduler rebuild the domains */
	partition_sched_domains(ndoms, doms, attr);

	return NOTIFY_OK;
}

#ifdef CONFIG_MEMORY_HOTPLUG
static int cpuset_track_online_nodes(struct notifier_block *self,
				unsigned long action, void *arg)
{
	NODEMASK_ALLOC(nodemask_t, oldmems, GFP_KERNEL);

	if (oldmems == NULL)
		return NOTIFY_DONE;

	cgroup_lock();
	switch (action) {
	case MEM_ONLINE:
		*oldmems = top_cpuset.mems_allowed;
		mutex_lock(&callback_mutex);
		top_cpuset.mems_allowed = node_states[N_HIGH_MEMORY];
		mutex_unlock(&callback_mutex);
		update_tasks_nodemask(&top_cpuset, oldmems, NULL);
		break;
	case MEM_OFFLINE:
		/*
		 * needn't update top_cpuset.mems_allowed explicitly because
		 * scan_for_empty_cpusets() will update it.
		 */
		scan_for_empty_cpusets(&top_cpuset);
		break;
	default:
		break;
	}
	cgroup_unlock();

	NODEMASK_FREE(oldmems);
	return NOTIFY_OK;
}
#endif


void __init cpuset_init_smp(void)
{
	cpumask_copy(top_cpuset.cpus_allowed, cpu_active_mask);
	top_cpuset.mems_allowed = node_states[N_HIGH_MEMORY];

	hotcpu_notifier(cpuset_track_online_cpus, 0);
	hotplug_memory_notifier(cpuset_track_online_nodes, 10);

	cpuset_wq = create_singlethread_workqueue("cpuset");
	BUG_ON(!cpuset_wq);
}


void cpuset_cpus_allowed(struct task_struct *tsk, struct cpumask *pmask)
{
	mutex_lock(&callback_mutex);
	task_lock(tsk);
	guarantee_online_cpus(task_cs(tsk), pmask);
	task_unlock(tsk);
	mutex_unlock(&callback_mutex);
}

int cpuset_cpus_allowed_fallback(struct task_struct *tsk)
{
	const struct cpuset *cs;
	int cpu;

	rcu_read_lock();
	cs = task_cs(tsk);
	if (cs)
		cpumask_copy(&tsk->cpus_allowed, cs->cpus_allowed);
	rcu_read_unlock();

	/*
	 * We own tsk->cpus_allowed, nobody can change it under us.
	 *
	 * But we used cs && cs->cpus_allowed lockless and thus can
	 * race with cgroup_attach_task() or update_cpumask() and get
	 * the wrong tsk->cpus_allowed. However, both cases imply the
	 * subsequent cpuset_change_cpumask()->set_cpus_allowed_ptr()
	 * which takes task_rq_lock().
	 *
	 * If we are called after it dropped the lock we must see all
	 * changes in tsk_cs()->cpus_allowed. Otherwise we can temporary
	 * set any mask even if it is not right from task_cs() pov,
	 * the pending set_cpus_allowed_ptr() will fix things.
	 */

	cpu = cpumask_any_and(&tsk->cpus_allowed, cpu_active_mask);
	if (cpu >= nr_cpu_ids) {
		/*
		 * Either tsk->cpus_allowed is wrong (see above) or it
		 * is actually empty. The latter case is only possible
		 * if we are racing with remove_tasks_in_empty_cpuset().
		 * Like above we can temporary set any mask and rely on
		 * set_cpus_allowed_ptr() as synchronization point.
		 */
		cpumask_copy(&tsk->cpus_allowed, cpu_possible_mask);
		cpu = cpumask_any(cpu_active_mask);
	}

	return cpu;
}

void cpuset_init_current_mems_allowed(void)
{
	nodes_setall(current->mems_allowed);
}


nodemask_t cpuset_mems_allowed(struct task_struct *tsk)
{
	nodemask_t mask;

	mutex_lock(&callback_mutex);
	task_lock(tsk);
	guarantee_online_mems(task_cs(tsk), &mask);
	task_unlock(tsk);
	mutex_unlock(&callback_mutex);

	return mask;
}

int cpuset_nodemask_valid_mems_allowed(nodemask_t *nodemask)
{
	return nodes_intersects(*nodemask, current->mems_allowed);
}

static const struct cpuset *nearest_hardwall_ancestor(const struct cpuset *cs)
{
	while (!(is_mem_exclusive(cs) || is_mem_hardwall(cs)) && cs->parent)
		cs = cs->parent;
	return cs;
}

int __cpuset_node_allowed_softwall(int node, gfp_t gfp_mask)
{
	const struct cpuset *cs;	/* current cpuset ancestors */
	int allowed;			/* is allocation in zone z allowed? */

	if (in_interrupt() || (gfp_mask & __GFP_THISNODE))
		return 1;
	might_sleep_if(!(gfp_mask & __GFP_HARDWALL));
	if (node_isset(node, current->mems_allowed))
		return 1;
	/*
	 * Allow tasks that have access to memory reserves because they have
	 * been OOM killed to get memory anywhere.
	 */
	if (unlikely(test_thread_flag(TIF_MEMDIE)))
		return 1;
	if (gfp_mask & __GFP_HARDWALL)	/* If hardwall request, stop here */
		return 0;

	if (current->flags & PF_EXITING) /* Let dying task have memory */
		return 1;

	/* Not hardwall and node outside mems_allowed: scan up cpusets */
	mutex_lock(&callback_mutex);

	task_lock(current);
	cs = nearest_hardwall_ancestor(task_cs(current));
	task_unlock(current);

	allowed = node_isset(node, cs->mems_allowed);
	mutex_unlock(&callback_mutex);
	return allowed;
}

int __cpuset_node_allowed_hardwall(int node, gfp_t gfp_mask)
{
	if (in_interrupt() || (gfp_mask & __GFP_THISNODE))
		return 1;
	if (node_isset(node, current->mems_allowed))
		return 1;
	/*
	 * Allow tasks that have access to memory reserves because they have
	 * been OOM killed to get memory anywhere.
	 */
	if (unlikely(test_thread_flag(TIF_MEMDIE)))
		return 1;
	return 0;
}


void cpuset_unlock(void)
{
	mutex_unlock(&callback_mutex);
}


static int cpuset_spread_node(int *rotor)
{
	int node;

	node = next_node(*rotor, current->mems_allowed);
	if (node == MAX_NUMNODES)
		node = first_node(current->mems_allowed);
	*rotor = node;
	return node;
}

int cpuset_mem_spread_node(void)
{
	return cpuset_spread_node(&current->cpuset_mem_spread_rotor);
}

int cpuset_slab_spread_node(void)
{
	return cpuset_spread_node(&current->cpuset_slab_spread_rotor);
}

EXPORT_SYMBOL_GPL(cpuset_mem_spread_node);


int cpuset_mems_allowed_intersects(const struct task_struct *tsk1,
				   const struct task_struct *tsk2)
{
	return nodes_intersects(tsk1->mems_allowed, tsk2->mems_allowed);
}

void cpuset_print_task_mems_allowed(struct task_struct *tsk)
{
	struct dentry *dentry;

	dentry = task_cs(tsk)->css.cgroup->dentry;
	spin_lock(&cpuset_buffer_lock);
	snprintf(cpuset_name, CPUSET_NAME_LEN,
		 dentry ? (const char *)dentry->d_name.name : "/");
	nodelist_scnprintf(cpuset_nodelist, CPUSET_NODELIST_LEN,
			   tsk->mems_allowed);
	printk(KERN_INFO "%s cpuset=%s mems_allowed=%s\n",
	       tsk->comm, cpuset_name, cpuset_nodelist);
	spin_unlock(&cpuset_buffer_lock);
}


int cpuset_memory_pressure_enabled __read_mostly;


void __cpuset_memory_pressure_bump(void)
{
	task_lock(current);
	fmeter_markevent(&task_cs(current)->fmeter);
	task_unlock(current);
}

#ifdef CONFIG_PROC_PID_CPUSET
static int proc_cpuset_show(struct seq_file *m, void *unused_v)
{
	struct pid *pid;
	struct task_struct *tsk;
	char *buf;
	struct cgroup_subsys_state *css;
	int retval;

	retval = -ENOMEM;
	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		goto out;

	retval = -ESRCH;
	pid = m->private;
	tsk = get_pid_task(pid, PIDTYPE_PID);
	if (!tsk)
		goto out_free;

	retval = -EINVAL;
	cgroup_lock();
	css = task_subsys_state(tsk, cpuset_subsys_id);
	retval = cgroup_path(css->cgroup, buf, PAGE_SIZE);
	if (retval < 0)
		goto out_unlock;
	seq_puts(m, buf);
	seq_putc(m, '\n');
out_unlock:
	cgroup_unlock();
	put_task_struct(tsk);
out_free:
	kfree(buf);
out:
	return retval;
}

static int cpuset_open(struct inode *inode, struct file *file)
{
	struct pid *pid = PROC_I(inode)->pid;
	return single_open(file, proc_cpuset_show, pid);
}

const struct file_operations proc_cpuset_operations = {
	.open		= cpuset_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif /* CONFIG_PROC_PID_CPUSET */

/* Display task mems_allowed in /proc/<pid>/status file. */
void cpuset_task_status_allowed(struct seq_file *m, struct task_struct *task)
{
	seq_printf(m, "Mems_allowed:\t");
	seq_nodemask(m, &task->mems_allowed);
	seq_printf(m, "\n");
	seq_printf(m, "Mems_allowed_list:\t");
	seq_nodemask_list(m, &task->mems_allowed);
	seq_printf(m, "\n");
}
