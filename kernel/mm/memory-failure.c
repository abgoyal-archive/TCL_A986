

#define DEBUG 1		/* remove me in 2.6.34 */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/kernel-page-flags.h>
#include <linux/sched.h>
#include <linux/ksm.h>
#include <linux/rmap.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/backing-dev.h>
#include <linux/migrate.h>
#include <linux/page-isolation.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include "internal.h"

int sysctl_memory_failure_early_kill __read_mostly = 0;

int sysctl_memory_failure_recovery __read_mostly = 1;

atomic_long_t mce_bad_pages __read_mostly = ATOMIC_LONG_INIT(0);

#if defined(CONFIG_HWPOISON_INJECT) || defined(CONFIG_HWPOISON_INJECT_MODULE)

u32 hwpoison_filter_enable = 0;
u32 hwpoison_filter_dev_major = ~0U;
u32 hwpoison_filter_dev_minor = ~0U;
u64 hwpoison_filter_flags_mask;
u64 hwpoison_filter_flags_value;
EXPORT_SYMBOL_GPL(hwpoison_filter_enable);
EXPORT_SYMBOL_GPL(hwpoison_filter_dev_major);
EXPORT_SYMBOL_GPL(hwpoison_filter_dev_minor);
EXPORT_SYMBOL_GPL(hwpoison_filter_flags_mask);
EXPORT_SYMBOL_GPL(hwpoison_filter_flags_value);

static int hwpoison_filter_dev(struct page *p)
{
	struct address_space *mapping;
	dev_t dev;

	if (hwpoison_filter_dev_major == ~0U &&
	    hwpoison_filter_dev_minor == ~0U)
		return 0;

	/*
	 * page_mapping() does not accept slab page
	 */
	if (PageSlab(p))
		return -EINVAL;

	mapping = page_mapping(p);
	if (mapping == NULL || mapping->host == NULL)
		return -EINVAL;

	dev = mapping->host->i_sb->s_dev;
	if (hwpoison_filter_dev_major != ~0U &&
	    hwpoison_filter_dev_major != MAJOR(dev))
		return -EINVAL;
	if (hwpoison_filter_dev_minor != ~0U &&
	    hwpoison_filter_dev_minor != MINOR(dev))
		return -EINVAL;

	return 0;
}

static int hwpoison_filter_flags(struct page *p)
{
	if (!hwpoison_filter_flags_mask)
		return 0;

	if ((stable_page_flags(p) & hwpoison_filter_flags_mask) ==
				    hwpoison_filter_flags_value)
		return 0;
	else
		return -EINVAL;
}

#ifdef	CONFIG_CGROUP_MEM_RES_CTLR_SWAP
u64 hwpoison_filter_memcg;
EXPORT_SYMBOL_GPL(hwpoison_filter_memcg);
static int hwpoison_filter_task(struct page *p)
{
	struct mem_cgroup *mem;
	struct cgroup_subsys_state *css;
	unsigned long ino;

	if (!hwpoison_filter_memcg)
		return 0;

	mem = try_get_mem_cgroup_from_page(p);
	if (!mem)
		return -EINVAL;

	css = mem_cgroup_css(mem);
	/* root_mem_cgroup has NULL dentries */
	if (!css->cgroup->dentry)
		return -EINVAL;

	ino = css->cgroup->dentry->d_inode->i_ino;
	css_put(css);

	if (ino != hwpoison_filter_memcg)
		return -EINVAL;

	return 0;
}
#else
static int hwpoison_filter_task(struct page *p) { return 0; }
#endif

int hwpoison_filter(struct page *p)
{
	if (!hwpoison_filter_enable)
		return 0;

	if (hwpoison_filter_dev(p))
		return -EINVAL;

	if (hwpoison_filter_flags(p))
		return -EINVAL;

	if (hwpoison_filter_task(p))
		return -EINVAL;

	return 0;
}
#else
int hwpoison_filter(struct page *p)
{
	return 0;
}
#endif

EXPORT_SYMBOL_GPL(hwpoison_filter);

static int kill_proc_ao(struct task_struct *t, unsigned long addr, int trapno,
			unsigned long pfn)
{
	struct siginfo si;
	int ret;

	printk(KERN_ERR
		"MCE %#lx: Killing %s:%d early due to hardware memory corruption\n",
		pfn, t->comm, t->pid);
	si.si_signo = SIGBUS;
	si.si_errno = 0;
	si.si_code = BUS_MCEERR_AO;
	si.si_addr = (void *)addr;
#ifdef __ARCH_SI_TRAPNO
	si.si_trapno = trapno;
#endif
	si.si_addr_lsb = PAGE_SHIFT;
	/*
	 * Don't use force here, it's convenient if the signal
	 * can be temporarily blocked.
	 * This could cause a loop when the user sets SIGBUS
	 * to SIG_IGN, but hopefully noone will do that?
	 */
	ret = send_sig_info(SIGBUS, &si, t);  /* synchronous? */
	if (ret < 0)
		printk(KERN_INFO "MCE: Error sending signal to %s:%d: %d\n",
		       t->comm, t->pid, ret);
	return ret;
}

void shake_page(struct page *p, int access)
{
	if (!PageSlab(p)) {
		lru_add_drain_all();
		if (PageLRU(p))
			return;
		drain_all_pages();
		if (PageLRU(p) || is_free_buddy_page(p))
			return;
	}

	/*
	 * Only all shrink_slab here (which would also
	 * shrink other caches) if access is not potentially fatal.
	 */
	if (access) {
		int nr;
		do {
			nr = shrink_slab(1000, GFP_KERNEL, 1000);
			if (page_count(p) == 0)
				break;
		} while (nr > 10);
	}
}
EXPORT_SYMBOL_GPL(shake_page);


struct to_kill {
	struct list_head nd;
	struct task_struct *tsk;
	unsigned long addr;
	unsigned addr_valid:1;
};


static void add_to_kill(struct task_struct *tsk, struct page *p,
		       struct vm_area_struct *vma,
		       struct list_head *to_kill,
		       struct to_kill **tkc)
{
	struct to_kill *tk;

	if (*tkc) {
		tk = *tkc;
		*tkc = NULL;
	} else {
		tk = kmalloc(sizeof(struct to_kill), GFP_ATOMIC);
		if (!tk) {
			printk(KERN_ERR
		"MCE: Out of memory while machine check handling\n");
			return;
		}
	}
	tk->addr = page_address_in_vma(p, vma);
	tk->addr_valid = 1;

	/*
	 * In theory we don't have to kill when the page was
	 * munmaped. But it could be also a mremap. Since that's
	 * likely very rare kill anyways just out of paranoia, but use
	 * a SIGKILL because the error is not contained anymore.
	 */
	if (tk->addr == -EFAULT) {
		pr_debug("MCE: Unable to find user space address %lx in %s\n",
			page_to_pfn(p), tsk->comm);
		tk->addr_valid = 0;
	}
	get_task_struct(tsk);
	tk->tsk = tsk;
	list_add_tail(&tk->nd, to_kill);
}

static void kill_procs_ao(struct list_head *to_kill, int doit, int trapno,
			  int fail, unsigned long pfn)
{
	struct to_kill *tk, *next;

	list_for_each_entry_safe (tk, next, to_kill, nd) {
		if (doit) {
			/*
			 * In case something went wrong with munmapping
			 * make sure the process doesn't catch the
			 * signal and then access the memory. Just kill it.
			 */
			if (fail || tk->addr_valid == 0) {
				printk(KERN_ERR
		"MCE %#lx: forcibly killing %s:%d because of failure to unmap corrupted page\n",
					pfn, tk->tsk->comm, tk->tsk->pid);
				force_sig(SIGKILL, tk->tsk);
			}

			/*
			 * In theory the process could have mapped
			 * something else on the address in-between. We could
			 * check for that, but we need to tell the
			 * process anyways.
			 */
			else if (kill_proc_ao(tk->tsk, tk->addr, trapno,
					      pfn) < 0)
				printk(KERN_ERR
		"MCE %#lx: Cannot send advisory machine check signal to %s:%d\n",
					pfn, tk->tsk->comm, tk->tsk->pid);
		}
		put_task_struct(tk->tsk);
		kfree(tk);
	}
}

static int task_early_kill(struct task_struct *tsk)
{
	if (!tsk->mm)
		return 0;
	if (tsk->flags & PF_MCE_PROCESS)
		return !!(tsk->flags & PF_MCE_EARLY);
	return sysctl_memory_failure_early_kill;
}

static void collect_procs_anon(struct page *page, struct list_head *to_kill,
			      struct to_kill **tkc)
{
	struct vm_area_struct *vma;
	struct task_struct *tsk;
	struct anon_vma *av;

	read_lock(&tasklist_lock);
	av = page_lock_anon_vma(page);
	if (av == NULL)	/* Not actually mapped anymore */
		goto out;
	for_each_process (tsk) {
		struct anon_vma_chain *vmac;

		if (!task_early_kill(tsk))
			continue;
		list_for_each_entry(vmac, &av->head, same_anon_vma) {
			vma = vmac->vma;
			if (!page_mapped_in_vma(page, vma))
				continue;
			if (vma->vm_mm == tsk->mm)
				add_to_kill(tsk, page, vma, to_kill, tkc);
		}
	}
	page_unlock_anon_vma(av);
out:
	read_unlock(&tasklist_lock);
}

static void collect_procs_file(struct page *page, struct list_head *to_kill,
			      struct to_kill **tkc)
{
	struct vm_area_struct *vma;
	struct task_struct *tsk;
	struct prio_tree_iter iter;
	struct address_space *mapping = page->mapping;

	/*
	 * A note on the locking order between the two locks.
	 * We don't rely on this particular order.
	 * If you have some other code that needs a different order
	 * feel free to switch them around. Or add a reverse link
	 * from mm_struct to task_struct, then this could be all
	 * done without taking tasklist_lock and looping over all tasks.
	 */

	read_lock(&tasklist_lock);
	spin_lock(&mapping->i_mmap_lock);
	for_each_process(tsk) {
		pgoff_t pgoff = page->index << (PAGE_CACHE_SHIFT - PAGE_SHIFT);

		if (!task_early_kill(tsk))
			continue;

		vma_prio_tree_foreach(vma, &iter, &mapping->i_mmap, pgoff,
				      pgoff) {
			/*
			 * Send early kill signal to tasks where a vma covers
			 * the page but the corrupted page is not necessarily
			 * mapped it in its pte.
			 * Assume applications who requested early kill want
			 * to be informed of all such data corruptions.
			 */
			if (vma->vm_mm == tsk->mm)
				add_to_kill(tsk, page, vma, to_kill, tkc);
		}
	}
	spin_unlock(&mapping->i_mmap_lock);
	read_unlock(&tasklist_lock);
}

static void collect_procs(struct page *page, struct list_head *tokill)
{
	struct to_kill *tk;

	if (!page->mapping)
		return;

	tk = kmalloc(sizeof(struct to_kill), GFP_NOIO);
	if (!tk)
		return;
	if (PageAnon(page))
		collect_procs_anon(page, tokill, &tk);
	else
		collect_procs_file(page, tokill, &tk);
	kfree(tk);
}


enum outcome {
	IGNORED,	/* Error: cannot be handled */
	FAILED,		/* Error: handling failed */
	DELAYED,	/* Will be handled later */
	RECOVERED,	/* Successfully recovered */
};

static const char *action_name[] = {
	[IGNORED] = "Ignored",
	[FAILED] = "Failed",
	[DELAYED] = "Delayed",
	[RECOVERED] = "Recovered",
};

static int delete_from_lru_cache(struct page *p)
{
	if (!isolate_lru_page(p)) {
		/*
		 * Clear sensible page flags, so that the buddy system won't
		 * complain when the page is unpoison-and-freed.
		 */
		ClearPageActive(p);
		ClearPageUnevictable(p);
		/*
		 * drop the page count elevated by isolate_lru_page()
		 */
		page_cache_release(p);
		return 0;
	}
	return -EIO;
}

static int me_kernel(struct page *p, unsigned long pfn)
{
	return IGNORED;
}

static int me_unknown(struct page *p, unsigned long pfn)
{
	printk(KERN_ERR "MCE %#lx: Unknown page state\n", pfn);
	return FAILED;
}

static int me_pagecache_clean(struct page *p, unsigned long pfn)
{
	int err;
	int ret = FAILED;
	struct address_space *mapping;

	delete_from_lru_cache(p);

	/*
	 * For anonymous pages we're done the only reference left
	 * should be the one m_f() holds.
	 */
	if (PageAnon(p))
		return RECOVERED;

	/*
	 * Now truncate the page in the page cache. This is really
	 * more like a "temporary hole punch"
	 * Don't do this for block devices when someone else
	 * has a reference, because it could be file system metadata
	 * and that's not safe to truncate.
	 */
	mapping = page_mapping(p);
	if (!mapping) {
		/*
		 * Page has been teared down in the meanwhile
		 */
		return FAILED;
	}

	/*
	 * Truncation is a bit tricky. Enable it per file system for now.
	 *
	 * Open: to take i_mutex or not for this? Right now we don't.
	 */
	if (mapping->a_ops->error_remove_page) {
		err = mapping->a_ops->error_remove_page(mapping, p);
		if (err != 0) {
			printk(KERN_INFO "MCE %#lx: Failed to punch page: %d\n",
					pfn, err);
		} else if (page_has_private(p) &&
				!try_to_release_page(p, GFP_NOIO)) {
			pr_debug("MCE %#lx: failed to release buffers\n", pfn);
		} else {
			ret = RECOVERED;
		}
	} else {
		/*
		 * If the file system doesn't support it just invalidate
		 * This fails on dirty or anything with private pages
		 */
		if (invalidate_inode_page(p))
			ret = RECOVERED;
		else
			printk(KERN_INFO "MCE %#lx: Failed to invalidate\n",
				pfn);
	}
	return ret;
}

static int me_pagecache_dirty(struct page *p, unsigned long pfn)
{
	struct address_space *mapping = page_mapping(p);

	SetPageError(p);
	/* TBD: print more information about the file. */
	if (mapping) {
		/*
		 * IO error will be reported by write(), fsync(), etc.
		 * who check the mapping.
		 * This way the application knows that something went
		 * wrong with its dirty file data.
		 *
		 * There's one open issue:
		 *
		 * The EIO will be only reported on the next IO
		 * operation and then cleared through the IO map.
		 * Normally Linux has two mechanisms to pass IO error
		 * first through the AS_EIO flag in the address space
		 * and then through the PageError flag in the page.
		 * Since we drop pages on memory failure handling the
		 * only mechanism open to use is through AS_AIO.
		 *
		 * This has the disadvantage that it gets cleared on
		 * the first operation that returns an error, while
		 * the PageError bit is more sticky and only cleared
		 * when the page is reread or dropped.  If an
		 * application assumes it will always get error on
		 * fsync, but does other operations on the fd before
		 * and the page is dropped inbetween then the error
		 * will not be properly reported.
		 *
		 * This can already happen even without hwpoisoned
		 * pages: first on metadata IO errors (which only
		 * report through AS_EIO) or when the page is dropped
		 * at the wrong time.
		 *
		 * So right now we assume that the application DTRT on
		 * the first EIO, but we're not worse than other parts
		 * of the kernel.
		 */
		mapping_set_error(mapping, EIO);
	}

	return me_pagecache_clean(p, pfn);
}

static int me_swapcache_dirty(struct page *p, unsigned long pfn)
{
	ClearPageDirty(p);
	/* Trigger EIO in shmem: */
	ClearPageUptodate(p);

	if (!delete_from_lru_cache(p))
		return DELAYED;
	else
		return FAILED;
}

static int me_swapcache_clean(struct page *p, unsigned long pfn)
{
	delete_from_swap_cache(p);

	if (!delete_from_lru_cache(p))
		return RECOVERED;
	else
		return FAILED;
}

static int me_huge_page(struct page *p, unsigned long pfn)
{
	return FAILED;
}


#define dirty		(1UL << PG_dirty)
#define sc		(1UL << PG_swapcache)
#define unevict		(1UL << PG_unevictable)
#define mlock		(1UL << PG_mlocked)
#define writeback	(1UL << PG_writeback)
#define lru		(1UL << PG_lru)
#define swapbacked	(1UL << PG_swapbacked)
#define head		(1UL << PG_head)
#define tail		(1UL << PG_tail)
#define compound	(1UL << PG_compound)
#define slab		(1UL << PG_slab)
#define reserved	(1UL << PG_reserved)

static struct page_state {
	unsigned long mask;
	unsigned long res;
	char *msg;
	int (*action)(struct page *p, unsigned long pfn);
} error_states[] = {
	{ reserved,	reserved,	"reserved kernel",	me_kernel },
	/*
	 * free pages are specially detected outside this table:
	 * PG_buddy pages only make a small fraction of all free pages.
	 */

	/*
	 * Could in theory check if slab page is free or if we can drop
	 * currently unused objects without touching them. But just
	 * treat it as standard kernel for now.
	 */
	{ slab,		slab,		"kernel slab",	me_kernel },

#ifdef CONFIG_PAGEFLAGS_EXTENDED
	{ head,		head,		"huge",		me_huge_page },
	{ tail,		tail,		"huge",		me_huge_page },
#else
	{ compound,	compound,	"huge",		me_huge_page },
#endif

	{ sc|dirty,	sc|dirty,	"swapcache",	me_swapcache_dirty },
	{ sc|dirty,	sc,		"swapcache",	me_swapcache_clean },

	{ unevict|dirty, unevict|dirty,	"unevictable LRU", me_pagecache_dirty},
	{ unevict,	unevict,	"unevictable LRU", me_pagecache_clean},

	{ mlock|dirty,	mlock|dirty,	"mlocked LRU",	me_pagecache_dirty },
	{ mlock,	mlock,		"mlocked LRU",	me_pagecache_clean },

	{ lru|dirty,	lru|dirty,	"LRU",		me_pagecache_dirty },
	{ lru|dirty,	lru,		"clean LRU",	me_pagecache_clean },

	/*
	 * Catchall entry: must be at end.
	 */
	{ 0,		0,		"unknown page state",	me_unknown },
};

#undef dirty
#undef sc
#undef unevict
#undef mlock
#undef writeback
#undef lru
#undef swapbacked
#undef head
#undef tail
#undef compound
#undef slab
#undef reserved

static void action_result(unsigned long pfn, char *msg, int result)
{
	struct page *page = pfn_to_page(pfn);

	printk(KERN_ERR "MCE %#lx: %s%s page recovery: %s\n",
		pfn,
		PageDirty(page) ? "dirty " : "",
		msg, action_name[result]);
}

static int page_action(struct page_state *ps, struct page *p,
			unsigned long pfn)
{
	int result;
	int count;

	result = ps->action(p, pfn);
	action_result(pfn, ps->msg, result);

	count = page_count(p) - 1;
	if (ps->action == me_swapcache_dirty && result == DELAYED)
		count--;
	if (count != 0) {
		printk(KERN_ERR
		       "MCE %#lx: %s page still referenced by %d users\n",
		       pfn, ps->msg, count);
		result = FAILED;
	}

	/* Could do more checks here if page looks ok */
	/*
	 * Could adjust zone counters here to correct for the missing page.
	 */

	return (result == RECOVERED || result == DELAYED) ? 0 : -EBUSY;
}

#define N_UNMAP_TRIES 5

static int hwpoison_user_mappings(struct page *p, unsigned long pfn,
				  int trapno)
{
	enum ttu_flags ttu = TTU_UNMAP | TTU_IGNORE_MLOCK | TTU_IGNORE_ACCESS;
	struct address_space *mapping;
	LIST_HEAD(tokill);
	int ret;
	int i;
	int kill = 1;

	if (PageReserved(p) || PageSlab(p))
		return SWAP_SUCCESS;

	/*
	 * This check implies we don't kill processes if their pages
	 * are in the swap cache early. Those are always late kills.
	 */
	if (!page_mapped(p))
		return SWAP_SUCCESS;

	if (PageCompound(p) || PageKsm(p))
		return SWAP_FAIL;

	if (PageSwapCache(p)) {
		printk(KERN_ERR
		       "MCE %#lx: keeping poisoned page in swap cache\n", pfn);
		ttu |= TTU_IGNORE_HWPOISON;
	}

	/*
	 * Propagate the dirty bit from PTEs to struct page first, because we
	 * need this to decide if we should kill or just drop the page.
	 * XXX: the dirty test could be racy: set_page_dirty() may not always
	 * be called inside page lock (it's recommended but not enforced).
	 */
	mapping = page_mapping(p);
	if (!PageDirty(p) && mapping && mapping_cap_writeback_dirty(mapping)) {
		if (page_mkclean(p)) {
			SetPageDirty(p);
		} else {
			kill = 0;
			ttu |= TTU_IGNORE_HWPOISON;
			printk(KERN_INFO
	"MCE %#lx: corrupted page was clean: dropped without side effects\n",
				pfn);
		}
	}

	/*
	 * First collect all the processes that have the page
	 * mapped in dirty form.  This has to be done before try_to_unmap,
	 * because ttu takes the rmap data structures down.
	 *
	 * Error handling: We ignore errors here because
	 * there's nothing that can be done.
	 */
	if (kill)
		collect_procs(p, &tokill);

	/*
	 * try_to_unmap can fail temporarily due to races.
	 * Try a few times (RED-PEN better strategy?)
	 */
	for (i = 0; i < N_UNMAP_TRIES; i++) {
		ret = try_to_unmap(p, ttu);
		if (ret == SWAP_SUCCESS)
			break;
		pr_debug("MCE %#lx: try_to_unmap retry needed %d\n", pfn,  ret);
	}

	if (ret != SWAP_SUCCESS)
		printk(KERN_ERR "MCE %#lx: failed to unmap page (mapcount=%d)\n",
				pfn, page_mapcount(p));

	/*
	 * Now that the dirty bit has been propagated to the
	 * struct page and all unmaps done we can decide if
	 * killing is needed or not.  Only kill when the page
	 * was dirty, otherwise the tokill list is merely
	 * freed.  When there was a problem unmapping earlier
	 * use a more force-full uncatchable kill to prevent
	 * any accesses to the poisoned memory.
	 */
	kill_procs_ao(&tokill, !!PageDirty(p), trapno,
		      ret != SWAP_SUCCESS, pfn);

	return ret;
}

int __memory_failure(unsigned long pfn, int trapno, int flags)
{
	struct page_state *ps;
	struct page *p;
	int res;

	if (!sysctl_memory_failure_recovery)
		panic("Memory failure from trap %d on page %lx", trapno, pfn);

	if (!pfn_valid(pfn)) {
		printk(KERN_ERR
		       "MCE %#lx: memory outside kernel control\n",
		       pfn);
		return -ENXIO;
	}

	p = pfn_to_page(pfn);
	if (TestSetPageHWPoison(p)) {
		printk(KERN_ERR "MCE %#lx: already hardware poisoned\n", pfn);
		return 0;
	}

	atomic_long_add(1, &mce_bad_pages);

	/*
	 * We need/can do nothing about count=0 pages.
	 * 1) it's a free page, and therefore in safe hand:
	 *    prep_new_page() will be the gate keeper.
	 * 2) it's part of a non-compound high order page.
	 *    Implies some kernel user: cannot stop them from
	 *    R/W the page; let's pray that the page has been
	 *    used and will be freed some time later.
	 * In fact it's dangerous to directly bump up page count from 0,
	 * that may make page_freeze_refs()/page_unfreeze_refs() mismatch.
	 */
	if (!(flags & MF_COUNT_INCREASED) &&
		!get_page_unless_zero(compound_head(p))) {
		if (is_free_buddy_page(p)) {
			action_result(pfn, "free buddy", DELAYED);
			return 0;
		} else {
			action_result(pfn, "high order kernel", IGNORED);
			return -EBUSY;
		}
	}

	/*
	 * We ignore non-LRU pages for good reasons.
	 * - PG_locked is only well defined for LRU pages and a few others
	 * - to avoid races with __set_page_locked()
	 * - to avoid races with __SetPageSlab*() (and more non-atomic ops)
	 * The check (unnecessarily) ignores LRU pages being isolated and
	 * walked by the page reclaim code, however that's not a big loss.
	 */
	if (!PageLRU(p))
		shake_page(p, 0);
	if (!PageLRU(p)) {
		/*
		 * shake_page could have turned it free.
		 */
		if (is_free_buddy_page(p)) {
			action_result(pfn, "free buddy, 2nd try", DELAYED);
			return 0;
		}
		action_result(pfn, "non LRU", IGNORED);
		put_page(p);
		return -EBUSY;
	}

	/*
	 * Lock the page and wait for writeback to finish.
	 * It's very difficult to mess with pages currently under IO
	 * and in many cases impossible, so we just avoid it here.
	 */
	lock_page_nosync(p);

	/*
	 * unpoison always clear PG_hwpoison inside page lock
	 */
	if (!PageHWPoison(p)) {
		printk(KERN_ERR "MCE %#lx: just unpoisoned\n", pfn);
		res = 0;
		goto out;
	}
	if (hwpoison_filter(p)) {
		if (TestClearPageHWPoison(p))
			atomic_long_dec(&mce_bad_pages);
		unlock_page(p);
		put_page(p);
		return 0;
	}

	wait_on_page_writeback(p);

	/*
	 * Now take care of user space mappings.
	 * Abort on fail: __remove_from_page_cache() assumes unmapped page.
	 */
	if (hwpoison_user_mappings(p, pfn, trapno) != SWAP_SUCCESS) {
		printk(KERN_ERR "MCE %#lx: cannot unmap page, give up\n", pfn);
		res = -EBUSY;
		goto out;
	}

	/*
	 * Torn down by someone else?
	 */
	if (PageLRU(p) && !PageSwapCache(p) && p->mapping == NULL) {
		action_result(pfn, "already truncated LRU", IGNORED);
		res = -EBUSY;
		goto out;
	}

	res = -EBUSY;
	for (ps = error_states;; ps++) {
		if ((p->flags & ps->mask) == ps->res) {
			res = page_action(ps, p, pfn);
			break;
		}
	}
out:
	unlock_page(p);
	return res;
}
EXPORT_SYMBOL_GPL(__memory_failure);

void memory_failure(unsigned long pfn, int trapno)
{
	__memory_failure(pfn, trapno, 0);
}

int unpoison_memory(unsigned long pfn)
{
	struct page *page;
	struct page *p;
	int freeit = 0;

	if (!pfn_valid(pfn))
		return -ENXIO;

	p = pfn_to_page(pfn);
	page = compound_head(p);

	if (!PageHWPoison(p)) {
		pr_debug("MCE: Page was already unpoisoned %#lx\n", pfn);
		return 0;
	}

	if (!get_page_unless_zero(page)) {
		if (TestClearPageHWPoison(p))
			atomic_long_dec(&mce_bad_pages);
		pr_debug("MCE: Software-unpoisoned free page %#lx\n", pfn);
		return 0;
	}

	lock_page_nosync(page);
	/*
	 * This test is racy because PG_hwpoison is set outside of page lock.
	 * That's acceptable because that won't trigger kernel panic. Instead,
	 * the PG_hwpoison page will be caught and isolated on the entrance to
	 * the free buddy page pool.
	 */
	if (TestClearPageHWPoison(p)) {
		pr_debug("MCE: Software-unpoisoned page %#lx\n", pfn);
		atomic_long_dec(&mce_bad_pages);
		freeit = 1;
	}
	unlock_page(page);

	put_page(page);
	if (freeit)
		put_page(page);

	return 0;
}
EXPORT_SYMBOL(unpoison_memory);

static struct page *new_page(struct page *p, unsigned long private, int **x)
{
	int nid = page_to_nid(p);
	return alloc_pages_exact_node(nid, GFP_HIGHUSER_MOVABLE, 0);
}

static int get_any_page(struct page *p, unsigned long pfn, int flags)
{
	int ret;

	if (flags & MF_COUNT_INCREASED)
		return 1;

	/*
	 * The lock_system_sleep prevents a race with memory hotplug,
	 * because the isolation assumes there's only a single user.
	 * This is a big hammer, a better would be nicer.
	 */
	lock_system_sleep();

	/*
	 * Isolate the page, so that it doesn't get reallocated if it
	 * was free.
	 */
	set_migratetype_isolate(p);
	if (!get_page_unless_zero(compound_head(p))) {
		if (is_free_buddy_page(p)) {
			pr_debug("get_any_page: %#lx free buddy page\n", pfn);
			/* Set hwpoison bit while page is still isolated */
			SetPageHWPoison(p);
			ret = 0;
		} else {
			pr_debug("get_any_page: %#lx: unknown zero refcount page type %lx\n",
				pfn, p->flags);
			ret = -EIO;
		}
	} else {
		/* Not a free page */
		ret = 1;
	}
	unset_migratetype_isolate(p);
	unlock_system_sleep();
	return ret;
}

int soft_offline_page(struct page *page, int flags)
{
	int ret;
	unsigned long pfn = page_to_pfn(page);

	ret = get_any_page(page, pfn, flags);
	if (ret < 0)
		return ret;
	if (ret == 0)
		goto done;

	/*
	 * Page cache page we can handle?
	 */
	if (!PageLRU(page)) {
		/*
		 * Try to free it.
		 */
		put_page(page);
		shake_page(page, 1);

		/*
		 * Did it turn free?
		 */
		ret = get_any_page(page, pfn, 0);
		if (ret < 0)
			return ret;
		if (ret == 0)
			goto done;
	}
	if (!PageLRU(page)) {
		pr_debug("soft_offline: %#lx: unknown non LRU page type %lx\n",
				pfn, page->flags);
		return -EIO;
	}

	lock_page(page);
	wait_on_page_writeback(page);

	/*
	 * Synchronized using the page lock with memory_failure()
	 */
	if (PageHWPoison(page)) {
		unlock_page(page);
		put_page(page);
		pr_debug("soft offline: %#lx page already poisoned\n", pfn);
		return -EBUSY;
	}

	/*
	 * Try to invalidate first. This should work for
	 * non dirty unmapped page cache pages.
	 */
	ret = invalidate_inode_page(page);
	unlock_page(page);

	/*
	 * Drop count because page migration doesn't like raised
	 * counts. The page could get re-allocated, but if it becomes
	 * LRU the isolation will just fail.
	 * RED-PEN would be better to keep it isolated here, but we
	 * would need to fix isolation locking first.
	 */
	put_page(page);
	if (ret == 1) {
		ret = 0;
		pr_debug("soft_offline: %#lx: invalidated\n", pfn);
		goto done;
	}

	/*
	 * Simple invalidation didn't work.
	 * Try to migrate to a new page instead. migrate.c
	 * handles a large number of cases for us.
	 */
	ret = isolate_lru_page(page);
	if (!ret) {
		LIST_HEAD(pagelist);

		list_add(&page->lru, &pagelist);
		ret = migrate_pages(&pagelist, new_page, MPOL_MF_MOVE_ALL, 0);
		if (ret) {
			pr_debug("soft offline: %#lx: migration failed %d, type %lx\n",
				pfn, ret, page->flags);
			if (ret > 0)
				ret = -EIO;
		}
	} else {
		pr_debug("soft offline: %#lx: isolation failed: %d, page count %d, type %lx\n",
				pfn, ret, page_count(page), page->flags);
	}
	if (ret)
		return ret;

done:
	atomic_long_add(1, &mce_bad_pages);
	SetPageHWPoison(page);
	/* keep elevated page count for bad page */
	return ret;
}
