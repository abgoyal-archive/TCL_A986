
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/jhash.h>
#include <linux/init.h>
#include <linux/futex.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/syscalls.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/magic.h>
#include <linux/pid.h>
#include <linux/nsproxy.h>

#include <asm/futex.h>

#include "rtmutex_common.h"

int __read_mostly futex_cmpxchg_enabled;

#define FUTEX_HASHBITS (CONFIG_BASE_SMALL ? 4 : 8)

struct futex_pi_state {
	/*
	 * list of 'owned' pi_state instances - these have to be
	 * cleaned up in do_exit() if the task exits prematurely:
	 */
	struct list_head list;

	/*
	 * The PI object:
	 */
	struct rt_mutex pi_mutex;

	struct task_struct *owner;
	atomic_t refcount;

	union futex_key key;
};

struct futex_q {
	struct plist_node list;

	struct task_struct *task;
	spinlock_t *lock_ptr;
	union futex_key key;
	struct futex_pi_state *pi_state;
	struct rt_mutex_waiter *rt_waiter;
	union futex_key *requeue_pi_key;
	u32 bitset;
};

struct futex_hash_bucket {
	spinlock_t lock;
	struct plist_head chain;
};

static struct futex_hash_bucket futex_queues[1<<FUTEX_HASHBITS];

static struct futex_hash_bucket *hash_futex(union futex_key *key)
{
	u32 hash = jhash2((u32*)&key->both.word,
			  (sizeof(key->both.word)+sizeof(key->both.ptr))/4,
			  key->both.offset);
	return &futex_queues[hash & ((1 << FUTEX_HASHBITS)-1)];
}

static inline int match_futex(union futex_key *key1, union futex_key *key2)
{
	return (key1 && key2
		&& key1->both.word == key2->both.word
		&& key1->both.ptr == key2->both.ptr
		&& key1->both.offset == key2->both.offset);
}

static void get_futex_key_refs(union futex_key *key)
{
	if (!key->both.ptr)
		return;

	switch (key->both.offset & (FUT_OFF_INODE|FUT_OFF_MMSHARED)) {
	case FUT_OFF_INODE:
		atomic_inc(&key->shared.inode->i_count);
		break;
	case FUT_OFF_MMSHARED:
		atomic_inc(&key->private.mm->mm_count);
		break;
	}
}

static void drop_futex_key_refs(union futex_key *key)
{
	if (!key->both.ptr) {
		/* If we're here then we tried to put a key we failed to get */
		WARN_ON_ONCE(1);
		return;
	}

	switch (key->both.offset & (FUT_OFF_INODE|FUT_OFF_MMSHARED)) {
	case FUT_OFF_INODE:
		iput(key->shared.inode);
		break;
	case FUT_OFF_MMSHARED:
		mmdrop(key->private.mm);
		break;
	}
}

static int
get_futex_key(u32 __user *uaddr, int fshared, union futex_key *key)
{
	unsigned long address = (unsigned long)uaddr;
	struct mm_struct *mm = current->mm;
	struct page *page;
	int err;
	struct vm_area_struct *vma;

	/*
	 * The futex address must be "naturally" aligned.
	 */
	key->both.offset = address % PAGE_SIZE;
	if (unlikely((address % sizeof(u32)) != 0))
		return -EINVAL;
	address -= key->both.offset;

	/*
	 * PROCESS_PRIVATE futexes are fast.
	 * As the mm cannot disappear under us and the 'key' only needs
	 * virtual address, we dont even have to find the underlying vma.
	 * Note : We do have to check 'uaddr' is a valid user address,
	 *        but access_ok() should be faster than find_vma()
	 */
	if (!fshared) {
		if (unlikely(!access_ok(VERIFY_WRITE, uaddr, sizeof(u32))))
			return -EFAULT;
		key->private.mm = mm;
		key->private.address = address;
		get_futex_key_refs(key);
		return 0;
	}

	/*
	 * The futex is hashed differently depending on whether
	 * it's in a shared or private mapping.  So check vma first.
	 */
	vma = find_extend_vma(mm, address);
	if (unlikely(!vma))
		return -EFAULT;

	/*
	 * Permissions.
	 */
	if (unlikely((vma->vm_flags & (VM_IO|VM_READ)) != VM_READ))
		return (vma->vm_flags & VM_IO) ? -EPERM : -EACCES;

	/*
	 * Private mappings are handled in a simple way.
	 *
	 * NOTE: When userspace waits on a MAP_SHARED mapping, even if
	 * it's a read-only handle, it's expected that futexes attach to
	 * the object not the particular process.  Therefore we use
	 * VM_MAYSHARE here, not VM_SHARED which is restricted to shared
	 * mappings of _writable_ handles.
	 */
	if (likely(!(vma->vm_flags & VM_MAYSHARE))) {
		key->both.offset |= FUT_OFF_MMSHARED; /* reference taken on mm */
		key->private.mm = mm;
		key->private.address = address;
		get_futex_key_refs(key);
		return 0;
	}

again:
	err = get_user_pages_fast(address, 1, 1, &page);
	if (err < 0)
		return err;

	page = compound_head(page);
	lock_page(page);
	if (!page->mapping) {
		unlock_page(page);
		put_page(page);
		goto again;
	}

	/*
	 * Private mappings are handled in a simple way.
	 *
	 * NOTE: When userspace waits on a MAP_SHARED mapping, even if
	 * it's a read-only handle, it's expected that futexes attach to
	 * the object not the particular process.
	 */
	if (PageAnon(page)) {
		key->both.offset |= FUT_OFF_MMSHARED; /* ref taken on mm */
		key->private.mm = mm;
		key->private.address = address;
	} else {
		key->both.offset |= FUT_OFF_INODE; /* inode-based key */
		key->shared.inode = page->mapping->host;
		key->shared.pgoff = page->index;
	}

	get_futex_key_refs(key);

	unlock_page(page);
	put_page(page);
	return 0;
}

static inline
void put_futex_key(int fshared, union futex_key *key)
{
	drop_futex_key_refs(key);
}

static int fault_in_user_writeable(u32 __user *uaddr)
{
	struct mm_struct *mm = current->mm;
	int ret;

	down_read(&mm->mmap_sem);
	ret = get_user_pages(current, mm, (unsigned long)uaddr,
			     1, 1, 0, NULL, NULL);
	up_read(&mm->mmap_sem);

	return ret < 0 ? ret : 0;
}

static struct futex_q *futex_top_waiter(struct futex_hash_bucket *hb,
					union futex_key *key)
{
	struct futex_q *this;

	plist_for_each_entry(this, &hb->chain, list) {
		if (match_futex(&this->key, key))
			return this;
	}
	return NULL;
}

static u32 cmpxchg_futex_value_locked(u32 __user *uaddr, u32 uval, u32 newval)
{
	u32 curval;

	pagefault_disable();
	curval = futex_atomic_cmpxchg_inatomic(uaddr, uval, newval);
	pagefault_enable();

	return curval;
}

static int get_futex_value_locked(u32 *dest, u32 __user *from)
{
	int ret;

	pagefault_disable();
	ret = __copy_from_user_inatomic(dest, from, sizeof(u32));
	pagefault_enable();

	return ret ? -EFAULT : 0;
}


static int refill_pi_state_cache(void)
{
	struct futex_pi_state *pi_state;

	if (likely(current->pi_state_cache))
		return 0;

	pi_state = kzalloc(sizeof(*pi_state), GFP_KERNEL);

	if (!pi_state)
		return -ENOMEM;

	INIT_LIST_HEAD(&pi_state->list);
	/* pi_mutex gets initialized later */
	pi_state->owner = NULL;
	atomic_set(&pi_state->refcount, 1);
	pi_state->key = FUTEX_KEY_INIT;

	current->pi_state_cache = pi_state;

	return 0;
}

static struct futex_pi_state * alloc_pi_state(void)
{
	struct futex_pi_state *pi_state = current->pi_state_cache;

	WARN_ON(!pi_state);
	current->pi_state_cache = NULL;

	return pi_state;
}

static void free_pi_state(struct futex_pi_state *pi_state)
{
	if (!atomic_dec_and_test(&pi_state->refcount))
		return;

	/*
	 * If pi_state->owner is NULL, the owner is most probably dying
	 * and has cleaned up the pi_state already
	 */
	if (pi_state->owner) {
		raw_spin_lock_irq(&pi_state->owner->pi_lock);
		list_del_init(&pi_state->list);
		raw_spin_unlock_irq(&pi_state->owner->pi_lock);

		rt_mutex_proxy_unlock(&pi_state->pi_mutex, pi_state->owner);
	}

	if (current->pi_state_cache)
		kfree(pi_state);
	else {
		/*
		 * pi_state->list is already empty.
		 * clear pi_state->owner.
		 * refcount is at 0 - put it back to 1.
		 */
		pi_state->owner = NULL;
		atomic_set(&pi_state->refcount, 1);
		current->pi_state_cache = pi_state;
	}
}

static struct task_struct * futex_find_get_task(pid_t pid)
{
	struct task_struct *p;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (p)
		get_task_struct(p);

	rcu_read_unlock();

	return p;
}

void exit_pi_state_list(struct task_struct *curr)
{
	struct list_head *next, *head = &curr->pi_state_list;
	struct futex_pi_state *pi_state;
	struct futex_hash_bucket *hb;
	union futex_key key = FUTEX_KEY_INIT;

	if (!futex_cmpxchg_enabled)
		return;
	/*
	 * We are a ZOMBIE and nobody can enqueue itself on
	 * pi_state_list anymore, but we have to be careful
	 * versus waiters unqueueing themselves:
	 */
	raw_spin_lock_irq(&curr->pi_lock);
	while (!list_empty(head)) {

		next = head->next;
		pi_state = list_entry(next, struct futex_pi_state, list);
		key = pi_state->key;
		hb = hash_futex(&key);
		raw_spin_unlock_irq(&curr->pi_lock);

		spin_lock(&hb->lock);

		raw_spin_lock_irq(&curr->pi_lock);
		/*
		 * We dropped the pi-lock, so re-check whether this
		 * task still owns the PI-state:
		 */
		if (head->next != next) {
			spin_unlock(&hb->lock);
			continue;
		}

		WARN_ON(pi_state->owner != curr);
		WARN_ON(list_empty(&pi_state->list));
		list_del_init(&pi_state->list);
		pi_state->owner = NULL;
		raw_spin_unlock_irq(&curr->pi_lock);

		rt_mutex_unlock(&pi_state->pi_mutex);

		spin_unlock(&hb->lock);

		raw_spin_lock_irq(&curr->pi_lock);
	}
	raw_spin_unlock_irq(&curr->pi_lock);
}

static int
lookup_pi_state(u32 uval, struct futex_hash_bucket *hb,
		union futex_key *key, struct futex_pi_state **ps)
{
	struct futex_pi_state *pi_state = NULL;
	struct futex_q *this, *next;
	struct plist_head *head;
	struct task_struct *p;
	pid_t pid = uval & FUTEX_TID_MASK;

	head = &hb->chain;

	plist_for_each_entry_safe(this, next, head, list) {
		if (match_futex(&this->key, key)) {
			/*
			 * Another waiter already exists - bump up
			 * the refcount and return its pi_state:
			 */
			pi_state = this->pi_state;
			/*
			 * Userspace might have messed up non PI and PI futexes
			 */
			if (unlikely(!pi_state))
				return -EINVAL;

			WARN_ON(!atomic_read(&pi_state->refcount));

			/*
			 * When pi_state->owner is NULL then the owner died
			 * and another waiter is on the fly. pi_state->owner
			 * is fixed up by the task which acquires
			 * pi_state->rt_mutex.
			 *
			 * We do not check for pid == 0 which can happen when
			 * the owner died and robust_list_exit() cleared the
			 * TID.
			 */
			if (pid && pi_state->owner) {
				/*
				 * Bail out if user space manipulated the
				 * futex value.
				 */
				if (pid != task_pid_vnr(pi_state->owner))
					return -EINVAL;
			}

			atomic_inc(&pi_state->refcount);
			*ps = pi_state;

			return 0;
		}
	}

	/*
	 * We are the first waiter - try to look up the real owner and attach
	 * the new pi_state to it, but bail out when TID = 0
	 */
	if (!pid)
		return -ESRCH;
	p = futex_find_get_task(pid);
	if (!p)
		return -ESRCH;

	/*
	 * We need to look at the task state flags to figure out,
	 * whether the task is exiting. To protect against the do_exit
	 * change of the task flags, we do this protected by
	 * p->pi_lock:
	 */
	raw_spin_lock_irq(&p->pi_lock);
	if (unlikely(p->flags & PF_EXITING)) {
		/*
		 * The task is on the way out. When PF_EXITPIDONE is
		 * set, we know that the task has finished the
		 * cleanup:
		 */
		int ret = (p->flags & PF_EXITPIDONE) ? -ESRCH : -EAGAIN;

		raw_spin_unlock_irq(&p->pi_lock);
		put_task_struct(p);
		return ret;
	}

	pi_state = alloc_pi_state();

	/*
	 * Initialize the pi_mutex in locked state and make 'p'
	 * the owner of it:
	 */
	rt_mutex_init_proxy_locked(&pi_state->pi_mutex, p);

	/* Store the key for possible exit cleanups: */
	pi_state->key = *key;

	WARN_ON(!list_empty(&pi_state->list));
	list_add(&pi_state->list, &p->pi_state_list);
	pi_state->owner = p;
	raw_spin_unlock_irq(&p->pi_lock);

	put_task_struct(p);

	*ps = pi_state;

	return 0;
}

static int futex_lock_pi_atomic(u32 __user *uaddr, struct futex_hash_bucket *hb,
				union futex_key *key,
				struct futex_pi_state **ps,
				struct task_struct *task, int set_waiters)
{
	int lock_taken, ret, ownerdied = 0;
	u32 uval, newval, curval;

retry:
	ret = lock_taken = 0;

	/*
	 * To avoid races, we attempt to take the lock here again
	 * (by doing a 0 -> TID atomic cmpxchg), while holding all
	 * the locks. It will most likely not succeed.
	 */
	newval = task_pid_vnr(task);
	if (set_waiters)
		newval |= FUTEX_WAITERS;

	curval = cmpxchg_futex_value_locked(uaddr, 0, newval);

	if (unlikely(curval == -EFAULT))
		return -EFAULT;

	/*
	 * Detect deadlocks.
	 */
	if ((unlikely((curval & FUTEX_TID_MASK) == task_pid_vnr(task))))
		return -EDEADLK;

	/*
	 * Surprise - we got the lock. Just return to userspace:
	 */
	if (unlikely(!curval))
		return 1;

	uval = curval;

	/*
	 * Set the FUTEX_WAITERS flag, so the owner will know it has someone
	 * to wake at the next unlock.
	 */
	newval = curval | FUTEX_WAITERS;

	/*
	 * There are two cases, where a futex might have no owner (the
	 * owner TID is 0): OWNER_DIED. We take over the futex in this
	 * case. We also do an unconditional take over, when the owner
	 * of the futex died.
	 *
	 * This is safe as we are protected by the hash bucket lock !
	 */
	if (unlikely(ownerdied || !(curval & FUTEX_TID_MASK))) {
		/* Keep the OWNER_DIED bit */
		newval = (curval & ~FUTEX_TID_MASK) | task_pid_vnr(task);
		ownerdied = 0;
		lock_taken = 1;
	}

	curval = cmpxchg_futex_value_locked(uaddr, uval, newval);

	if (unlikely(curval == -EFAULT))
		return -EFAULT;
	if (unlikely(curval != uval))
		goto retry;

	/*
	 * We took the lock due to owner died take over.
	 */
	if (unlikely(lock_taken))
		return 1;

	/*
	 * We dont have the lock. Look up the PI state (or create it if
	 * we are the first waiter):
	 */
	ret = lookup_pi_state(uval, hb, key, ps);

	if (unlikely(ret)) {
		switch (ret) {
		case -ESRCH:
			/*
			 * No owner found for this futex. Check if the
			 * OWNER_DIED bit is set to figure out whether
			 * this is a robust futex or not.
			 */
			if (get_futex_value_locked(&curval, uaddr))
				return -EFAULT;

			/*
			 * We simply start over in case of a robust
			 * futex. The code above will take the futex
			 * and return happy.
			 */
			if (curval & FUTEX_OWNER_DIED) {
				ownerdied = 1;
				goto retry;
			}
		default:
			break;
		}
	}

	return ret;
}

static void wake_futex(struct futex_q *q)
{
	struct task_struct *p = q->task;

	/*
	 * We set q->lock_ptr = NULL _before_ we wake up the task. If
	 * a non futex wake up happens on another CPU then the task
	 * might exit and p would dereference a non existing task
	 * struct. Prevent this by holding a reference on p across the
	 * wake up.
	 */
	get_task_struct(p);

	plist_del(&q->list, &q->list.plist);
	/*
	 * The waiting task can free the futex_q as soon as
	 * q->lock_ptr = NULL is written, without taking any locks. A
	 * memory barrier is required here to prevent the following
	 * store to lock_ptr from getting ahead of the plist_del.
	 */
	smp_wmb();
	q->lock_ptr = NULL;

	wake_up_state(p, TASK_NORMAL);
	put_task_struct(p);
}

static int wake_futex_pi(u32 __user *uaddr, u32 uval, struct futex_q *this)
{
	struct task_struct *new_owner;
	struct futex_pi_state *pi_state = this->pi_state;
	u32 curval, newval;

	if (!pi_state)
		return -EINVAL;

	/*
	 * If current does not own the pi_state then the futex is
	 * inconsistent and user space fiddled with the futex value.
	 */
	if (pi_state->owner != current)
		return -EINVAL;

	raw_spin_lock(&pi_state->pi_mutex.wait_lock);
	new_owner = rt_mutex_next_owner(&pi_state->pi_mutex);

	/*
	 * This happens when we have stolen the lock and the original
	 * pending owner did not enqueue itself back on the rt_mutex.
	 * Thats not a tragedy. We know that way, that a lock waiter
	 * is on the fly. We make the futex_q waiter the pending owner.
	 */
	if (!new_owner)
		new_owner = this->task;

	/*
	 * We pass it to the next owner. (The WAITERS bit is always
	 * kept enabled while there is PI state around. We must also
	 * preserve the owner died bit.)
	 */
	if (!(uval & FUTEX_OWNER_DIED)) {
		int ret = 0;

		newval = FUTEX_WAITERS | task_pid_vnr(new_owner);

		curval = cmpxchg_futex_value_locked(uaddr, uval, newval);

		if (curval == -EFAULT)
			ret = -EFAULT;
		else if (curval != uval)
			ret = -EINVAL;
		if (ret) {
			raw_spin_unlock(&pi_state->pi_mutex.wait_lock);
			return ret;
		}
	}

	raw_spin_lock_irq(&pi_state->owner->pi_lock);
	WARN_ON(list_empty(&pi_state->list));
	list_del_init(&pi_state->list);
	raw_spin_unlock_irq(&pi_state->owner->pi_lock);

	raw_spin_lock_irq(&new_owner->pi_lock);
	WARN_ON(!list_empty(&pi_state->list));
	list_add(&pi_state->list, &new_owner->pi_state_list);
	pi_state->owner = new_owner;
	raw_spin_unlock_irq(&new_owner->pi_lock);

	raw_spin_unlock(&pi_state->pi_mutex.wait_lock);
	rt_mutex_unlock(&pi_state->pi_mutex);

	return 0;
}

static int unlock_futex_pi(u32 __user *uaddr, u32 uval)
{
	u32 oldval;

	/*
	 * There is no waiter, so we unlock the futex. The owner died
	 * bit has not to be preserved here. We are the owner:
	 */
	oldval = cmpxchg_futex_value_locked(uaddr, uval, 0);

	if (oldval == -EFAULT)
		return oldval;
	if (oldval != uval)
		return -EAGAIN;

	return 0;
}

static inline void
double_lock_hb(struct futex_hash_bucket *hb1, struct futex_hash_bucket *hb2)
{
	if (hb1 <= hb2) {
		spin_lock(&hb1->lock);
		if (hb1 < hb2)
			spin_lock_nested(&hb2->lock, SINGLE_DEPTH_NESTING);
	} else { /* hb1 > hb2 */
		spin_lock(&hb2->lock);
		spin_lock_nested(&hb1->lock, SINGLE_DEPTH_NESTING);
	}
}

static inline void
double_unlock_hb(struct futex_hash_bucket *hb1, struct futex_hash_bucket *hb2)
{
	spin_unlock(&hb1->lock);
	if (hb1 != hb2)
		spin_unlock(&hb2->lock);
}

static int futex_wake(u32 __user *uaddr, int fshared, int nr_wake, u32 bitset)
{
	struct futex_hash_bucket *hb;
	struct futex_q *this, *next;
	struct plist_head *head;
	union futex_key key = FUTEX_KEY_INIT;
	int ret;

	if (!bitset)
		return -EINVAL;

	ret = get_futex_key(uaddr, fshared, &key);
	if (unlikely(ret != 0))
		goto out;

	hb = hash_futex(&key);
	spin_lock(&hb->lock);
	head = &hb->chain;

	plist_for_each_entry_safe(this, next, head, list) {
		if (match_futex (&this->key, &key)) {
			if (this->pi_state || this->rt_waiter) {
				ret = -EINVAL;
				break;
			}

			/* Check if one of the bits is set in both bitsets */
			if (!(this->bitset & bitset))
				continue;

			wake_futex(this);
			if (++ret >= nr_wake)
				break;
		}
	}

	spin_unlock(&hb->lock);
	put_futex_key(fshared, &key);
out:
	return ret;
}

static int
futex_wake_op(u32 __user *uaddr1, int fshared, u32 __user *uaddr2,
	      int nr_wake, int nr_wake2, int op)
{
	union futex_key key1 = FUTEX_KEY_INIT, key2 = FUTEX_KEY_INIT;
	struct futex_hash_bucket *hb1, *hb2;
	struct plist_head *head;
	struct futex_q *this, *next;
	int ret, op_ret;

retry:
	ret = get_futex_key(uaddr1, fshared, &key1);
	if (unlikely(ret != 0))
		goto out;
	ret = get_futex_key(uaddr2, fshared, &key2);
	if (unlikely(ret != 0))
		goto out_put_key1;

	hb1 = hash_futex(&key1);
	hb2 = hash_futex(&key2);

retry_private:
	double_lock_hb(hb1, hb2);
	op_ret = futex_atomic_op_inuser(op, uaddr2);
	if (unlikely(op_ret < 0)) {

		double_unlock_hb(hb1, hb2);

#ifndef CONFIG_MMU
		/*
		 * we don't get EFAULT from MMU faults if we don't have an MMU,
		 * but we might get them from range checking
		 */
		ret = op_ret;
		goto out_put_keys;
#endif

		if (unlikely(op_ret != -EFAULT)) {
			ret = op_ret;
			goto out_put_keys;
		}

		ret = fault_in_user_writeable(uaddr2);
		if (ret)
			goto out_put_keys;

		if (!fshared)
			goto retry_private;

		put_futex_key(fshared, &key2);
		put_futex_key(fshared, &key1);
		goto retry;
	}

	head = &hb1->chain;

	plist_for_each_entry_safe(this, next, head, list) {
		if (match_futex (&this->key, &key1)) {
			wake_futex(this);
			if (++ret >= nr_wake)
				break;
		}
	}

	if (op_ret > 0) {
		head = &hb2->chain;

		op_ret = 0;
		plist_for_each_entry_safe(this, next, head, list) {
			if (match_futex (&this->key, &key2)) {
				wake_futex(this);
				if (++op_ret >= nr_wake2)
					break;
			}
		}
		ret += op_ret;
	}

	double_unlock_hb(hb1, hb2);
out_put_keys:
	put_futex_key(fshared, &key2);
out_put_key1:
	put_futex_key(fshared, &key1);
out:
	return ret;
}

static inline
void requeue_futex(struct futex_q *q, struct futex_hash_bucket *hb1,
		   struct futex_hash_bucket *hb2, union futex_key *key2)
{

	/*
	 * If key1 and key2 hash to the same bucket, no need to
	 * requeue.
	 */
	if (likely(&hb1->chain != &hb2->chain)) {
		plist_del(&q->list, &hb1->chain);
		plist_add(&q->list, &hb2->chain);
		q->lock_ptr = &hb2->lock;
#ifdef CONFIG_DEBUG_PI_LIST
		q->list.plist.spinlock = &hb2->lock;
#endif
	}
	get_futex_key_refs(key2);
	q->key = *key2;
}

static inline
void requeue_pi_wake_futex(struct futex_q *q, union futex_key *key,
			   struct futex_hash_bucket *hb)
{
	get_futex_key_refs(key);
	q->key = *key;

	WARN_ON(plist_node_empty(&q->list));
	plist_del(&q->list, &q->list.plist);

	WARN_ON(!q->rt_waiter);
	q->rt_waiter = NULL;

	q->lock_ptr = &hb->lock;
#ifdef CONFIG_DEBUG_PI_LIST
	q->list.plist.spinlock = &hb->lock;
#endif

	wake_up_state(q->task, TASK_NORMAL);
}

static int futex_proxy_trylock_atomic(u32 __user *pifutex,
				 struct futex_hash_bucket *hb1,
				 struct futex_hash_bucket *hb2,
				 union futex_key *key1, union futex_key *key2,
				 struct futex_pi_state **ps, int set_waiters)
{
	struct futex_q *top_waiter = NULL;
	u32 curval;
	int ret;

	if (get_futex_value_locked(&curval, pifutex))
		return -EFAULT;

	/*
	 * Find the top_waiter and determine if there are additional waiters.
	 * If the caller intends to requeue more than 1 waiter to pifutex,
	 * force futex_lock_pi_atomic() to set the FUTEX_WAITERS bit now,
	 * as we have means to handle the possible fault.  If not, don't set
	 * the bit unecessarily as it will force the subsequent unlock to enter
	 * the kernel.
	 */
	top_waiter = futex_top_waiter(hb1, key1);

	/* There are no waiters, nothing for us to do. */
	if (!top_waiter)
		return 0;

	/* Ensure we requeue to the expected futex. */
	if (!match_futex(top_waiter->requeue_pi_key, key2))
		return -EINVAL;

	/*
	 * Try to take the lock for top_waiter.  Set the FUTEX_WAITERS bit in
	 * the contended case or if set_waiters is 1.  The pi_state is returned
	 * in ps in contended cases.
	 */
	ret = futex_lock_pi_atomic(pifutex, hb2, key2, ps, top_waiter->task,
				   set_waiters);
	if (ret == 1)
		requeue_pi_wake_futex(top_waiter, key2, hb2);

	return ret;
}

static int futex_requeue(u32 __user *uaddr1, int fshared, u32 __user *uaddr2,
			 int nr_wake, int nr_requeue, u32 *cmpval,
			 int requeue_pi)
{
	union futex_key key1 = FUTEX_KEY_INIT, key2 = FUTEX_KEY_INIT;
	int drop_count = 0, task_count = 0, ret;
	struct futex_pi_state *pi_state = NULL;
	struct futex_hash_bucket *hb1, *hb2;
	struct plist_head *head1;
	struct futex_q *this, *next;
	u32 curval2;

	if (requeue_pi) {
		/*
		 * requeue_pi requires a pi_state, try to allocate it now
		 * without any locks in case it fails.
		 */
		if (refill_pi_state_cache())
			return -ENOMEM;
		/*
		 * requeue_pi must wake as many tasks as it can, up to nr_wake
		 * + nr_requeue, since it acquires the rt_mutex prior to
		 * returning to userspace, so as to not leave the rt_mutex with
		 * waiters and no owner.  However, second and third wake-ups
		 * cannot be predicted as they involve race conditions with the
		 * first wake and a fault while looking up the pi_state.  Both
		 * pthread_cond_signal() and pthread_cond_broadcast() should
		 * use nr_wake=1.
		 */
		if (nr_wake != 1)
			return -EINVAL;
	}

retry:
	if (pi_state != NULL) {
		/*
		 * We will have to lookup the pi_state again, so free this one
		 * to keep the accounting correct.
		 */
		free_pi_state(pi_state);
		pi_state = NULL;
	}

	ret = get_futex_key(uaddr1, fshared, &key1);
	if (unlikely(ret != 0))
		goto out;
	ret = get_futex_key(uaddr2, fshared, &key2);
	if (unlikely(ret != 0))
		goto out_put_key1;

	hb1 = hash_futex(&key1);
	hb2 = hash_futex(&key2);

retry_private:
	double_lock_hb(hb1, hb2);

	if (likely(cmpval != NULL)) {
		u32 curval;

		ret = get_futex_value_locked(&curval, uaddr1);

		if (unlikely(ret)) {
			double_unlock_hb(hb1, hb2);

			ret = get_user(curval, uaddr1);
			if (ret)
				goto out_put_keys;

			if (!fshared)
				goto retry_private;

			put_futex_key(fshared, &key2);
			put_futex_key(fshared, &key1);
			goto retry;
		}
		if (curval != *cmpval) {
			ret = -EAGAIN;
			goto out_unlock;
		}
	}

	if (requeue_pi && (task_count - nr_wake < nr_requeue)) {
		/*
		 * Attempt to acquire uaddr2 and wake the top waiter. If we
		 * intend to requeue waiters, force setting the FUTEX_WAITERS
		 * bit.  We force this here where we are able to easily handle
		 * faults rather in the requeue loop below.
		 */
		ret = futex_proxy_trylock_atomic(uaddr2, hb1, hb2, &key1,
						 &key2, &pi_state, nr_requeue);

		/*
		 * At this point the top_waiter has either taken uaddr2 or is
		 * waiting on it.  If the former, then the pi_state will not
		 * exist yet, look it up one more time to ensure we have a
		 * reference to it.
		 */
		if (ret == 1) {
			WARN_ON(pi_state);
			drop_count++;
			task_count++;
			ret = get_futex_value_locked(&curval2, uaddr2);
			if (!ret)
				ret = lookup_pi_state(curval2, hb2, &key2,
						      &pi_state);
		}

		switch (ret) {
		case 0:
			break;
		case -EFAULT:
			double_unlock_hb(hb1, hb2);
			put_futex_key(fshared, &key2);
			put_futex_key(fshared, &key1);
			ret = fault_in_user_writeable(uaddr2);
			if (!ret)
				goto retry;
			goto out;
		case -EAGAIN:
			/* The owner was exiting, try again. */
			double_unlock_hb(hb1, hb2);
			put_futex_key(fshared, &key2);
			put_futex_key(fshared, &key1);
			cond_resched();
			goto retry;
		default:
			goto out_unlock;
		}
	}

	head1 = &hb1->chain;
	plist_for_each_entry_safe(this, next, head1, list) {
		if (task_count - nr_wake >= nr_requeue)
			break;

		if (!match_futex(&this->key, &key1))
			continue;

		/*
		 * FUTEX_WAIT_REQEUE_PI and FUTEX_CMP_REQUEUE_PI should always
		 * be paired with each other and no other futex ops.
		 */
		if ((requeue_pi && !this->rt_waiter) ||
		    (!requeue_pi && this->rt_waiter)) {
			ret = -EINVAL;
			break;
		}

		/*
		 * Wake nr_wake waiters.  For requeue_pi, if we acquired the
		 * lock, we already woke the top_waiter.  If not, it will be
		 * woken by futex_unlock_pi().
		 */
		if (++task_count <= nr_wake && !requeue_pi) {
			wake_futex(this);
			continue;
		}

		/* Ensure we requeue to the expected futex for requeue_pi. */
		if (requeue_pi && !match_futex(this->requeue_pi_key, &key2)) {
			ret = -EINVAL;
			break;
		}

		/*
		 * Requeue nr_requeue waiters and possibly one more in the case
		 * of requeue_pi if we couldn't acquire the lock atomically.
		 */
		if (requeue_pi) {
			/* Prepare the waiter to take the rt_mutex. */
			atomic_inc(&pi_state->refcount);
			this->pi_state = pi_state;
			ret = rt_mutex_start_proxy_lock(&pi_state->pi_mutex,
							this->rt_waiter,
							this->task, 1);
			if (ret == 1) {
				/* We got the lock. */
				requeue_pi_wake_futex(this, &key2, hb2);
				drop_count++;
				continue;
			} else if (ret) {
				/* -EDEADLK */
				this->pi_state = NULL;
				free_pi_state(pi_state);
				goto out_unlock;
			}
		}
		requeue_futex(this, hb1, hb2, &key2);
		drop_count++;
	}

out_unlock:
	double_unlock_hb(hb1, hb2);

	/*
	 * drop_futex_key_refs() must be called outside the spinlocks. During
	 * the requeue we moved futex_q's from the hash bucket at key1 to the
	 * one at key2 and updated their key pointer.  We no longer need to
	 * hold the references to key1.
	 */
	while (--drop_count >= 0)
		drop_futex_key_refs(&key1);

out_put_keys:
	put_futex_key(fshared, &key2);
out_put_key1:
	put_futex_key(fshared, &key1);
out:
	if (pi_state != NULL)
		free_pi_state(pi_state);
	return ret ? ret : task_count;
}

/* The key must be already stored in q->key. */
static inline struct futex_hash_bucket *queue_lock(struct futex_q *q)
{
	struct futex_hash_bucket *hb;

	get_futex_key_refs(&q->key);
	hb = hash_futex(&q->key);
	q->lock_ptr = &hb->lock;

	spin_lock(&hb->lock);
	return hb;
}

static inline void
queue_unlock(struct futex_q *q, struct futex_hash_bucket *hb)
{
	spin_unlock(&hb->lock);
	drop_futex_key_refs(&q->key);
}

static inline void queue_me(struct futex_q *q, struct futex_hash_bucket *hb)
{
	int prio;

	/*
	 * The priority used to register this element is
	 * - either the real thread-priority for the real-time threads
	 * (i.e. threads with a priority lower than MAX_RT_PRIO)
	 * - or MAX_RT_PRIO for non-RT threads.
	 * Thus, all RT-threads are woken first in priority order, and
	 * the others are woken last, in FIFO order.
	 */
	prio = min(current->normal_prio, MAX_RT_PRIO);

	plist_node_init(&q->list, prio);
#ifdef CONFIG_DEBUG_PI_LIST
	q->list.plist.spinlock = &hb->lock;
#endif
	plist_add(&q->list, &hb->chain);
	q->task = current;
	spin_unlock(&hb->lock);
}

static int unqueue_me(struct futex_q *q)
{
	spinlock_t *lock_ptr;
	int ret = 0;

	/* In the common case we don't take the spinlock, which is nice. */
retry:
	lock_ptr = q->lock_ptr;
	barrier();
	if (lock_ptr != NULL) {
		spin_lock(lock_ptr);
		/*
		 * q->lock_ptr can change between reading it and
		 * spin_lock(), causing us to take the wrong lock.  This
		 * corrects the race condition.
		 *
		 * Reasoning goes like this: if we have the wrong lock,
		 * q->lock_ptr must have changed (maybe several times)
		 * between reading it and the spin_lock().  It can
		 * change again after the spin_lock() but only if it was
		 * already changed before the spin_lock().  It cannot,
		 * however, change back to the original value.  Therefore
		 * we can detect whether we acquired the correct lock.
		 */
		if (unlikely(lock_ptr != q->lock_ptr)) {
			spin_unlock(lock_ptr);
			goto retry;
		}
		WARN_ON(plist_node_empty(&q->list));
		plist_del(&q->list, &q->list.plist);

		BUG_ON(q->pi_state);

		spin_unlock(lock_ptr);
		ret = 1;
	}

	drop_futex_key_refs(&q->key);
	return ret;
}

static void unqueue_me_pi(struct futex_q *q)
{
	WARN_ON(plist_node_empty(&q->list));
	plist_del(&q->list, &q->list.plist);

	BUG_ON(!q->pi_state);
	free_pi_state(q->pi_state);
	q->pi_state = NULL;

	spin_unlock(q->lock_ptr);

	drop_futex_key_refs(&q->key);
}

static int fixup_pi_state_owner(u32 __user *uaddr, struct futex_q *q,
				struct task_struct *newowner, int fshared)
{
	u32 newtid = task_pid_vnr(newowner) | FUTEX_WAITERS;
	struct futex_pi_state *pi_state = q->pi_state;
	struct task_struct *oldowner = pi_state->owner;
	u32 uval, curval, newval;
	int ret;

	/* Owner died? */
	if (!pi_state->owner)
		newtid |= FUTEX_OWNER_DIED;

	/*
	 * We are here either because we stole the rtmutex from the
	 * pending owner or we are the pending owner which failed to
	 * get the rtmutex. We have to replace the pending owner TID
	 * in the user space variable. This must be atomic as we have
	 * to preserve the owner died bit here.
	 *
	 * Note: We write the user space value _before_ changing the pi_state
	 * because we can fault here. Imagine swapped out pages or a fork
	 * that marked all the anonymous memory readonly for cow.
	 *
	 * Modifying pi_state _before_ the user space value would
	 * leave the pi_state in an inconsistent state when we fault
	 * here, because we need to drop the hash bucket lock to
	 * handle the fault. This might be observed in the PID check
	 * in lookup_pi_state.
	 */
retry:
	if (get_futex_value_locked(&uval, uaddr))
		goto handle_fault;

	while (1) {
		newval = (uval & FUTEX_OWNER_DIED) | newtid;

		curval = cmpxchg_futex_value_locked(uaddr, uval, newval);

		if (curval == -EFAULT)
			goto handle_fault;
		if (curval == uval)
			break;
		uval = curval;
	}

	/*
	 * We fixed up user space. Now we need to fix the pi_state
	 * itself.
	 */
	if (pi_state->owner != NULL) {
		raw_spin_lock_irq(&pi_state->owner->pi_lock);
		WARN_ON(list_empty(&pi_state->list));
		list_del_init(&pi_state->list);
		raw_spin_unlock_irq(&pi_state->owner->pi_lock);
	}

	pi_state->owner = newowner;

	raw_spin_lock_irq(&newowner->pi_lock);
	WARN_ON(!list_empty(&pi_state->list));
	list_add(&pi_state->list, &newowner->pi_state_list);
	raw_spin_unlock_irq(&newowner->pi_lock);
	return 0;

	/*
	 * To handle the page fault we need to drop the hash bucket
	 * lock here. That gives the other task (either the pending
	 * owner itself or the task which stole the rtmutex) the
	 * chance to try the fixup of the pi_state. So once we are
	 * back from handling the fault we need to check the pi_state
	 * after reacquiring the hash bucket lock and before trying to
	 * do another fixup. When the fixup has been done already we
	 * simply return.
	 */
handle_fault:
	spin_unlock(q->lock_ptr);

	ret = fault_in_user_writeable(uaddr);

	spin_lock(q->lock_ptr);

	/*
	 * Check if someone else fixed it for us:
	 */
	if (pi_state->owner != oldowner)
		return 0;

	if (ret)
		return ret;

	goto retry;
}

#define FLAGS_SHARED		0x01
#define FLAGS_CLOCKRT		0x02
#define FLAGS_HAS_TIMEOUT	0x04

static long futex_wait_restart(struct restart_block *restart);

static int fixup_owner(u32 __user *uaddr, int fshared, struct futex_q *q,
		       int locked)
{
	struct task_struct *owner;
	int ret = 0;

	if (locked) {
		/*
		 * Got the lock. We might not be the anticipated owner if we
		 * did a lock-steal - fix up the PI-state in that case:
		 */
		if (q->pi_state->owner != current)
			ret = fixup_pi_state_owner(uaddr, q, current, fshared);
		goto out;
	}

	/*
	 * Catch the rare case, where the lock was released when we were on the
	 * way back before we locked the hash bucket.
	 */
	if (q->pi_state->owner == current) {
		/*
		 * Try to get the rt_mutex now. This might fail as some other
		 * task acquired the rt_mutex after we removed ourself from the
		 * rt_mutex waiters list.
		 */
		if (rt_mutex_trylock(&q->pi_state->pi_mutex)) {
			locked = 1;
			goto out;
		}

		/*
		 * pi_state is incorrect, some other task did a lock steal and
		 * we returned due to timeout or signal without taking the
		 * rt_mutex. Too late. We can access the rt_mutex_owner without
		 * locking, as the other task is now blocked on the hash bucket
		 * lock. Fix the state up.
		 */
		owner = rt_mutex_owner(&q->pi_state->pi_mutex);
		ret = fixup_pi_state_owner(uaddr, q, owner, fshared);
		goto out;
	}

	/*
	 * Paranoia check. If we did not take the lock, then we should not be
	 * the owner, nor the pending owner, of the rt_mutex.
	 */
	if (rt_mutex_owner(&q->pi_state->pi_mutex) == current)
		printk(KERN_ERR "fixup_owner: ret = %d pi-mutex: %p "
				"pi-state %p\n", ret,
				q->pi_state->pi_mutex.owner,
				q->pi_state->owner);

out:
	return ret ? ret : locked;
}

static void futex_wait_queue_me(struct futex_hash_bucket *hb, struct futex_q *q,
				struct hrtimer_sleeper *timeout)
{
	/*
	 * The task state is guaranteed to be set before another task can
	 * wake it. set_current_state() is implemented using set_mb() and
	 * queue_me() calls spin_unlock() upon completion, both serializing
	 * access to the hash list and forcing another memory barrier.
	 */
	set_current_state(TASK_INTERRUPTIBLE);
	queue_me(q, hb);

	/* Arm the timer */
	if (timeout) {
		hrtimer_start_expires(&timeout->timer, HRTIMER_MODE_ABS);
		if (!hrtimer_active(&timeout->timer))
			timeout->task = NULL;
	}

	/*
	 * If we have been removed from the hash list, then another task
	 * has tried to wake us, and we can skip the call to schedule().
	 */
	if (likely(!plist_node_empty(&q->list))) {
		/*
		 * If the timer has already expired, current will already be
		 * flagged for rescheduling. Only call schedule if there
		 * is no timeout, or if it has yet to expire.
		 */
		if (!timeout || timeout->task)
			schedule();
	}
	__set_current_state(TASK_RUNNING);
}

static int futex_wait_setup(u32 __user *uaddr, u32 val, int fshared,
			   struct futex_q *q, struct futex_hash_bucket **hb)
{
	u32 uval;
	int ret;

	/*
	 * Access the page AFTER the hash-bucket is locked.
	 * Order is important:
	 *
	 *   Userspace waiter: val = var; if (cond(val)) futex_wait(&var, val);
	 *   Userspace waker:  if (cond(var)) { var = new; futex_wake(&var); }
	 *
	 * The basic logical guarantee of a futex is that it blocks ONLY
	 * if cond(var) is known to be true at the time of blocking, for
	 * any cond.  If we queued after testing *uaddr, that would open
	 * a race condition where we could block indefinitely with
	 * cond(var) false, which would violate the guarantee.
	 *
	 * A consequence is that futex_wait() can return zero and absorb
	 * a wakeup when *uaddr != val on entry to the syscall.  This is
	 * rare, but normal.
	 */
retry:
	q->key = FUTEX_KEY_INIT;
	ret = get_futex_key(uaddr, fshared, &q->key);
	if (unlikely(ret != 0))
		return ret;

retry_private:
	*hb = queue_lock(q);

	ret = get_futex_value_locked(&uval, uaddr);

	if (ret) {
		queue_unlock(q, *hb);

		ret = get_user(uval, uaddr);
		if (ret)
			goto out;

		if (!fshared)
			goto retry_private;

		put_futex_key(fshared, &q->key);
		goto retry;
	}

	if (uval != val) {
		queue_unlock(q, *hb);
		ret = -EWOULDBLOCK;
	}

out:
	if (ret)
		put_futex_key(fshared, &q->key);
	return ret;
}

static int futex_wait(u32 __user *uaddr, int fshared,
		      u32 val, ktime_t *abs_time, u32 bitset, int clockrt)
{
	struct hrtimer_sleeper timeout, *to = NULL;
	struct restart_block *restart;
	struct futex_hash_bucket *hb;
	struct futex_q q;
	int ret;

	if (!bitset)
		return -EINVAL;

	q.pi_state = NULL;
	q.bitset = bitset;
	q.rt_waiter = NULL;
	q.requeue_pi_key = NULL;

	if (abs_time) {
		to = &timeout;

		hrtimer_init_on_stack(&to->timer, clockrt ? CLOCK_REALTIME :
				      CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
		hrtimer_init_sleeper(to, current);
		hrtimer_set_expires_range_ns(&to->timer, *abs_time,
					     current->timer_slack_ns);
	}

retry:
	/* Prepare to wait on uaddr. */
	ret = futex_wait_setup(uaddr, val, fshared, &q, &hb);
	if (ret)
		goto out;

	/* queue_me and wait for wakeup, timeout, or a signal. */
	futex_wait_queue_me(hb, &q, to);

	/* If we were woken (and unqueued), we succeeded, whatever. */
	ret = 0;
	if (!unqueue_me(&q))
		goto out_put_key;
	ret = -ETIMEDOUT;
	if (to && !to->task)
		goto out_put_key;

	/*
	 * We expect signal_pending(current), but we might be the
	 * victim of a spurious wakeup as well.
	 */
	if (!signal_pending(current)) {
		put_futex_key(fshared, &q.key);
		goto retry;
	}

	ret = -ERESTARTSYS;
	if (!abs_time)
		goto out_put_key;

	restart = &current_thread_info()->restart_block;
	restart->fn = futex_wait_restart;
	restart->futex.uaddr = (u32 *)uaddr;
	restart->futex.val = val;
	restart->futex.time = abs_time->tv64;
	restart->futex.bitset = bitset;
	restart->futex.flags = FLAGS_HAS_TIMEOUT;

	if (fshared)
		restart->futex.flags |= FLAGS_SHARED;
	if (clockrt)
		restart->futex.flags |= FLAGS_CLOCKRT;

	ret = -ERESTART_RESTARTBLOCK;

out_put_key:
	put_futex_key(fshared, &q.key);
out:
	if (to) {
		hrtimer_cancel(&to->timer);
		destroy_hrtimer_on_stack(&to->timer);
	}
	return ret;
}


static long futex_wait_restart(struct restart_block *restart)
{
	u32 __user *uaddr = (u32 __user *)restart->futex.uaddr;
	int fshared = 0;
	ktime_t t, *tp = NULL;

	if (restart->futex.flags & FLAGS_HAS_TIMEOUT) {
		t.tv64 = restart->futex.time;
		tp = &t;
	}
	restart->fn = do_no_restart_syscall;
	if (restart->futex.flags & FLAGS_SHARED)
		fshared = 1;
	return (long)futex_wait(uaddr, fshared, restart->futex.val, tp,
				restart->futex.bitset,
				restart->futex.flags & FLAGS_CLOCKRT);
}


static int futex_lock_pi(u32 __user *uaddr, int fshared,
			 int detect, ktime_t *time, int trylock)
{
	struct hrtimer_sleeper timeout, *to = NULL;
	struct futex_hash_bucket *hb;
	struct futex_q q;
	int res, ret;

	if (refill_pi_state_cache())
		return -ENOMEM;

	if (time) {
		to = &timeout;
		hrtimer_init_on_stack(&to->timer, CLOCK_REALTIME,
				      HRTIMER_MODE_ABS);
		hrtimer_init_sleeper(to, current);
		hrtimer_set_expires(&to->timer, *time);
	}

	q.pi_state = NULL;
	q.rt_waiter = NULL;
	q.requeue_pi_key = NULL;
retry:
	q.key = FUTEX_KEY_INIT;
	ret = get_futex_key(uaddr, fshared, &q.key);
	if (unlikely(ret != 0))
		goto out;

retry_private:
	hb = queue_lock(&q);

	ret = futex_lock_pi_atomic(uaddr, hb, &q.key, &q.pi_state, current, 0);
	if (unlikely(ret)) {
		switch (ret) {
		case 1:
			/* We got the lock. */
			ret = 0;
			goto out_unlock_put_key;
		case -EFAULT:
			goto uaddr_faulted;
		case -EAGAIN:
			/*
			 * Task is exiting and we just wait for the
			 * exit to complete.
			 */
			queue_unlock(&q, hb);
			put_futex_key(fshared, &q.key);
			cond_resched();
			goto retry;
		default:
			goto out_unlock_put_key;
		}
	}

	/*
	 * Only actually queue now that the atomic ops are done:
	 */
	queue_me(&q, hb);

	WARN_ON(!q.pi_state);
	/*
	 * Block on the PI mutex:
	 */
	if (!trylock)
		ret = rt_mutex_timed_lock(&q.pi_state->pi_mutex, to, 1);
	else {
		ret = rt_mutex_trylock(&q.pi_state->pi_mutex);
		/* Fixup the trylock return value: */
		ret = ret ? 0 : -EWOULDBLOCK;
	}

	spin_lock(q.lock_ptr);
	/*
	 * Fixup the pi_state owner and possibly acquire the lock if we
	 * haven't already.
	 */
	res = fixup_owner(uaddr, fshared, &q, !ret);
	/*
	 * If fixup_owner() returned an error, proprogate that.  If it acquired
	 * the lock, clear our -ETIMEDOUT or -EINTR.
	 */
	if (res)
		ret = (res < 0) ? res : 0;

	/*
	 * If fixup_owner() faulted and was unable to handle the fault, unlock
	 * it and return the fault to userspace.
	 */
	if (ret && (rt_mutex_owner(&q.pi_state->pi_mutex) == current))
		rt_mutex_unlock(&q.pi_state->pi_mutex);

	/* Unqueue and drop the lock */
	unqueue_me_pi(&q);

	goto out_put_key;

out_unlock_put_key:
	queue_unlock(&q, hb);

out_put_key:
	put_futex_key(fshared, &q.key);
out:
	if (to)
		destroy_hrtimer_on_stack(&to->timer);
	return ret != -EINTR ? ret : -ERESTARTNOINTR;

uaddr_faulted:
	queue_unlock(&q, hb);

	ret = fault_in_user_writeable(uaddr);
	if (ret)
		goto out_put_key;

	if (!fshared)
		goto retry_private;

	put_futex_key(fshared, &q.key);
	goto retry;
}

static int futex_unlock_pi(u32 __user *uaddr, int fshared)
{
	struct futex_hash_bucket *hb;
	struct futex_q *this, *next;
	u32 uval;
	struct plist_head *head;
	union futex_key key = FUTEX_KEY_INIT;
	int ret;

retry:
	if (get_user(uval, uaddr))
		return -EFAULT;
	/*
	 * We release only a lock we actually own:
	 */
	if ((uval & FUTEX_TID_MASK) != task_pid_vnr(current))
		return -EPERM;

	ret = get_futex_key(uaddr, fshared, &key);
	if (unlikely(ret != 0))
		goto out;

	hb = hash_futex(&key);
	spin_lock(&hb->lock);

	/*
	 * To avoid races, try to do the TID -> 0 atomic transition
	 * again. If it succeeds then we can return without waking
	 * anyone else up:
	 */
	if (!(uval & FUTEX_OWNER_DIED))
		uval = cmpxchg_futex_value_locked(uaddr, task_pid_vnr(current), 0);


	if (unlikely(uval == -EFAULT))
		goto pi_faulted;
	/*
	 * Rare case: we managed to release the lock atomically,
	 * no need to wake anyone else up:
	 */
	if (unlikely(uval == task_pid_vnr(current)))
		goto out_unlock;

	/*
	 * Ok, other tasks may need to be woken up - check waiters
	 * and do the wakeup if necessary:
	 */
	head = &hb->chain;

	plist_for_each_entry_safe(this, next, head, list) {
		if (!match_futex (&this->key, &key))
			continue;
		ret = wake_futex_pi(uaddr, uval, this);
		/*
		 * The atomic access to the futex value
		 * generated a pagefault, so retry the
		 * user-access and the wakeup:
		 */
		if (ret == -EFAULT)
			goto pi_faulted;
		goto out_unlock;
	}
	/*
	 * No waiters - kernel unlocks the futex:
	 */
	if (!(uval & FUTEX_OWNER_DIED)) {
		ret = unlock_futex_pi(uaddr, uval);
		if (ret == -EFAULT)
			goto pi_faulted;
	}

out_unlock:
	spin_unlock(&hb->lock);
	put_futex_key(fshared, &key);

out:
	return ret;

pi_faulted:
	spin_unlock(&hb->lock);
	put_futex_key(fshared, &key);

	ret = fault_in_user_writeable(uaddr);
	if (!ret)
		goto retry;

	return ret;
}

static inline
int handle_early_requeue_pi_wakeup(struct futex_hash_bucket *hb,
				   struct futex_q *q, union futex_key *key2,
				   struct hrtimer_sleeper *timeout)
{
	int ret = 0;

	/*
	 * With the hb lock held, we avoid races while we process the wakeup.
	 * We only need to hold hb (and not hb2) to ensure atomicity as the
	 * wakeup code can't change q.key from uaddr to uaddr2 if we hold hb.
	 * It can't be requeued from uaddr2 to something else since we don't
	 * support a PI aware source futex for requeue.
	 */
	if (!match_futex(&q->key, key2)) {
		WARN_ON(q->lock_ptr && (&hb->lock != q->lock_ptr));
		/*
		 * We were woken prior to requeue by a timeout or a signal.
		 * Unqueue the futex_q and determine which it was.
		 */
		plist_del(&q->list, &q->list.plist);

		/* Handle spurious wakeups gracefully */
		ret = -EWOULDBLOCK;
		if (timeout && !timeout->task)
			ret = -ETIMEDOUT;
		else if (signal_pending(current))
			ret = -ERESTARTNOINTR;
	}
	return ret;
}

static int futex_wait_requeue_pi(u32 __user *uaddr, int fshared,
				 u32 val, ktime_t *abs_time, u32 bitset,
				 int clockrt, u32 __user *uaddr2)
{
	struct hrtimer_sleeper timeout, *to = NULL;
	struct rt_mutex_waiter rt_waiter;
	struct rt_mutex *pi_mutex = NULL;
	struct futex_hash_bucket *hb;
	union futex_key key2;
	struct futex_q q;
	int res, ret;

	if (!bitset)
		return -EINVAL;

	if (abs_time) {
		to = &timeout;
		hrtimer_init_on_stack(&to->timer, clockrt ? CLOCK_REALTIME :
				      CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
		hrtimer_init_sleeper(to, current);
		hrtimer_set_expires_range_ns(&to->timer, *abs_time,
					     current->timer_slack_ns);
	}

	/*
	 * The waiter is allocated on our stack, manipulated by the requeue
	 * code while we sleep on uaddr.
	 */
	debug_rt_mutex_init_waiter(&rt_waiter);
	rt_waiter.task = NULL;

	key2 = FUTEX_KEY_INIT;
	ret = get_futex_key(uaddr2, fshared, &key2);
	if (unlikely(ret != 0))
		goto out;

	q.pi_state = NULL;
	q.bitset = bitset;
	q.rt_waiter = &rt_waiter;
	q.requeue_pi_key = &key2;

	/* Prepare to wait on uaddr. */
	ret = futex_wait_setup(uaddr, val, fshared, &q, &hb);
	if (ret)
		goto out_key2;

	/* Queue the futex_q, drop the hb lock, wait for wakeup. */
	futex_wait_queue_me(hb, &q, to);

	spin_lock(&hb->lock);
	ret = handle_early_requeue_pi_wakeup(hb, &q, &key2, to);
	spin_unlock(&hb->lock);
	if (ret)
		goto out_put_keys;

	/*
	 * In order for us to be here, we know our q.key == key2, and since
	 * we took the hb->lock above, we also know that futex_requeue() has
	 * completed and we no longer have to concern ourselves with a wakeup
	 * race with the atomic proxy lock acquition by the requeue code.
	 */

	/* Check if the requeue code acquired the second futex for us. */
	if (!q.rt_waiter) {
		/*
		 * Got the lock. We might not be the anticipated owner if we
		 * did a lock-steal - fix up the PI-state in that case.
		 */
		if (q.pi_state && (q.pi_state->owner != current)) {
			spin_lock(q.lock_ptr);
			ret = fixup_pi_state_owner(uaddr2, &q, current,
						   fshared);
			spin_unlock(q.lock_ptr);
		}
	} else {
		/*
		 * We have been woken up by futex_unlock_pi(), a timeout, or a
		 * signal.  futex_unlock_pi() will not destroy the lock_ptr nor
		 * the pi_state.
		 */
		WARN_ON(!&q.pi_state);
		pi_mutex = &q.pi_state->pi_mutex;
		ret = rt_mutex_finish_proxy_lock(pi_mutex, to, &rt_waiter, 1);
		debug_rt_mutex_free_waiter(&rt_waiter);

		spin_lock(q.lock_ptr);
		/*
		 * Fixup the pi_state owner and possibly acquire the lock if we
		 * haven't already.
		 */
		res = fixup_owner(uaddr2, fshared, &q, !ret);
		/*
		 * If fixup_owner() returned an error, proprogate that.  If it
		 * acquired the lock, clear -ETIMEDOUT or -EINTR.
		 */
		if (res)
			ret = (res < 0) ? res : 0;

		/* Unqueue and drop the lock. */
		unqueue_me_pi(&q);
	}

	/*
	 * If fixup_pi_state_owner() faulted and was unable to handle the
	 * fault, unlock the rt_mutex and return the fault to userspace.
	 */
	if (ret == -EFAULT) {
		if (rt_mutex_owner(pi_mutex) == current)
			rt_mutex_unlock(pi_mutex);
	} else if (ret == -EINTR) {
		/*
		 * We've already been requeued, but cannot restart by calling
		 * futex_lock_pi() directly. We could restart this syscall, but
		 * it would detect that the user space "val" changed and return
		 * -EWOULDBLOCK.  Save the overhead of the restart and return
		 * -EWOULDBLOCK directly.
		 */
		ret = -EWOULDBLOCK;
	}

out_put_keys:
	put_futex_key(fshared, &q.key);
out_key2:
	put_futex_key(fshared, &key2);

out:
	if (to) {
		hrtimer_cancel(&to->timer);
		destroy_hrtimer_on_stack(&to->timer);
	}
	return ret;
}


SYSCALL_DEFINE2(set_robust_list, struct robust_list_head __user *, head,
		size_t, len)
{
	if (!futex_cmpxchg_enabled)
		return -ENOSYS;
	/*
	 * The kernel knows only one size for now:
	 */
	if (unlikely(len != sizeof(*head)))
		return -EINVAL;

	current->robust_list = head;

	return 0;
}

SYSCALL_DEFINE3(get_robust_list, int, pid,
		struct robust_list_head __user * __user *, head_ptr,
		size_t __user *, len_ptr)
{
	struct robust_list_head __user *head;
	unsigned long ret;
	const struct cred *cred = current_cred(), *pcred;

	if (!futex_cmpxchg_enabled)
		return -ENOSYS;

	if (!pid)
		head = current->robust_list;
	else {
		struct task_struct *p;

		ret = -ESRCH;
		rcu_read_lock();
		p = find_task_by_vpid(pid);
		if (!p)
			goto err_unlock;
		ret = -EPERM;
		pcred = __task_cred(p);
		if (cred->euid != pcred->euid &&
		    cred->euid != pcred->uid &&
		    !capable(CAP_SYS_PTRACE))
			goto err_unlock;
		head = p->robust_list;
		rcu_read_unlock();
	}

	if (put_user(sizeof(*head), len_ptr))
		return -EFAULT;
	return put_user(head, head_ptr);

err_unlock:
	rcu_read_unlock();

	return ret;
}

int handle_futex_death(u32 __user *uaddr, struct task_struct *curr, int pi)
{
	u32 uval, nval, mval;

retry:
	if (get_user(uval, uaddr))
		return -1;

	if ((uval & FUTEX_TID_MASK) == task_pid_vnr(curr)) {
		/*
		 * Ok, this dying thread is truly holding a futex
		 * of interest. Set the OWNER_DIED bit atomically
		 * via cmpxchg, and if the value had FUTEX_WAITERS
		 * set, wake up a waiter (if any). (We have to do a
		 * futex_wake() even if OWNER_DIED is already set -
		 * to handle the rare but possible case of recursive
		 * thread-death.) The rest of the cleanup is done in
		 * userspace.
		 */
		mval = (uval & FUTEX_WAITERS) | FUTEX_OWNER_DIED;
		nval = futex_atomic_cmpxchg_inatomic(uaddr, uval, mval);

		if (nval == -EFAULT)
			return -1;

		if (nval != uval)
			goto retry;

		/*
		 * Wake robust non-PI futexes here. The wakeup of
		 * PI futexes happens in exit_pi_state():
		 */
		if (!pi && (uval & FUTEX_WAITERS))
			futex_wake(uaddr, 1, 1, FUTEX_BITSET_MATCH_ANY);
	}
	return 0;
}

static inline int fetch_robust_entry(struct robust_list __user **entry,
				     struct robust_list __user * __user *head,
				     int *pi)
{
	unsigned long uentry;

	if (get_user(uentry, (unsigned long __user *)head))
		return -EFAULT;

	*entry = (void __user *)(uentry & ~1UL);
	*pi = uentry & 1;

	return 0;
}

void exit_robust_list(struct task_struct *curr)
{
	struct robust_list_head __user *head = curr->robust_list;
	struct robust_list __user *entry, *next_entry, *pending;
	unsigned int limit = ROBUST_LIST_LIMIT, pi, next_pi, pip;
	unsigned long futex_offset;
	int rc;

	if (!futex_cmpxchg_enabled)
		return;

	/*
	 * Fetch the list head (which was registered earlier, via
	 * sys_set_robust_list()):
	 */
	if (fetch_robust_entry(&entry, &head->list.next, &pi))
		return;
	/*
	 * Fetch the relative futex offset:
	 */
	if (get_user(futex_offset, &head->futex_offset))
		return;
	/*
	 * Fetch any possibly pending lock-add first, and handle it
	 * if it exists:
	 */
	if (fetch_robust_entry(&pending, &head->list_op_pending, &pip))
		return;

	next_entry = NULL;	/* avoid warning with gcc */
	while (entry != &head->list) {
		/*
		 * Fetch the next entry in the list before calling
		 * handle_futex_death:
		 */
		rc = fetch_robust_entry(&next_entry, &entry->next, &next_pi);
		/*
		 * A pending lock might already be on the list, so
		 * don't process it twice:
		 */
		if (entry != pending)
			if (handle_futex_death((void __user *)entry + futex_offset,
						curr, pi))
				return;
		if (rc)
			return;
		entry = next_entry;
		pi = next_pi;
		/*
		 * Avoid excessively long or circular lists:
		 */
		if (!--limit)
			break;

		cond_resched();
	}

	if (pending)
		handle_futex_death((void __user *)pending + futex_offset,
				   curr, pip);
}

long do_futex(u32 __user *uaddr, int op, u32 val, ktime_t *timeout,
		u32 __user *uaddr2, u32 val2, u32 val3)
{
	int clockrt, ret = -ENOSYS;
	int cmd = op & FUTEX_CMD_MASK;
	int fshared = 0;

	if (!(op & FUTEX_PRIVATE_FLAG))
		fshared = 1;

	clockrt = op & FUTEX_CLOCK_REALTIME;
	if (clockrt && cmd != FUTEX_WAIT_BITSET && cmd != FUTEX_WAIT_REQUEUE_PI)
		return -ENOSYS;

	switch (cmd) {
	case FUTEX_WAIT:
		val3 = FUTEX_BITSET_MATCH_ANY;
	case FUTEX_WAIT_BITSET:
		ret = futex_wait(uaddr, fshared, val, timeout, val3, clockrt);
		break;
	case FUTEX_WAKE:
		val3 = FUTEX_BITSET_MATCH_ANY;
	case FUTEX_WAKE_BITSET:
		ret = futex_wake(uaddr, fshared, val, val3);
		break;
	case FUTEX_REQUEUE:
		ret = futex_requeue(uaddr, fshared, uaddr2, val, val2, NULL, 0);
		break;
	case FUTEX_CMP_REQUEUE:
		ret = futex_requeue(uaddr, fshared, uaddr2, val, val2, &val3,
				    0);
		break;
	case FUTEX_WAKE_OP:
		ret = futex_wake_op(uaddr, fshared, uaddr2, val, val2, val3);
		break;
	case FUTEX_LOCK_PI:
		if (futex_cmpxchg_enabled)
			ret = futex_lock_pi(uaddr, fshared, val, timeout, 0);
		break;
	case FUTEX_UNLOCK_PI:
		if (futex_cmpxchg_enabled)
			ret = futex_unlock_pi(uaddr, fshared);
		break;
	case FUTEX_TRYLOCK_PI:
		if (futex_cmpxchg_enabled)
			ret = futex_lock_pi(uaddr, fshared, 0, timeout, 1);
		break;
	case FUTEX_WAIT_REQUEUE_PI:
		val3 = FUTEX_BITSET_MATCH_ANY;
		ret = futex_wait_requeue_pi(uaddr, fshared, val, timeout, val3,
					    clockrt, uaddr2);
		break;
	case FUTEX_CMP_REQUEUE_PI:
		ret = futex_requeue(uaddr, fshared, uaddr2, val, val2, &val3,
				    1);
		break;
	default:
		ret = -ENOSYS;
	}
	return ret;
}


SYSCALL_DEFINE6(futex, u32 __user *, uaddr, int, op, u32, val,
		struct timespec __user *, utime, u32 __user *, uaddr2,
		u32, val3)
{
	struct timespec ts;
	ktime_t t, *tp = NULL;
	u32 val2 = 0;
	int cmd = op & FUTEX_CMD_MASK;

	if (utime && (cmd == FUTEX_WAIT || cmd == FUTEX_LOCK_PI ||
		      cmd == FUTEX_WAIT_BITSET ||
		      cmd == FUTEX_WAIT_REQUEUE_PI)) {
		if (copy_from_user(&ts, utime, sizeof(ts)) != 0)
			return -EFAULT;
		if (!timespec_valid(&ts))
			return -EINVAL;

		t = timespec_to_ktime(ts);
		if (cmd == FUTEX_WAIT)
			t = ktime_add_safe(ktime_get(), t);
		tp = &t;
	}
	/*
	 * requeue parameter in 'utime' if cmd == FUTEX_*_REQUEUE_*.
	 * number of waiters to wake in 'utime' if cmd == FUTEX_WAKE_OP.
	 */
	if (cmd == FUTEX_REQUEUE || cmd == FUTEX_CMP_REQUEUE ||
	    cmd == FUTEX_CMP_REQUEUE_PI || cmd == FUTEX_WAKE_OP)
		val2 = (u32) (unsigned long) utime;

	return do_futex(uaddr, op, val, tp, uaddr2, val2, val3);
}

static int __init futex_init(void)
{
	u32 curval;
	int i;

	/*
	 * This will fail and we want it. Some arch implementations do
	 * runtime detection of the futex_atomic_cmpxchg_inatomic()
	 * functionality. We want to know that before we call in any
	 * of the complex code paths. Also we want to prevent
	 * registration of robust lists in that case. NULL is
	 * guaranteed to fault and we get -EFAULT on functional
	 * implementation, the non functional ones will return
	 * -ENOSYS.
	 */
	curval = cmpxchg_futex_value_locked(NULL, 0, 0);
	if (curval == -EFAULT)
		futex_cmpxchg_enabled = 1;

	for (i = 0; i < ARRAY_SIZE(futex_queues); i++) {
		plist_head_init(&futex_queues[i].chain, &futex_queues[i].lock);
		spin_lock_init(&futex_queues[i].lock);
	}

	return 0;
}
__initcall(futex_init);
