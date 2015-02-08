
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/spinlock.h>
#include <linux/page-flags.h>
#include <asm/bug.h>
#include "ctree.h"
#include "extent_io.h"
#include "locking.h"

static inline void spin_nested(struct extent_buffer *eb)
{
	spin_lock(&eb->lock);
}

void btrfs_set_lock_blocking(struct extent_buffer *eb)
{
	if (!test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags)) {
		set_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags);
		spin_unlock(&eb->lock);
	}
	/* exit with the spin lock released and the bit set */
}

void btrfs_clear_lock_blocking(struct extent_buffer *eb)
{
	if (test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags)) {
		spin_nested(eb);
		clear_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags);
		smp_mb__after_clear_bit();
	}
	/* exit with the spin lock held */
}

static int btrfs_spin_on_block(struct extent_buffer *eb)
{
	int i;

	for (i = 0; i < 512; i++) {
		if (!test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
			return 1;
		if (need_resched())
			break;
		cpu_relax();
	}
	return 0;
}

int btrfs_try_spin_lock(struct extent_buffer *eb)
{
	int i;

	if (btrfs_spin_on_block(eb)) {
		spin_nested(eb);
		if (!test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
			return 1;
		spin_unlock(&eb->lock);
	}
	/* spin for a bit on the BLOCKING flag */
	for (i = 0; i < 2; i++) {
		cpu_relax();
		if (!btrfs_spin_on_block(eb))
			break;

		spin_nested(eb);
		if (!test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
			return 1;
		spin_unlock(&eb->lock);
	}
	return 0;
}

static int btrfs_wake_function(wait_queue_t *wait, unsigned mode,
			       int sync, void *key)
{
	autoremove_wake_function(wait, mode, sync, key);
	return 1;
}

int btrfs_tree_lock(struct extent_buffer *eb)
{
	DEFINE_WAIT(wait);
	wait.func = btrfs_wake_function;

	if (!btrfs_spin_on_block(eb))
		goto sleep;

	while(1) {
		spin_nested(eb);

		/* nobody is blocking, exit with the spinlock held */
		if (!test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
			return 0;

		/*
		 * we have the spinlock, but the real owner is blocking.
		 * wait for them
		 */
		spin_unlock(&eb->lock);

		/*
		 * spin for a bit, and if the blocking flag goes away,
		 * loop around
		 */
		cpu_relax();
		if (btrfs_spin_on_block(eb))
			continue;
sleep:
		prepare_to_wait_exclusive(&eb->lock_wq, &wait,
					  TASK_UNINTERRUPTIBLE);

		if (test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
			schedule();

		finish_wait(&eb->lock_wq, &wait);
	}
	return 0;
}

int btrfs_try_tree_lock(struct extent_buffer *eb)
{
	if (spin_trylock(&eb->lock)) {
		if (test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags)) {
			/*
			 * we've got the spinlock, but the real owner is
			 * blocking.  Drop the spinlock and return failure
			 */
			spin_unlock(&eb->lock);
			return 0;
		}
		return 1;
	}
	/* someone else has the spinlock giveup */
	return 0;
}

int btrfs_tree_unlock(struct extent_buffer *eb)
{
	/*
	 * if we were a blocking owner, we don't have the spinlock held
	 * just clear the bit and look for waiters
	 */
	if (test_and_clear_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
		smp_mb__after_clear_bit();
	else
		spin_unlock(&eb->lock);

	if (waitqueue_active(&eb->lock_wq))
		wake_up(&eb->lock_wq);
	return 0;
}

void btrfs_assert_tree_locked(struct extent_buffer *eb)
{
	if (!test_bit(EXTENT_BUFFER_BLOCKING, &eb->bflags))
		assert_spin_locked(&eb->lock);
}
