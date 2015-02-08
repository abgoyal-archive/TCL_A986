

#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/swap.h>

static DEFINE_SPINLOCK(swap_token_lock);
struct mm_struct *swap_token_mm;
static unsigned int global_faults;

void grab_swap_token(struct mm_struct *mm)
{
	int current_interval;

	global_faults++;

	current_interval = global_faults - mm->faultstamp;

	if (!spin_trylock(&swap_token_lock))
		return;

	/* First come first served */
	if (swap_token_mm == NULL) {
		mm->token_priority = mm->token_priority + 2;
		swap_token_mm = mm;
		goto out;
	}

	if (mm != swap_token_mm) {
		if (current_interval < mm->last_interval)
			mm->token_priority++;
		else {
			if (likely(mm->token_priority > 0))
				mm->token_priority--;
		}
		/* Check if we deserve the token */
		if (mm->token_priority > swap_token_mm->token_priority) {
			mm->token_priority += 2;
			swap_token_mm = mm;
		}
	} else {
		/* Token holder came in again! */
		mm->token_priority += 2;
	}

out:
	mm->faultstamp = global_faults;
	mm->last_interval = current_interval;
	spin_unlock(&swap_token_lock);
}

/* Called on process exit. */
void __put_swap_token(struct mm_struct *mm)
{
	spin_lock(&swap_token_lock);
	if (likely(mm == swap_token_mm))
		swap_token_mm = NULL;
	spin_unlock(&swap_token_lock);
}
