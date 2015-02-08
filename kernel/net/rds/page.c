
#include <linux/highmem.h>
#include <linux/gfp.h>

#include "rds.h"

struct rds_page_remainder {
	struct page	*r_page;
	unsigned long	r_offset;
};

DEFINE_PER_CPU_SHARED_ALIGNED(struct rds_page_remainder, rds_page_remainders);

int rds_page_copy_user(struct page *page, unsigned long offset,
		       void __user *ptr, unsigned long bytes,
		       int to_user)
{
	unsigned long ret;
	void *addr;

	if (to_user)
		rds_stats_add(s_copy_to_user, bytes);
	else
		rds_stats_add(s_copy_from_user, bytes);

	addr = kmap_atomic(page, KM_USER0);
	if (to_user)
		ret = __copy_to_user_inatomic(ptr, addr + offset, bytes);
	else
		ret = __copy_from_user_inatomic(addr + offset, ptr, bytes);
	kunmap_atomic(addr, KM_USER0);

	if (ret) {
		addr = kmap(page);
		if (to_user)
			ret = copy_to_user(ptr, addr + offset, bytes);
		else
			ret = copy_from_user(addr + offset, ptr, bytes);
		kunmap(page);
		if (ret)
			return -EFAULT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rds_page_copy_user);

int rds_page_remainder_alloc(struct scatterlist *scat, unsigned long bytes,
			     gfp_t gfp)
{
	struct rds_page_remainder *rem;
	unsigned long flags;
	struct page *page;
	int ret;

	gfp |= __GFP_HIGHMEM;

	/* jump straight to allocation if we're trying for a huge page */
	if (bytes >= PAGE_SIZE) {
		page = alloc_page(gfp);
		if (page == NULL) {
			ret = -ENOMEM;
		} else {
			sg_set_page(scat, page, PAGE_SIZE, 0);
			ret = 0;
		}
		goto out;
	}

	rem = &per_cpu(rds_page_remainders, get_cpu());
	local_irq_save(flags);

	while (1) {
		/* avoid a tiny region getting stuck by tossing it */
		if (rem->r_page && bytes > (PAGE_SIZE - rem->r_offset)) {
			rds_stats_inc(s_page_remainder_miss);
			__free_page(rem->r_page);
			rem->r_page = NULL;
		}

		/* hand out a fragment from the cached page */
		if (rem->r_page && bytes <= (PAGE_SIZE - rem->r_offset)) {
			sg_set_page(scat, rem->r_page, bytes, rem->r_offset);
			get_page(sg_page(scat));

			if (rem->r_offset != 0)
				rds_stats_inc(s_page_remainder_hit);

			rem->r_offset += bytes;
			if (rem->r_offset == PAGE_SIZE) {
				__free_page(rem->r_page);
				rem->r_page = NULL;
			}
			ret = 0;
			break;
		}

		/* alloc if there is nothing for us to use */
		local_irq_restore(flags);
		put_cpu();

		page = alloc_page(gfp);

		rem = &per_cpu(rds_page_remainders, get_cpu());
		local_irq_save(flags);

		if (page == NULL) {
			ret = -ENOMEM;
			break;
		}

		/* did someone race to fill the remainder before us? */
		if (rem->r_page) {
			__free_page(page);
			continue;
		}

		/* otherwise install our page and loop around to alloc */
		rem->r_page = page;
		rem->r_offset = 0;
	}

	local_irq_restore(flags);
	put_cpu();
out:
	rdsdebug("bytes %lu ret %d %p %u %u\n", bytes, ret,
		 ret ? NULL : sg_page(scat), ret ? 0 : scat->offset,
		 ret ? 0 : scat->length);
	return ret;
}

static int rds_page_remainder_cpu_notify(struct notifier_block *self,
					 unsigned long action, void *hcpu)
{
	struct rds_page_remainder *rem;
	long cpu = (long)hcpu;

	rem = &per_cpu(rds_page_remainders, cpu);

	rdsdebug("cpu %ld action 0x%lx\n", cpu, action);

	switch (action) {
	case CPU_DEAD:
		if (rem->r_page)
			__free_page(rem->r_page);
		rem->r_page = NULL;
		break;
	}

	return 0;
}

static struct notifier_block rds_page_remainder_nb = {
	.notifier_call = rds_page_remainder_cpu_notify,
};

void rds_page_exit(void)
{
	int i;

	for_each_possible_cpu(i)
		rds_page_remainder_cpu_notify(&rds_page_remainder_nb,
					      (unsigned long)CPU_DEAD,
					      (void *)(long)i);
}
