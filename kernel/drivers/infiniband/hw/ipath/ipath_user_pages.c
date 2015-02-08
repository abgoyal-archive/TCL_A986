

#include <linux/mm.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include "ipath_kernel.h"

static void __ipath_release_user_pages(struct page **p, size_t num_pages,
				   int dirty)
{
	size_t i;

	for (i = 0; i < num_pages; i++) {
		ipath_cdbg(MM, "%lu/%lu put_page %p\n", (unsigned long) i,
			   (unsigned long) num_pages, p[i]);
		if (dirty)
			set_page_dirty_lock(p[i]);
		put_page(p[i]);
	}
}

/* call with current->mm->mmap_sem held */
static int __get_user_pages(unsigned long start_page, size_t num_pages,
			struct page **p, struct vm_area_struct **vma)
{
	unsigned long lock_limit;
	size_t got;
	int ret;

	lock_limit = rlimit(RLIMIT_MEMLOCK) >> PAGE_SHIFT;

	if (num_pages > lock_limit) {
		ret = -ENOMEM;
		goto bail;
	}

	ipath_cdbg(VERBOSE, "pin %lx pages from vaddr %lx\n",
		   (unsigned long) num_pages, start_page);

	for (got = 0; got < num_pages; got += ret) {
		ret = get_user_pages(current, current->mm,
				     start_page + got * PAGE_SIZE,
				     num_pages - got, 1, 1,
				     p + got, vma);
		if (ret < 0)
			goto bail_release;
	}

	current->mm->locked_vm += num_pages;

	ret = 0;
	goto bail;

bail_release:
	__ipath_release_user_pages(p, got, 0);
bail:
	return ret;
}

dma_addr_t ipath_map_page(struct pci_dev *hwdev, struct page *page,
	unsigned long offset, size_t size, int direction)
{
	dma_addr_t phys;

	phys = pci_map_page(hwdev, page, offset, size, direction);

	if (phys == 0) {
		pci_unmap_page(hwdev, phys, size, direction);
		phys = pci_map_page(hwdev, page, offset, size, direction);
		/*
		 * FIXME: If we get 0 again, we should keep this page,
		 * map another, then free the 0 page.
		 */
	}

	return phys;
}

dma_addr_t ipath_map_single(struct pci_dev *hwdev, void *ptr, size_t size,
	int direction)
{
	dma_addr_t phys;

	phys = pci_map_single(hwdev, ptr, size, direction);

	if (phys == 0) {
		pci_unmap_single(hwdev, phys, size, direction);
		phys = pci_map_single(hwdev, ptr, size, direction);
		/*
		 * FIXME: If we get 0 again, we should keep this page,
		 * map another, then free the 0 page.
		 */
	}

	return phys;
}

int ipath_get_user_pages(unsigned long start_page, size_t num_pages,
			 struct page **p)
{
	int ret;

	down_write(&current->mm->mmap_sem);

	ret = __get_user_pages(start_page, num_pages, p, NULL);

	up_write(&current->mm->mmap_sem);

	return ret;
}

void ipath_release_user_pages(struct page **p, size_t num_pages)
{
	down_write(&current->mm->mmap_sem);

	__ipath_release_user_pages(p, num_pages, 1);

	current->mm->locked_vm -= num_pages;

	up_write(&current->mm->mmap_sem);
}

struct ipath_user_pages_work {
	struct work_struct work;
	struct mm_struct *mm;
	unsigned long num_pages;
};

static void user_pages_account(struct work_struct *_work)
{
	struct ipath_user_pages_work *work =
		container_of(_work, struct ipath_user_pages_work, work);

	down_write(&work->mm->mmap_sem);
	work->mm->locked_vm -= work->num_pages;
	up_write(&work->mm->mmap_sem);
	mmput(work->mm);
	kfree(work);
}

void ipath_release_user_pages_on_close(struct page **p, size_t num_pages)
{
	struct ipath_user_pages_work *work;
	struct mm_struct *mm;

	__ipath_release_user_pages(p, num_pages, 1);

	mm = get_task_mm(current);
	if (!mm)
		return;

	work = kmalloc(sizeof(*work), GFP_KERNEL);
	if (!work)
		goto bail_mm;

	INIT_WORK(&work->work, user_pages_account);
	work->mm = mm;
	work->num_pages = num_pages;

	schedule_work(&work->work);
	return;

bail_mm:
	mmput(mm);
	return;
}
