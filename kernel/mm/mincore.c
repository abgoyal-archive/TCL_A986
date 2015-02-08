

#include <linux/pagemap.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/syscalls.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/hugetlb.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>

static void mincore_hugetlb_page_range(struct vm_area_struct *vma,
				unsigned long addr, unsigned long end,
				unsigned char *vec)
{
#ifdef CONFIG_HUGETLB_PAGE
	struct hstate *h;

	h = hstate_vma(vma);
	while (1) {
		unsigned char present;
		pte_t *ptep;
		/*
		 * Huge pages are always in RAM for now, but
		 * theoretically it needs to be checked.
		 */
		ptep = huge_pte_offset(current->mm,
				       addr & huge_page_mask(h));
		present = ptep && !huge_pte_none(huge_ptep_get(ptep));
		while (1) {
			*vec = present;
			vec++;
			addr += PAGE_SIZE;
			if (addr == end)
				return;
			/* check hugepage border */
			if (!(addr & ~huge_page_mask(h)))
				break;
		}
	}
#else
	BUG();
#endif
}

static unsigned char mincore_page(struct address_space *mapping, pgoff_t pgoff)
{
	unsigned char present = 0;
	struct page *page;

	/*
	 * When tmpfs swaps out a page from a file, any process mapping that
	 * file will not get a swp_entry_t in its pte, but rather it is like
	 * any other file mapping (ie. marked !present and faulted in with
	 * tmpfs's .fault). So swapped out tmpfs mappings are tested here.
	 *
	 * However when tmpfs moves the page from pagecache and into swapcache,
	 * it is still in core, but the find_get_page below won't find it.
	 * No big deal, but make a note of it.
	 */
	page = find_get_page(mapping, pgoff);
	if (page) {
		present = PageUptodate(page);
		page_cache_release(page);
	}

	return present;
}

static void mincore_unmapped_range(struct vm_area_struct *vma,
				unsigned long addr, unsigned long end,
				unsigned char *vec)
{
	unsigned long nr = (end - addr) >> PAGE_SHIFT;
	int i;

	if (vma->vm_file) {
		pgoff_t pgoff;

		pgoff = linear_page_index(vma, addr);
		for (i = 0; i < nr; i++, pgoff++)
			vec[i] = mincore_page(vma->vm_file->f_mapping, pgoff);
	} else {
		for (i = 0; i < nr; i++)
			vec[i] = 0;
	}
}

static void mincore_pte_range(struct vm_area_struct *vma, pmd_t *pmd,
			unsigned long addr, unsigned long end,
			unsigned char *vec)
{
	unsigned long next;
	spinlock_t *ptl;
	pte_t *ptep;

	ptep = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
	do {
		pte_t pte = *ptep;
		pgoff_t pgoff;

		next = addr + PAGE_SIZE;
		if (pte_none(pte))
			mincore_unmapped_range(vma, addr, next, vec);
		else if (pte_present(pte))
			*vec = 1;
		else if (pte_file(pte)) {
			pgoff = pte_to_pgoff(pte);
			*vec = mincore_page(vma->vm_file->f_mapping, pgoff);
		} else { /* pte is a swap entry */
			swp_entry_t entry = pte_to_swp_entry(pte);

			if (is_migration_entry(entry)) {
				/* migration entries are always uptodate */
				*vec = 1;
			} else {
#ifdef CONFIG_SWAP
				pgoff = entry.val;
				*vec = mincore_page(&swapper_space, pgoff);
#else
				WARN_ON(1);
				*vec = 1;
#endif
			}
		}
		vec++;
	} while (ptep++, addr = next, addr != end);
	pte_unmap_unlock(ptep - 1, ptl);
}

static void mincore_pmd_range(struct vm_area_struct *vma, pud_t *pud,
			unsigned long addr, unsigned long end,
			unsigned char *vec)
{
	unsigned long next;
	pmd_t *pmd;

	pmd = pmd_offset(pud, addr);
	do {
		next = pmd_addr_end(addr, end);
		if (pmd_none_or_clear_bad(pmd))
			mincore_unmapped_range(vma, addr, next, vec);
		else
			mincore_pte_range(vma, pmd, addr, next, vec);
		vec += (next - addr) >> PAGE_SHIFT;
	} while (pmd++, addr = next, addr != end);
}

static void mincore_pud_range(struct vm_area_struct *vma, pgd_t *pgd,
			unsigned long addr, unsigned long end,
			unsigned char *vec)
{
	unsigned long next;
	pud_t *pud;

	pud = pud_offset(pgd, addr);
	do {
		next = pud_addr_end(addr, end);
		if (pud_none_or_clear_bad(pud))
			mincore_unmapped_range(vma, addr, next, vec);
		else
			mincore_pmd_range(vma, pud, addr, next, vec);
		vec += (next - addr) >> PAGE_SHIFT;
	} while (pud++, addr = next, addr != end);
}

static void mincore_page_range(struct vm_area_struct *vma,
			unsigned long addr, unsigned long end,
			unsigned char *vec)
{
	unsigned long next;
	pgd_t *pgd;

	pgd = pgd_offset(vma->vm_mm, addr);
	do {
		next = pgd_addr_end(addr, end);
		if (pgd_none_or_clear_bad(pgd))
			mincore_unmapped_range(vma, addr, next, vec);
		else
			mincore_pud_range(vma, pgd, addr, next, vec);
		vec += (next - addr) >> PAGE_SHIFT;
	} while (pgd++, addr = next, addr != end);
}

static long do_mincore(unsigned long addr, unsigned long pages, unsigned char *vec)
{
	struct vm_area_struct *vma;
	unsigned long end;

	vma = find_vma(current->mm, addr);
	if (!vma || addr < vma->vm_start)
		return -ENOMEM;

	end = min(vma->vm_end, addr + (pages << PAGE_SHIFT));

	if (is_vm_hugetlb_page(vma)) {
		mincore_hugetlb_page_range(vma, addr, end, vec);
		return (end - addr) >> PAGE_SHIFT;
	}

	end = pmd_addr_end(addr, end);

	if (is_vm_hugetlb_page(vma))
		mincore_hugetlb_page_range(vma, addr, end, vec);
	else
		mincore_page_range(vma, addr, end, vec);

	return (end - addr) >> PAGE_SHIFT;
}

SYSCALL_DEFINE3(mincore, unsigned long, start, size_t, len,
		unsigned char __user *, vec)
{
	long retval;
	unsigned long pages;
	unsigned char *tmp;

	/* Check the start address: needs to be page-aligned.. */
 	if (start & ~PAGE_CACHE_MASK)
		return -EINVAL;

	/* ..and we need to be passed a valid user-space range */
	if (!access_ok(VERIFY_READ, (void __user *) start, len))
		return -ENOMEM;

	/* This also avoids any overflows on PAGE_CACHE_ALIGN */
	pages = len >> PAGE_SHIFT;
	pages += (len & ~PAGE_MASK) != 0;

	if (!access_ok(VERIFY_WRITE, vec, pages))
		return -EFAULT;

	tmp = (void *) __get_free_page(GFP_USER);
	if (!tmp)
		return -EAGAIN;

	retval = 0;
	while (pages) {
		/*
		 * Do at most PAGE_SIZE entries per iteration, due to
		 * the temporary buffer size.
		 */
		down_read(&current->mm->mmap_sem);
		retval = do_mincore(start, min(pages, PAGE_SIZE), tmp);
		up_read(&current->mm->mmap_sem);

		if (retval <= 0)
			break;
		if (copy_to_user(vec, tmp, retval)) {
			retval = -EFAULT;
			break;
		}
		pages -= retval;
		vec += retval;
		start += retval << PAGE_SHIFT;
		retval = 0;
	}
	free_page((unsigned long) tmp);
	return retval;
}
