
#include <linux/highmem.h>
#include <linux/module.h>

void *kmap(struct page *page)
{
	might_sleep();
	if (!PageHighMem(page))
		return page_address(page);
	return kmap_high(page);
}

EXPORT_SYMBOL(kmap);

void kunmap(struct page *page)
{
	if (in_interrupt())
		BUG();
	if (!PageHighMem(page))
		return;
	kunmap_high(page);
}

EXPORT_SYMBOL(kunmap);

struct page *kmap_atomic_to_page(void *ptr)
{
	return virt_to_page(ptr);
}
