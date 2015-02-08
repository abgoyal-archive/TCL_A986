

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/suspend.h>


struct nvs_page {
	unsigned long phys_start;
	unsigned int size;
	void *kaddr;
	void *data;
	struct list_head node;
};

static LIST_HEAD(nvs_list);

int suspend_nvs_register(unsigned long start, unsigned long size)
{
	struct nvs_page *entry, *next;

	while (size > 0) {
		unsigned int nr_bytes;

		entry = kzalloc(sizeof(struct nvs_page), GFP_KERNEL);
		if (!entry)
			goto Error;

		list_add_tail(&entry->node, &nvs_list);
		entry->phys_start = start;
		nr_bytes = PAGE_SIZE - (start & ~PAGE_MASK);
		entry->size = (size < nr_bytes) ? size : nr_bytes;

		start += entry->size;
		size -= entry->size;
	}
	return 0;

 Error:
	list_for_each_entry_safe(entry, next, &nvs_list, node) {
		list_del(&entry->node);
		kfree(entry);
	}
	return -ENOMEM;
}

void suspend_nvs_free(void)
{
	struct nvs_page *entry;

	list_for_each_entry(entry, &nvs_list, node)
		if (entry->data) {
			free_page((unsigned long)entry->data);
			entry->data = NULL;
			if (entry->kaddr) {
				iounmap(entry->kaddr);
				entry->kaddr = NULL;
			}
		}
}

int suspend_nvs_alloc(void)
{
	struct nvs_page *entry;

	list_for_each_entry(entry, &nvs_list, node) {
		entry->data = (void *)__get_free_page(GFP_KERNEL);
		if (!entry->data) {
			suspend_nvs_free();
			return -ENOMEM;
		}
	}
	return 0;
}

void suspend_nvs_save(void)
{
	struct nvs_page *entry;

	printk(KERN_INFO "PM: Saving platform NVS memory\n");

	list_for_each_entry(entry, &nvs_list, node)
		if (entry->data) {
			entry->kaddr = ioremap(entry->phys_start, entry->size);
			memcpy(entry->data, entry->kaddr, entry->size);
		}
}

void suspend_nvs_restore(void)
{
	struct nvs_page *entry;

	printk(KERN_INFO "PM: Restoring platform NVS memory\n");

	list_for_each_entry(entry, &nvs_list, node)
		if (entry->data)
			memcpy(entry->kaddr, entry->data, entry->size);
}
