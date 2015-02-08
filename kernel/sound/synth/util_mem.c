

#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/util_mem.h>

MODULE_AUTHOR("Takashi Iwai");
MODULE_DESCRIPTION("Generic memory management routines for soundcard memory allocation");
MODULE_LICENSE("GPL");

#define get_memblk(p)	list_entry(p, struct snd_util_memblk, list)

struct snd_util_memhdr *
snd_util_memhdr_new(int memsize)
{
	struct snd_util_memhdr *hdr;

	hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
	if (hdr == NULL)
		return NULL;
	hdr->size = memsize;
	mutex_init(&hdr->block_mutex);
	INIT_LIST_HEAD(&hdr->block);

	return hdr;
}

void snd_util_memhdr_free(struct snd_util_memhdr *hdr)
{
	struct list_head *p;

	if (!hdr)
		return;
	/* release all blocks */
	while ((p = hdr->block.next) != &hdr->block) {
		list_del(p);
		kfree(get_memblk(p));
	}
	kfree(hdr);
}

struct snd_util_memblk *
__snd_util_mem_alloc(struct snd_util_memhdr *hdr, int size)
{
	struct snd_util_memblk *blk;
	unsigned int units, prev_offset;
	struct list_head *p;

	if (snd_BUG_ON(!hdr || size <= 0))
		return NULL;

	/* word alignment */
	units = size;
	if (units & 1)
		units++;
	if (units > hdr->size)
		return NULL;

	/* look for empty block */
	prev_offset = 0;
	list_for_each(p, &hdr->block) {
		blk = get_memblk(p);
		if (blk->offset - prev_offset >= units)
			goto __found;
		prev_offset = blk->offset + blk->size;
	}
	if (hdr->size - prev_offset < units)
		return NULL;

__found:
	return __snd_util_memblk_new(hdr, units, p->prev);
}


struct snd_util_memblk *
__snd_util_memblk_new(struct snd_util_memhdr *hdr, unsigned int units,
		      struct list_head *prev)
{
	struct snd_util_memblk *blk;

	blk = kmalloc(sizeof(struct snd_util_memblk) + hdr->block_extra_size,
		      GFP_KERNEL);
	if (blk == NULL)
		return NULL;

	if (prev == &hdr->block)
		blk->offset = 0;
	else {
		struct snd_util_memblk *p = get_memblk(prev);
		blk->offset = p->offset + p->size;
	}
	blk->size = units;
	list_add(&blk->list, prev);
	hdr->nblocks++;
	hdr->used += units;
	return blk;
}


struct snd_util_memblk *
snd_util_mem_alloc(struct snd_util_memhdr *hdr, int size)
{
	struct snd_util_memblk *blk;
	mutex_lock(&hdr->block_mutex);
	blk = __snd_util_mem_alloc(hdr, size);
	mutex_unlock(&hdr->block_mutex);
	return blk;
}


void
__snd_util_mem_free(struct snd_util_memhdr *hdr, struct snd_util_memblk *blk)
{
	list_del(&blk->list);
	hdr->nblocks--;
	hdr->used -= blk->size;
	kfree(blk);
}

int snd_util_mem_free(struct snd_util_memhdr *hdr, struct snd_util_memblk *blk)
{
	if (snd_BUG_ON(!hdr || !blk))
		return -EINVAL;

	mutex_lock(&hdr->block_mutex);
	__snd_util_mem_free(hdr, blk);
	mutex_unlock(&hdr->block_mutex);
	return 0;
}

int snd_util_mem_avail(struct snd_util_memhdr *hdr)
{
	unsigned int size;
	mutex_lock(&hdr->block_mutex);
	size = hdr->size - hdr->used;
	mutex_unlock(&hdr->block_mutex);
	return size;
}


EXPORT_SYMBOL(snd_util_memhdr_new);
EXPORT_SYMBOL(snd_util_memhdr_free);
EXPORT_SYMBOL(snd_util_mem_alloc);
EXPORT_SYMBOL(snd_util_mem_free);
EXPORT_SYMBOL(snd_util_mem_avail);
EXPORT_SYMBOL(__snd_util_mem_alloc);
EXPORT_SYMBOL(__snd_util_mem_free);
EXPORT_SYMBOL(__snd_util_memblk_new);


static int __init alsa_util_mem_init(void)
{
	return 0;
}

static void __exit alsa_util_mem_exit(void)
{
}

module_init(alsa_util_mem_init)
module_exit(alsa_util_mem_exit)
