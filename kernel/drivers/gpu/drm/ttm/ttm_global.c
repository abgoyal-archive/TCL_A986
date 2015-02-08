

#include "ttm/ttm_module.h"
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/module.h>

struct ttm_global_item {
	struct mutex mutex;
	void *object;
	int refcount;
};

static struct ttm_global_item glob[TTM_GLOBAL_NUM];

void ttm_global_init(void)
{
	int i;

	for (i = 0; i < TTM_GLOBAL_NUM; ++i) {
		struct ttm_global_item *item = &glob[i];
		mutex_init(&item->mutex);
		item->object = NULL;
		item->refcount = 0;
	}
}

void ttm_global_release(void)
{
	int i;
	for (i = 0; i < TTM_GLOBAL_NUM; ++i) {
		struct ttm_global_item *item = &glob[i];
		BUG_ON(item->object != NULL);
		BUG_ON(item->refcount != 0);
	}
}

int ttm_global_item_ref(struct ttm_global_reference *ref)
{
	int ret;
	struct ttm_global_item *item = &glob[ref->global_type];
	void *object;

	mutex_lock(&item->mutex);
	if (item->refcount == 0) {
		item->object = kzalloc(ref->size, GFP_KERNEL);
		if (unlikely(item->object == NULL)) {
			ret = -ENOMEM;
			goto out_err;
		}

		ref->object = item->object;
		ret = ref->init(ref);
		if (unlikely(ret != 0))
			goto out_err;

	}
	++item->refcount;
	ref->object = item->object;
	object = item->object;
	mutex_unlock(&item->mutex);
	return 0;
out_err:
	mutex_unlock(&item->mutex);
	item->object = NULL;
	return ret;
}
EXPORT_SYMBOL(ttm_global_item_ref);

void ttm_global_item_unref(struct ttm_global_reference *ref)
{
	struct ttm_global_item *item = &glob[ref->global_type];

	mutex_lock(&item->mutex);
	BUG_ON(item->refcount == 0);
	BUG_ON(ref->object != item->object);
	if (--item->refcount == 0) {
		ref->release(ref);
		item->object = NULL;
	}
	mutex_unlock(&item->mutex);
}
EXPORT_SYMBOL(ttm_global_item_unref);

