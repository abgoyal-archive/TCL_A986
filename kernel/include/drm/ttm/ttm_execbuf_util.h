

#ifndef _TTM_EXECBUF_UTIL_H_
#define _TTM_EXECBUF_UTIL_H_

#include "ttm/ttm_bo_api.h"
#include <linux/list.h>


struct ttm_validate_buffer {
	struct list_head head;
	struct ttm_buffer_object *bo;
	void *new_sync_obj_arg;
	bool reserved;
};


extern void ttm_eu_backoff_reservation(struct list_head *list);


extern int ttm_eu_reserve_buffers(struct list_head *list, uint32_t val_seq);


extern void ttm_eu_fence_buffer_objects(struct list_head *list, void *sync_obj);

#endif
