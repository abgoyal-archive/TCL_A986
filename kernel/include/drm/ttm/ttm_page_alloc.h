
#ifndef TTM_PAGE_ALLOC
#define TTM_PAGE_ALLOC

#include "ttm_bo_driver.h"
#include "ttm_memory.h"

int ttm_get_pages(struct list_head *pages,
		  int flags,
		  enum ttm_caching_state cstate,
		  unsigned count);
void ttm_put_pages(struct list_head *pages,
		   unsigned page_count,
		   int flags,
		   enum ttm_caching_state cstate);
int ttm_page_alloc_init(struct ttm_mem_global *glob, unsigned max_pages);
void ttm_page_alloc_fini(void);

extern int ttm_page_alloc_debugfs(struct seq_file *m, void *data);
#endif
