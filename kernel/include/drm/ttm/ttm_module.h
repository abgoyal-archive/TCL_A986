

#ifndef _TTM_MODULE_H_
#define _TTM_MODULE_H_

#include <linux/kernel.h>
struct kobject;

#define TTM_PFX "[TTM] "

enum ttm_global_types {
	TTM_GLOBAL_TTM_MEM = 0,
	TTM_GLOBAL_TTM_BO,
	TTM_GLOBAL_TTM_OBJECT,
	TTM_GLOBAL_NUM
};

struct ttm_global_reference {
	enum ttm_global_types global_type;
	size_t size;
	void *object;
	int (*init) (struct ttm_global_reference *);
	void (*release) (struct ttm_global_reference *);
};

extern void ttm_global_init(void);
extern void ttm_global_release(void);
extern int ttm_global_item_ref(struct ttm_global_reference *ref);
extern void ttm_global_item_unref(struct ttm_global_reference *ref);
extern struct kobject *ttm_get_kobj(void);

#endif /* _TTM_MODULE_H_ */
