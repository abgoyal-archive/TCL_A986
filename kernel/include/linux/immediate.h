
#ifndef _LINUX_IMMEDIATE_H
#define _LINUX_IMMEDIATE_H


#ifdef USE_IMMEDIATE

#include <asm/immediate.h>

#define imv_set(name, i)						\
	do {								\
		name##__imv = (i);					\
		core_imv_update();					\
		module_imv_update();					\
	} while (0)

extern void core_imv_update(void);
extern void imv_update_range(const struct __imv *begin,
	const struct __imv *end);
extern void imv_unref_core_init(void);
extern void imv_unref(struct __imv *begin, struct __imv *end, void *start,
		unsigned long size);

#else


#define imv_read(name)			_imv_read(name)

#define imv_set(name, i)		(name##__imv = (i))

static inline void core_imv_update(void) { }
static inline void imv_unref_core_init(void) { }

#endif

#define DECLARE_IMV(type, name) extern __typeof__(type) name##__imv
#define DEFINE_IMV(type, name)  __typeof__(type) name##__imv

#define EXPORT_IMV_SYMBOL(name) EXPORT_SYMBOL(name##__imv)
#define EXPORT_IMV_SYMBOL_GPL(name) EXPORT_SYMBOL_GPL(name##__imv)

#define _imv_read(name)		(name##__imv)

#endif
