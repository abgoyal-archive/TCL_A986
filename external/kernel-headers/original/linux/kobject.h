
#ifndef _KOBJECT_H_
#define _KOBJECT_H_

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/kref.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <asm/atomic.h>

#define KOBJ_NAME_LEN			20
#define UEVENT_HELPER_PATH_LEN		256

/* path to the userspace helper executed on an event */
extern char uevent_helper[];

/* counter to tag the uevent, read only except for the kobject core */
extern u64 uevent_seqnum;

/* the actions here must match the proper string in lib/kobject_uevent.c */
typedef int __bitwise kobject_action_t;
enum kobject_action {
	KOBJ_ADD	= (__force kobject_action_t) 0x01,	/* exclusive to core */
	KOBJ_REMOVE	= (__force kobject_action_t) 0x02,	/* exclusive to core */
	KOBJ_CHANGE	= (__force kobject_action_t) 0x03,	/* device state change */
	KOBJ_MOUNT	= (__force kobject_action_t) 0x04,	/* mount event for block devices (broken) */
	KOBJ_UMOUNT	= (__force kobject_action_t) 0x05,	/* umount event for block devices (broken) */
	KOBJ_OFFLINE	= (__force kobject_action_t) 0x06,	/* device offline */
	KOBJ_ONLINE	= (__force kobject_action_t) 0x07,	/* device online */
};

struct kobject {
	const char		* k_name;
	char			name[KOBJ_NAME_LEN];
	struct kref		kref;
	struct list_head	entry;
	struct kobject		* parent;
	struct kset		* kset;
	struct kobj_type	* ktype;
	struct dentry		* dentry;
	wait_queue_head_t	poll;
};

extern int kobject_set_name(struct kobject *, const char *, ...)
	__attribute__((format(printf,2,3)));

static inline const char * kobject_name(const struct kobject * kobj)
{
	return kobj->k_name;
}

extern void kobject_init(struct kobject *);
extern void kobject_cleanup(struct kobject *);

extern int kobject_add(struct kobject *);
extern void kobject_del(struct kobject *);

extern int kobject_rename(struct kobject *, const char *new_name);

extern int kobject_register(struct kobject *);
extern void kobject_unregister(struct kobject *);

extern struct kobject * kobject_get(struct kobject *);
extern void kobject_put(struct kobject *);

extern struct kobject *kobject_add_dir(struct kobject *, const char *);

extern char * kobject_get_path(struct kobject *, gfp_t);

struct kobj_type {
	void (*release)(struct kobject *);
	struct sysfs_ops	* sysfs_ops;
	struct attribute	** default_attrs;
};


struct kset_uevent_ops {
	int (*filter)(struct kset *kset, struct kobject *kobj);
	const char *(*name)(struct kset *kset, struct kobject *kobj);
	int (*uevent)(struct kset *kset, struct kobject *kobj, char **envp,
			int num_envp, char *buffer, int buffer_size);
};

struct kset {
	struct subsystem	* subsys;
	struct kobj_type	* ktype;
	struct list_head	list;
	spinlock_t		list_lock;
	struct kobject		kobj;
	struct kset_uevent_ops	* uevent_ops;
};


extern void kset_init(struct kset * k);
extern int kset_add(struct kset * k);
extern int kset_register(struct kset * k);
extern void kset_unregister(struct kset * k);

static inline struct kset * to_kset(struct kobject * kobj)
{
	return kobj ? container_of(kobj,struct kset,kobj) : NULL;
}

static inline struct kset * kset_get(struct kset * k)
{
	return k ? to_kset(kobject_get(&k->kobj)) : NULL;
}

static inline void kset_put(struct kset * k)
{
	kobject_put(&k->kobj);
}

static inline struct kobj_type * get_ktype(struct kobject * k)
{
	if (k->kset && k->kset->ktype)
		return k->kset->ktype;
	else 
		return k->ktype;
}

extern struct kobject * kset_find_obj(struct kset *, const char *);


#define set_kset_name(str)	.kset = { .kobj = { .name = str } }



struct subsystem {
	struct kset		kset;
	struct rw_semaphore	rwsem;
};

#define decl_subsys(_name,_type,_uevent_ops) \
struct subsystem _name##_subsys = { \
	.kset = { \
		.kobj = { .name = __stringify(_name) }, \
		.ktype = _type, \
		.uevent_ops =_uevent_ops, \
	} \
}
#define decl_subsys_name(_varname,_name,_type,_uevent_ops) \
struct subsystem _varname##_subsys = { \
	.kset = { \
		.kobj = { .name = __stringify(_name) }, \
		.ktype = _type, \
		.uevent_ops =_uevent_ops, \
	} \
}

/* The global /sys/kernel/ subsystem for people to chain off of */
extern struct subsystem kernel_subsys;
/* The global /sys/hypervisor/ subsystem  */
extern struct subsystem hypervisor_subsys;



#define kobj_set_kset_s(obj,subsys) \
	(obj)->kobj.kset = &(subsys).kset


#define kset_set_kset_s(obj,subsys) \
	(obj)->kset.kobj.kset = &(subsys).kset


#define subsys_set_kset(obj,_subsys) \
	(obj)->subsys.kset.kobj.kset = &(_subsys).kset

extern void subsystem_init(struct subsystem *);
extern int subsystem_register(struct subsystem *);
extern void subsystem_unregister(struct subsystem *);

static inline struct subsystem * subsys_get(struct subsystem * s)
{
	return s ? container_of(kset_get(&s->kset),struct subsystem,kset) : NULL;
}

static inline void subsys_put(struct subsystem * s)
{
	kset_put(&s->kset);
}

struct subsys_attribute {
	struct attribute attr;
	ssize_t (*show)(struct subsystem *, char *);
	ssize_t (*store)(struct subsystem *, const char *, size_t); 
};

extern int subsys_create_file(struct subsystem * , struct subsys_attribute *);

#if defined(CONFIG_HOTPLUG)
void kobject_uevent(struct kobject *kobj, enum kobject_action action);

int add_uevent_var(char **envp, int num_envp, int *cur_index,
			char *buffer, int buffer_size, int *cur_len,
			const char *format, ...)
	__attribute__((format (printf, 7, 8)));
#else
static inline void kobject_uevent(struct kobject *kobj, enum kobject_action action) { }

static inline int add_uevent_var(char **envp, int num_envp, int *cur_index,
				      char *buffer, int buffer_size, int *cur_len, 
				      const char *format, ...)
{ return 0; }
#endif

#endif /* __KERNEL__ */
#endif /* _KOBJECT_H_ */
