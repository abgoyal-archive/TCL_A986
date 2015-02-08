
#ifndef __LINUX_KMOD_H__
#define __LINUX_KMOD_H__


#include <linux/gfp.h>
#include <linux/stddef.h>
#include <linux/errno.h>
#include <linux/compiler.h>
#include <linux/workqueue.h>

#define KMOD_PATH_LEN 256

#ifdef CONFIG_MODULES
extern char modprobe_path[]; /* for sysctl */
extern int __request_module(bool wait, const char *name, ...) \
	__attribute__((format(printf, 2, 3)));
#define request_module(mod...) __request_module(true, mod)
#define request_module_nowait(mod...) __request_module(false, mod)
#define try_then_request_module(x, mod...) \
	((x) ?: (__request_module(true, mod), (x)))
#else
static inline int request_module(const char *name, ...) { return -ENOSYS; }
static inline int request_module_nowait(const char *name, ...) { return -ENOSYS; }
#define try_then_request_module(x, mod...) (x)
#endif


struct key;
struct file;

enum umh_wait {
	UMH_NO_WAIT = -1,	/* don't wait at all */
	UMH_WAIT_EXEC = 0,	/* wait for the exec, but not the process */
	UMH_WAIT_PROC = 1,	/* wait for the process to complete */
};

struct subprocess_info {
	struct work_struct work;
	struct completion *complete;
	char *path;
	char **argv;
	char **envp;
	enum umh_wait wait;
	int retval;
	int (*init)(struct subprocess_info *info);
	void (*cleanup)(struct subprocess_info *info);
	void *data;
};

/* Allocate a subprocess_info structure */
struct subprocess_info *call_usermodehelper_setup(char *path, char **argv,
						  char **envp, gfp_t gfp_mask);

/* Set various pieces of state into the subprocess_info structure */
void call_usermodehelper_setfns(struct subprocess_info *info,
		    int (*init)(struct subprocess_info *info),
		    void (*cleanup)(struct subprocess_info *info),
		    void *data);

/* Actually execute the sub-process */
int call_usermodehelper_exec(struct subprocess_info *info, enum umh_wait wait);

void call_usermodehelper_freeinfo(struct subprocess_info *info);

static inline int
call_usermodehelper_fns(char *path, char **argv, char **envp,
			enum umh_wait wait,
			int (*init)(struct subprocess_info *info),
			void (*cleanup)(struct subprocess_info *), void *data)
{
	struct subprocess_info *info;
	gfp_t gfp_mask = (wait == UMH_NO_WAIT) ? GFP_ATOMIC : GFP_KERNEL;

	info = call_usermodehelper_setup(path, argv, envp, gfp_mask);

	if (info == NULL)
		return -ENOMEM;

	call_usermodehelper_setfns(info, init, cleanup, data);

	return call_usermodehelper_exec(info, wait);
}

static inline int
call_usermodehelper(char *path, char **argv, char **envp, enum umh_wait wait)
{
	return call_usermodehelper_fns(path, argv, envp, wait,
				       NULL, NULL, NULL);
}

extern void usermodehelper_init(void);

extern int usermodehelper_disable(void);
extern void usermodehelper_enable(void);

#endif /* __LINUX_KMOD_H__ */
