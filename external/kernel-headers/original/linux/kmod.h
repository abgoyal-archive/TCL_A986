#ifndef __LINUX_KMOD_H__
#define __LINUX_KMOD_H__


#include <linux/stddef.h>
#include <linux/errno.h>
#include <linux/compiler.h>

#define KMOD_PATH_LEN 256

#ifdef CONFIG_KMOD
extern int request_module(const char * name, ...) __attribute__ ((format (printf, 1, 2)));
#else
static inline int request_module(const char * name, ...) { return -ENOSYS; }
#endif

#define try_then_request_module(x, mod...) ((x) ?: (request_module(mod), (x)))

struct key;
extern int call_usermodehelper_keys(char *path, char *argv[], char *envp[],
				    struct key *session_keyring, int wait);

static inline int
call_usermodehelper(char *path, char **argv, char **envp, int wait)
{
	return call_usermodehelper_keys(path, argv, envp, NULL, wait);
}

extern void usermodehelper_init(void);

#endif /* __LINUX_KMOD_H__ */
