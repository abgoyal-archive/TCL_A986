#include "defs.h"

#if defined(LINUX)

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include <sys/utsname.h>
#ifndef HAVE_ANDROID_OS
#include <sys/user.h>
#endif
#include <sys/syscall.h>
#include <signal.h>

/* Bits of module.flags.  */

#define MOD_UNINITIALIZED	0
#define MOD_RUNNING		1
#define MOD_DELETED		2
#define MOD_AUTOCLEAN		4
#define MOD_VISITED  		8
#define MOD_USED_ONCE		16
#define MOD_JUST_FREED		32
#define MOD_INITIALIZING	64

/* Values for query_module's which.  */

#define QM_MODULES	1
#define QM_DEPS		2
#define QM_REFS		3
#define QM_SYMBOLS	4
#define QM_INFO		5

struct module_symbol
{
	unsigned long value;
	const char *name;
};

struct module_info
{
	unsigned long addr;
	unsigned long size;
	unsigned long flags;
	long usecount;
};

static const struct xlat which[] = {
	{ 0,		"0"		},
	{ QM_MODULES,	"QM_MODULES"	},
	{ QM_DEPS,	"QM_DEPS"	},
	{ QM_REFS,	"QM_REFS"	},
	{ QM_SYMBOLS,	"QM_SYMBOLS"	},
	{ QM_INFO,	"QM_INFO"	},
	{ 0,		NULL		},
};

static const struct xlat modflags[] = {
	{ MOD_UNINITIALIZED,	"MOD_UNINITIALIZED"	},
	{ MOD_RUNNING,		"MOD_RUNNING"		},
	{ MOD_DELETED,		"MOD_DELETED"		},
	{ MOD_AUTOCLEAN,	"MOD_AUTOCLEAN"		},
	{ MOD_VISITED,		"MOD_VISITED"		},
	{ MOD_USED_ONCE,	"MOD_USED_ONCE"		},
	{ MOD_JUST_FREED,	"MOD_JUST_FREED"	},
	{ 0,			NULL			},
};

int
sys_query_module(tcp)
struct tcb *tcp;
{

	if (exiting(tcp)) {
		printstr(tcp, tcp->u_arg[0], -1);
		tprintf(", ");
		printxval(which, tcp->u_arg[1], "QM_???");
		tprintf(", ");
		if (!verbose(tcp)) {
			tprintf("%#lx, %lu, %#lx", tcp->u_arg[2], tcp->u_arg[3], tcp->u_arg[4]);
		} else if (tcp->u_rval!=0) {
			size_t	ret;
			umove(tcp, tcp->u_arg[4], &ret);
			tprintf("%#lx, %lu, %Zu", tcp->u_arg[2], tcp->u_arg[3], ret);
		} else if (tcp->u_arg[1]==QM_INFO) {
			struct module_info	mi;
			size_t			ret;
			umove(tcp, tcp->u_arg[2], &mi);
			tprintf("{address=%#lx, size=%lu, flags=", mi.addr, mi.size);
			printflags(modflags, mi.flags, "MOD_???");
			tprintf(", usecount=%lu}", mi.usecount);
			umove(tcp, tcp->u_arg[4], &ret);
			tprintf(", %Zu", ret);
		} else if ((tcp->u_arg[1]==QM_MODULES) ||
			       	(tcp->u_arg[1]==QM_DEPS) ||
			       	(tcp->u_arg[1]==QM_REFS)) {
			size_t	ret;

			umove(tcp, tcp->u_arg[4], &ret);
			tprintf("{");
			if (!abbrev(tcp)) {
				char*	data	= (char*)malloc(tcp->u_arg[3]);
				char*	mod	= data;
				size_t	idx;

				if (data==NULL) {
					fprintf(stderr, "out of memory\n");
					tprintf(" /* %Zu entries */ ", ret);
				} else {
					umoven(tcp, tcp->u_arg[2], tcp->u_arg[3], data);
					for (idx=0; idx<ret; idx++) {
						if (idx!=0)
							tprintf(",");
						tprintf("%s", mod);
						mod+=strlen(mod)+1;
					}
					free(data);
				}
			} else
				tprintf(" /* %Zu entries */ ", ret);
			tprintf("}, %Zu", ret);
		} else if (tcp->u_arg[1]==QM_SYMBOLS) {
			size_t	ret;
			umove(tcp, tcp->u_arg[4], &ret);
			tprintf("{");
			if (!abbrev(tcp)) {
				char*			data	= (char *)malloc(tcp->u_arg[3]);
				struct module_symbol*	sym	= (struct module_symbol*)data;
				size_t			idx;

				if (data==NULL) {
					fprintf(stderr, "out of memory\n");
					tprintf(" /* %Zu entries */ ", ret);
				} else {
					umoven(tcp, tcp->u_arg[2], tcp->u_arg[3], data);
					for (idx=0; idx<ret; idx++) {
						tprintf("{name=%s, value=%lu} ", data+(long)sym->name, sym->value);
						sym++;
					}
					free(data);
				}
			} else
				tprintf(" /* %Zu entries */ ", ret);
			tprintf("}, %Zd", ret);
		} else {
			printstr(tcp, tcp->u_arg[2], tcp->u_arg[3]);
			tprintf(", %#lx", tcp->u_arg[4]);
		}
	}
	return 0;
}

int
sys_create_module(tcp)
struct tcb *tcp;
{
	if (entering(tcp)) {
		printpath(tcp, tcp->u_arg[0]);
		tprintf(", %lu", tcp->u_arg[1]);
	}
	return RVAL_HEX;
}

int
sys_init_module(tcp)
struct tcb *tcp;
{
	if (entering(tcp)) {
		printpath(tcp, tcp->u_arg[0]);
		tprintf(", %#lx", tcp->u_arg[1]);
	}
	return 0;
}
#endif /* LINUX */
