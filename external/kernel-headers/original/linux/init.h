#ifndef _LINUX_INIT_H
#define _LINUX_INIT_H

#include <linux/compiler.h>


#define __init		__attribute__ ((__section__ (".init.text")))
#define __initdata	__attribute__ ((__section__ (".init.data")))
#define __exitdata	__attribute__ ((__section__(".exit.data")))
#define __exit_call	__attribute_used__ __attribute__ ((__section__ (".exitcall.exit")))

#ifdef MODULE
#define __exit		__attribute__ ((__section__(".exit.text")))
#else
#define __exit		__attribute_used__ __attribute__ ((__section__(".exit.text")))
#endif

/* For assembly routines */
#define __INIT		.section	".init.text","ax"
#define __FINIT		.previous
#define __INITDATA	.section	".init.data","aw"

#ifndef __ASSEMBLY__
typedef int (*initcall_t)(void);
typedef void (*exitcall_t)(void);

extern initcall_t __con_initcall_start[], __con_initcall_end[];
extern initcall_t __security_initcall_start[], __security_initcall_end[];

/* Defined in init/main.c */
extern char saved_command_line[];

/* used by init/main.c */
extern void setup_arch(char **);

#endif
  
#ifndef MODULE

#ifndef __ASSEMBLY__


#define __define_initcall(level,fn) \
	static initcall_t __initcall_##fn __attribute_used__ \
	__attribute__((__section__(".initcall" level ".init"))) = fn

#define core_initcall(fn)		__define_initcall("1",fn)
#define postcore_initcall(fn)		__define_initcall("2",fn)
#define arch_initcall(fn)		__define_initcall("3",fn)
#define subsys_initcall(fn)		__define_initcall("4",fn)
#define fs_initcall(fn)			__define_initcall("5",fn)
#define device_initcall(fn)		__define_initcall("6",fn)
#define late_initcall(fn)		__define_initcall("7",fn)

#define __initcall(fn) device_initcall(fn)

#define __exitcall(fn) \
	static exitcall_t __exitcall_##fn __exit_call = fn

#define console_initcall(fn) \
	static initcall_t __initcall_##fn \
	__attribute_used__ __attribute__((__section__(".con_initcall.init")))=fn

#define security_initcall(fn) \
	static initcall_t __initcall_##fn \
	__attribute_used__ __attribute__((__section__(".security_initcall.init"))) = fn

struct obs_kernel_param {
	const char *str;
	int (*setup_func)(char *);
	int early;
};

#define __setup_param(str, unique_id, fn, early)			\
	static char __setup_str_##unique_id[] __initdata = str;	\
	static struct obs_kernel_param __setup_##unique_id	\
		__attribute_used__				\
		__attribute__((__section__(".init.setup")))	\
		__attribute__((aligned((sizeof(long)))))	\
		= { __setup_str_##unique_id, fn, early }

#define __setup_null_param(str, unique_id)			\
	__setup_param(str, unique_id, NULL, 0)

#define __setup(str, fn)					\
	__setup_param(str, fn, fn, 0)

#define __obsolete_setup(str)					\
	__setup_null_param(str, __LINE__)

#define early_param(str, fn)					\
	__setup_param(str, fn, fn, 1)

/* Relies on saved_command_line being set */
void __init parse_early_param(void);
#endif /* __ASSEMBLY__ */

#define module_init(x)	__initcall(x);

#define module_exit(x)	__exitcall(x);

#else /* MODULE */

/* Don't use these in modules, but some people do... */
#define core_initcall(fn)		module_init(fn)
#define postcore_initcall(fn)		module_init(fn)
#define arch_initcall(fn)		module_init(fn)
#define subsys_initcall(fn)		module_init(fn)
#define fs_initcall(fn)			module_init(fn)
#define device_initcall(fn)		module_init(fn)
#define late_initcall(fn)		module_init(fn)

#define security_initcall(fn)		module_init(fn)


/* Each module must use one module_init(), or one no_module_init */
#define module_init(initfn)					\
	static inline initcall_t __inittest(void)		\
	{ return initfn; }					\
	int init_module(void) __attribute__((alias(#initfn)));

/* This is only required if you want to be unloadable. */
#define module_exit(exitfn)					\
	static inline exitcall_t __exittest(void)		\
	{ return exitfn; }					\
	void cleanup_module(void) __attribute__((alias(#exitfn)));

#define __setup_param(str, unique_id, fn)	/* nothing */
#define __setup_null_param(str, unique_id) 	/* nothing */
#define __setup(str, func) 			/* nothing */
#define __obsolete_setup(str) 			/* nothing */
#endif

/* Data marked not to be saved by software_suspend() */
#define __nosavedata __attribute__ ((__section__ (".data.nosave")))

#ifdef CONFIG_MODULES
#define __init_or_module
#define __initdata_or_module
#else
#define __init_or_module __init
#define __initdata_or_module __initdata
#endif /*CONFIG_MODULES*/

#ifdef CONFIG_HOTPLUG
#define __devinit
#define __devinitdata
#define __devexit
#define __devexitdata
#else
#define __devinit __init
#define __devinitdata __initdata
#define __devexit __exit
#define __devexitdata __exitdata
#endif

#ifdef CONFIG_HOTPLUG_CPU
#define __cpuinit
#define __cpuinitdata
#define __cpuexit
#define __cpuexitdata
#else
#define __cpuinit	__init
#define __cpuinitdata __initdata
#define __cpuexit __exit
#define __cpuexitdata	__exitdata
#endif

#if defined(CONFIG_MEMORY_HOTPLUG) || defined(CONFIG_ACPI_HOTPLUG_MEMORY) \
	|| defined(CONFIG_ACPI_HOTPLUG_MEMORY_MODULE)
#define __meminit
#define __meminitdata
#define __memexit
#define __memexitdata
#else
#define __meminit	__init
#define __meminitdata __initdata
#define __memexit __exit
#define __memexitdata	__exitdata
#endif

#if defined(MODULE) || defined(CONFIG_HOTPLUG)
#define __devexit_p(x) x
#else
#define __devexit_p(x) NULL
#endif

#ifdef MODULE
#define __exit_p(x) x
#else
#define __exit_p(x) NULL
#endif

#endif /* _LINUX_INIT_H */
