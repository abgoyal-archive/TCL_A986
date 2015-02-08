#ifdef __KERNEL__


#ifdef __STDC__
#define ____glue(name,fn)	name##fn
#else
#define ____glue(name,fn)	name/**/fn
#endif
#define __glue(name,fn)		____glue(name,fn)



#undef CPU_ABORT_HANDLER
#undef MULTI_ABORT

#if defined(CONFIG_CPU_ARM610)
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER cpu_arm6_data_abort
# endif
#endif

#if defined(CONFIG_CPU_ARM710)
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER cpu_arm7_data_abort
# endif
#endif

#ifdef CONFIG_CPU_ABRT_LV4T
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER v4t_late_abort
# endif
#endif

#ifdef CONFIG_CPU_ABRT_EV4
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER v4_early_abort
# endif
#endif

#ifdef CONFIG_CPU_ABRT_EV4T
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER v4t_early_abort
# endif
#endif

#ifdef CONFIG_CPU_ABRT_EV5TJ
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER v5tj_early_abort
# endif
#endif

#ifdef CONFIG_CPU_ABRT_EV5T
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER v5t_early_abort
# endif
#endif

#ifdef CONFIG_CPU_ABRT_EV6
# ifdef CPU_ABORT_HANDLER
#  define MULTI_ABORT 1
# else
#  define CPU_ABORT_HANDLER v6_early_abort
# endif
#endif

#ifndef CPU_ABORT_HANDLER
#error Unknown data abort handler type
#endif

#endif
