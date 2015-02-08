
#ifndef __ASM_ARCH_OMAP_CPU_H
#define __ASM_ARCH_OMAP_CPU_H

extern unsigned int system_rev;

#define omap2_cpu_rev()		((system_rev >> 8) & 0x0f)

#undef MULTI_OMAP1
#undef MULTI_OMAP2
#undef OMAP_NAME

#ifdef CONFIG_ARCH_OMAP730
# ifdef OMAP_NAME
#  undef  MULTI_OMAP1
#  define MULTI_OMAP1
# else
#  define OMAP_NAME omap730
# endif
#endif
#ifdef CONFIG_ARCH_OMAP15XX
# ifdef OMAP_NAME
#  undef  MULTI_OMAP1
#  define MULTI_OMAP1
# else
#  define OMAP_NAME omap1510
# endif
#endif
#ifdef CONFIG_ARCH_OMAP16XX
# ifdef OMAP_NAME
#  undef  MULTI_OMAP1
#  define MULTI_OMAP1
# else
#  define OMAP_NAME omap16xx
# endif
#endif
#ifdef CONFIG_ARCH_OMAP24XX
# if (defined(OMAP_NAME) || defined(MULTI_OMAP1))
#  error "OMAP1 and OMAP2 can't be selected at the same time"
# else
#  undef  MULTI_OMAP2
#  define OMAP_NAME omap24xx
# endif
#endif

#define GET_OMAP_CLASS	(system_rev & 0xff)

#define IS_OMAP_CLASS(class, id)			\
static inline int is_omap ##class (void)		\
{							\
	return (GET_OMAP_CLASS == (id)) ? 1 : 0;	\
}

#define GET_OMAP_SUBCLASS	((system_rev >> 20) & 0x0fff)

#define IS_OMAP_SUBCLASS(subclass, id)			\
static inline int is_omap ##subclass (void)		\
{							\
	return (GET_OMAP_SUBCLASS == (id)) ? 1 : 0;	\
}

IS_OMAP_CLASS(7xx, 0x07)
IS_OMAP_CLASS(15xx, 0x15)
IS_OMAP_CLASS(16xx, 0x16)
IS_OMAP_CLASS(24xx, 0x24)

IS_OMAP_SUBCLASS(242x, 0x242)
IS_OMAP_SUBCLASS(243x, 0x243)

#define cpu_is_omap7xx()		0
#define cpu_is_omap15xx()		0
#define cpu_is_omap16xx()		0
#define cpu_is_omap24xx()		0
#define cpu_is_omap242x()		0
#define cpu_is_omap243x()		0

#if defined(MULTI_OMAP1)
# if defined(CONFIG_ARCH_OMAP730)
#  undef  cpu_is_omap7xx
#  define cpu_is_omap7xx()		is_omap7xx()
# endif
# if defined(CONFIG_ARCH_OMAP15XX)
#  undef  cpu_is_omap15xx
#  define cpu_is_omap15xx()		is_omap15xx()
# endif
# if defined(CONFIG_ARCH_OMAP16XX)
#  undef  cpu_is_omap16xx
#  define cpu_is_omap16xx()		is_omap16xx()
# endif
#else
# if defined(CONFIG_ARCH_OMAP730)
#  undef  cpu_is_omap7xx
#  define cpu_is_omap7xx()		1
# endif
# if defined(CONFIG_ARCH_OMAP15XX)
#  undef  cpu_is_omap15xx
#  define cpu_is_omap15xx()		1
# endif
# if defined(CONFIG_ARCH_OMAP16XX)
#  undef  cpu_is_omap16xx
#  define cpu_is_omap16xx()		1
# endif
# if defined(CONFIG_ARCH_OMAP24XX)
#  undef  cpu_is_omap24xx
#  undef  cpu_is_omap242x
#  undef  cpu_is_omap243x
#  define cpu_is_omap24xx()		1
#  define cpu_is_omap242x()		is_omap242x()
#  define cpu_is_omap243x()		is_omap243x()
# endif
#endif

#define GET_OMAP_TYPE	((system_rev >> 16) & 0xffff)

#define IS_OMAP_TYPE(type, id)				\
static inline int is_omap ##type (void)			\
{							\
	return (GET_OMAP_TYPE == (id)) ? 1 : 0;		\
}

IS_OMAP_TYPE(310, 0x0310)
IS_OMAP_TYPE(730, 0x0730)
IS_OMAP_TYPE(1510, 0x1510)
IS_OMAP_TYPE(1610, 0x1610)
IS_OMAP_TYPE(1611, 0x1611)
IS_OMAP_TYPE(5912, 0x1611)
IS_OMAP_TYPE(1621, 0x1621)
IS_OMAP_TYPE(1710, 0x1710)
IS_OMAP_TYPE(2420, 0x2420)
IS_OMAP_TYPE(2422, 0x2422)
IS_OMAP_TYPE(2423, 0x2423)
IS_OMAP_TYPE(2430, 0x2430)

#define cpu_is_omap310()		0
#define cpu_is_omap730()		0
#define cpu_is_omap1510()		0
#define cpu_is_omap1610()		0
#define cpu_is_omap5912()		0
#define cpu_is_omap1611()		0
#define cpu_is_omap1621()		0
#define cpu_is_omap1710()		0
#define cpu_is_omap2420()		0
#define cpu_is_omap2422()		0
#define cpu_is_omap2423()		0
#define cpu_is_omap2430()		0

#if defined(MULTI_OMAP1)
# if defined(CONFIG_ARCH_OMAP730)
#  undef  cpu_is_omap730
#  define cpu_is_omap730()		is_omap730()
# endif
#else
# if defined(CONFIG_ARCH_OMAP730)
#  undef  cpu_is_omap730
#  define cpu_is_omap730()		1
# endif
#endif

#if defined(CONFIG_ARCH_OMAP15XX)
# undef  cpu_is_omap310
# undef  cpu_is_omap1510
# define cpu_is_omap310()		is_omap310()
# define cpu_is_omap1510()		is_omap1510()
#endif

#if defined(CONFIG_ARCH_OMAP16XX)
# undef  cpu_is_omap1610
# undef  cpu_is_omap1611
# undef  cpu_is_omap5912
# undef  cpu_is_omap1621
# undef  cpu_is_omap1710
# define cpu_is_omap1610()		is_omap1610()
# define cpu_is_omap1611()		is_omap1611()
# define cpu_is_omap5912()		is_omap5912()
# define cpu_is_omap1621()		is_omap1621()
# define cpu_is_omap1710()		is_omap1710()
#endif

#if defined(CONFIG_ARCH_OMAP24XX)
# undef  cpu_is_omap2420
# undef  cpu_is_omap2422
# undef  cpu_is_omap2423
# undef  cpu_is_omap2430
# define cpu_is_omap2420()		is_omap2420()
# define cpu_is_omap2422()		is_omap2422()
# define cpu_is_omap2423()		is_omap2423()
# define cpu_is_omap2430()		is_omap2430()
#endif

/* Macros to detect if we have OMAP1 or OMAP2 */
#define cpu_class_is_omap1()	(cpu_is_omap730() || cpu_is_omap15xx() || \
				cpu_is_omap16xx())
#define cpu_class_is_omap2()	cpu_is_omap24xx()

#endif
