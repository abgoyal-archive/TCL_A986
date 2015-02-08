

#ifndef __PLAT_OMAP_MULTI_H
#define __PLAT_OMAP_MULTI_H

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
#ifdef CONFIG_ARCH_OMAP850
# ifdef OMAP_NAME
#  undef  MULTI_OMAP1
#  define MULTI_OMAP1
# else
#  define OMAP_NAME omap850
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
#ifdef CONFIG_ARCH_OMAP2PLUS
# if (defined(OMAP_NAME) || defined(MULTI_OMAP1))
#  error "OMAP1 and OMAP2PLUS can't be selected at the same time"
# endif
#endif
#ifdef CONFIG_ARCH_OMAP2420
# ifdef OMAP_NAME
#  undef  MULTI_OMAP2
#  define MULTI_OMAP2
# else
#  define OMAP_NAME omap2420
# endif
#endif
#ifdef CONFIG_ARCH_OMAP2430
# ifdef OMAP_NAME
#  undef  MULTI_OMAP2
#  define MULTI_OMAP2
# else
#  define OMAP_NAME omap2430
# endif
#endif
#ifdef CONFIG_ARCH_OMAP3
# ifdef OMAP_NAME
#  undef  MULTI_OMAP2
#  define MULTI_OMAP2
# else
#  define OMAP_NAME omap3
# endif
#endif
#ifdef CONFIG_ARCH_OMAP4
# ifdef OMAP_NAME
#  undef  MULTI_OMAP2
#  define MULTI_OMAP2
# else
#  define OMAP_NAME omap4
# endif
#endif

#endif	/* __PLAT_OMAP_MULTI_H */
