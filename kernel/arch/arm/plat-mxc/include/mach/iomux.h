
#ifndef __MACH_IOMUX_H__
#define __MACH_IOMUX_H__

/* This file will go away, please include mach/iomux-mx... directly */

#ifdef CONFIG_ARCH_MX1
#include <mach/iomux-mx1.h>
#endif
#ifdef CONFIG_ARCH_MX2
#include <mach/iomux-mx2x.h>
#ifdef CONFIG_MACH_MX21
#include <mach/iomux-mx21.h>
#endif
#ifdef CONFIG_MACH_MX27
#include <mach/iomux-mx27.h>
#endif
#endif

#endif /* __MACH_IOMUX_H__ */
