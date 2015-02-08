
#ifndef __ARCH_S390_PERCPU__
#define __ARCH_S390_PERCPU__

#define __my_cpu_offset S390_lowcore.percpu_offset

#if defined(CONFIG_SMP) && defined(__s390x__) && defined(MODULE)
#define ARCH_NEEDS_WEAK_PER_CPU
#endif

#include <asm-generic/percpu.h>

#endif /* __ARCH_S390_PERCPU__ */
