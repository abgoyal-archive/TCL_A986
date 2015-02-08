

#ifndef __ARCH_S390_CACHE_H
#define __ARCH_S390_CACHE_H

#define L1_CACHE_BYTES     256
#define L1_CACHE_SHIFT     8

#define __read_mostly __attribute__((__section__(".data..read_mostly")))

#endif
