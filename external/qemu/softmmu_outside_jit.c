


#include "qemu-common.h"
#include "cpu.h"

#define OUTSIDE_JIT
#define MMUSUFFIX       _outside_jit
#define GETPC()         NULL
#define env             cpu_single_env
#define ACCESS_TYPE     1
#define CPU_MMU_INDEX   (cpu_mmu_index(env))

#define SHIFT 0
#include "softmmu_template.h"

#define SHIFT 1
#include "softmmu_template.h"

#define SHIFT 2
#include "softmmu_template.h"

#define SHIFT 3
#include "softmmu_template.h"

#undef CPU_MMU_INDEX
#undef ACCESS_TYPE
#undef env
#undef MMUSUFFIX
