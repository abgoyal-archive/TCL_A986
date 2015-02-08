

#ifndef _ASM_X86_AMD_IOMMU_H
#define _ASM_X86_AMD_IOMMU_H

#include <linux/irqreturn.h>

#ifdef CONFIG_AMD_IOMMU

extern void amd_iommu_detect(void);

#else

static inline void amd_iommu_detect(void) { }

#endif

#endif /* _ASM_X86_AMD_IOMMU_H */
