

#ifndef _ASM_X86_AMD_IOMMU_PROTO_H
#define _ASM_X86_AMD_IOMMU_PROTO_H

struct amd_iommu;

extern int amd_iommu_init_dma_ops(void);
extern int amd_iommu_init_passthrough(void);
extern irqreturn_t amd_iommu_int_handler(int irq, void *data);
extern void amd_iommu_flush_all_domains(void);
extern void amd_iommu_flush_all_devices(void);
extern void amd_iommu_apply_erratum_63(u16 devid);
extern void amd_iommu_reset_cmd_buffer(struct amd_iommu *iommu);
extern int amd_iommu_init_devices(void);
extern void amd_iommu_uninit_devices(void);
extern void amd_iommu_init_notifier(void);
extern void amd_iommu_init_api(void);
#ifndef CONFIG_AMD_IOMMU_STATS

static inline void amd_iommu_stats_init(void) { }

#endif /* !CONFIG_AMD_IOMMU_STATS */

#endif /* _ASM_X86_AMD_IOMMU_PROTO_H  */
