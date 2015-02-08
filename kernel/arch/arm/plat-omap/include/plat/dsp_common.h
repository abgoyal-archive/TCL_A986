

#ifndef ASM_ARCH_DSP_COMMON_H
#define ASM_ARCH_DSP_COMMON_H

#if defined(CONFIG_ARCH_OMAP1) && defined(CONFIG_OMAP_MMU_FWK)
extern void omap_dsp_request_mpui(void);
extern void omap_dsp_release_mpui(void);
extern int omap_dsp_request_mem(void);
extern int omap_dsp_release_mem(void);
#else
static inline int omap_dsp_request_mem(void)
{
	return 0;
}
#define omap_dsp_release_mem()	do {} while (0)
#endif

#endif /* ASM_ARCH_DSP_COMMON_H */
