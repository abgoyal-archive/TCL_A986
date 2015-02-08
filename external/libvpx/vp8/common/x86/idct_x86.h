


#ifndef IDCT_X86_H
#define IDCT_X86_H


#if HAVE_MMX
extern prototype_idct(vp8_short_idct4x4llm_1_mmx);
extern prototype_idct(vp8_short_idct4x4llm_mmx);
extern prototype_idct_scalar_add(vp8_dc_only_idct_add_mmx);

extern prototype_second_order(vp8_short_inv_walsh4x4_mmx);
extern prototype_second_order(vp8_short_inv_walsh4x4_1_mmx);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_idct_idct1
#define vp8_idct_idct1 vp8_short_idct4x4llm_1_mmx

#undef  vp8_idct_idct16
#define vp8_idct_idct16 vp8_short_idct4x4llm_mmx

#undef  vp8_idct_idct1_scalar_add
#define vp8_idct_idct1_scalar_add vp8_dc_only_idct_add_mmx

#undef vp8_idct_iwalsh16
#define vp8_idct_iwalsh16 vp8_short_inv_walsh4x4_mmx

#undef vp8_idct_iwalsh1
#define vp8_idct_iwalsh1 vp8_short_inv_walsh4x4_1_mmx

#endif
#endif

#if HAVE_SSE2

extern prototype_second_order(vp8_short_inv_walsh4x4_sse2);

#if !CONFIG_RUNTIME_CPU_DETECT

#undef vp8_idct_iwalsh16
#define vp8_idct_iwalsh16 vp8_short_inv_walsh4x4_sse2

#endif

#endif



#endif
