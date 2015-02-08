


#ifndef DCT_X86_H
#define DCT_X86_H


#if HAVE_MMX
extern prototype_fdct(vp8_short_fdct4x4_mmx);
extern prototype_fdct(vp8_short_fdct8x4_mmx);

#if !CONFIG_RUNTIME_CPU_DETECT
#if 0
#undef  vp8_fdct_short4x4
#define vp8_fdct_short4x4 vp8_short_fdct4x4_mmx

#undef  vp8_fdct_short8x4
#define vp8_fdct_short8x4 vp8_short_fdct8x4_mmx
#endif

#endif
#endif


#if HAVE_SSE2
extern prototype_fdct(vp8_short_fdct8x4_wmt);
extern prototype_fdct(vp8_short_walsh4x4_sse2);

extern prototype_fdct(vp8_short_fdct4x4_sse2);

#if !CONFIG_RUNTIME_CPU_DETECT
#if 1
/* short SSE2 DCT currently disabled, does not match the MMX version */
#undef  vp8_fdct_short4x4
#define vp8_fdct_short4x4 vp8_short_fdct4x4_sse2

#undef  vp8_fdct_short8x4
#define vp8_fdct_short8x4 vp8_short_fdct8x4_sse2
#endif

#undef  vp8_fdct_fast4x4
#define vp8_fdct_fast4x4 vp8_short_fdct4x4_sse2

#undef  vp8_fdct_fast8x4
#define vp8_fdct_fast8x4 vp8_short_fdct8x4_sse2

#undef vp8_fdct_walsh_short4x4
#define vp8_fdct_walsh_short4x4  vp8_short_walsh4x4_sse2

#endif


#endif

#endif
