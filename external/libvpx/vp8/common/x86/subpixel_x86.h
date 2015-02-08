


#ifndef SUBPIXEL_X86_H
#define SUBPIXEL_X86_H


#if HAVE_MMX
extern prototype_subpixel_predict(vp8_sixtap_predict16x16_mmx);
extern prototype_subpixel_predict(vp8_sixtap_predict8x8_mmx);
extern prototype_subpixel_predict(vp8_sixtap_predict8x4_mmx);
extern prototype_subpixel_predict(vp8_sixtap_predict4x4_mmx);
extern prototype_subpixel_predict(vp8_bilinear_predict16x16_mmx);
extern prototype_subpixel_predict(vp8_bilinear_predict8x8_mmx);
extern prototype_subpixel_predict(vp8_bilinear_predict8x4_mmx);
extern prototype_subpixel_predict(vp8_bilinear_predict4x4_mmx);


#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_subpix_sixtap16x16
#define vp8_subpix_sixtap16x16 vp8_sixtap_predict16x16_mmx

#undef  vp8_subpix_sixtap8x8
#define vp8_subpix_sixtap8x8 vp8_sixtap_predict8x8_mmx

#undef  vp8_subpix_sixtap8x4
#define vp8_subpix_sixtap8x4 vp8_sixtap_predict8x4_mmx

#undef  vp8_subpix_sixtap4x4
#define vp8_subpix_sixtap4x4 vp8_sixtap_predict4x4_mmx

#undef  vp8_subpix_bilinear16x16
#define vp8_subpix_bilinear16x16 vp8_bilinear_predict16x16_mmx

#undef  vp8_subpix_bilinear8x8
#define vp8_subpix_bilinear8x8 vp8_bilinear_predict8x8_mmx

#undef  vp8_subpix_bilinear8x4
#define vp8_subpix_bilinear8x4 vp8_bilinear_predict8x4_mmx

#undef  vp8_subpix_bilinear4x4
#define vp8_subpix_bilinear4x4 vp8_bilinear_predict4x4_mmx

#endif
#endif


#if HAVE_SSE2
extern prototype_subpixel_predict(vp8_sixtap_predict16x16_sse2);
extern prototype_subpixel_predict(vp8_sixtap_predict8x8_sse2);
extern prototype_subpixel_predict(vp8_sixtap_predict8x4_sse2);
extern prototype_subpixel_predict(vp8_bilinear_predict16x16_sse2);
extern prototype_subpixel_predict(vp8_bilinear_predict8x8_sse2);


#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_subpix_sixtap16x16
#define vp8_subpix_sixtap16x16 vp8_sixtap_predict16x16_sse2

#undef  vp8_subpix_sixtap8x8
#define vp8_subpix_sixtap8x8 vp8_sixtap_predict8x8_sse2

#undef  vp8_subpix_sixtap8x4
#define vp8_subpix_sixtap8x4 vp8_sixtap_predict8x4_sse2

#undef  vp8_subpix_bilinear16x16
#define vp8_subpix_bilinear16x16 vp8_bilinear_predict16x16_sse2

#undef  vp8_subpix_bilinear8x8
#define vp8_subpix_bilinear8x8 vp8_bilinear_predict8x8_sse2

#endif
#endif

#if HAVE_SSSE3
extern prototype_subpixel_predict(vp8_sixtap_predict16x16_ssse3);
extern prototype_subpixel_predict(vp8_sixtap_predict8x8_ssse3);
extern prototype_subpixel_predict(vp8_sixtap_predict8x4_ssse3);
extern prototype_subpixel_predict(vp8_sixtap_predict4x4_ssse3);
extern prototype_subpixel_predict(vp8_bilinear_predict16x16_ssse3);
extern prototype_subpixel_predict(vp8_bilinear_predict8x8_ssse3);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_subpix_sixtap16x16
#define vp8_subpix_sixtap16x16 vp8_sixtap_predict16x16_ssse3

#undef  vp8_subpix_sixtap8x8
#define vp8_subpix_sixtap8x8 vp8_sixtap_predict8x8_ssse3

#undef  vp8_subpix_sixtap8x4
#define vp8_subpix_sixtap8x4 vp8_sixtap_predict8x4_ssse3

#undef  vp8_subpix_sixtap4x4
#define vp8_subpix_sixtap4x4 vp8_sixtap_predict4x4_ssse3


#undef  vp8_subpix_bilinear16x16
#define vp8_subpix_bilinear16x16 vp8_bilinear_predict16x16_ssse3

#undef  vp8_subpix_bilinear8x8
#define vp8_subpix_bilinear8x8 vp8_bilinear_predict8x8_ssse3

#endif
#endif



#endif
