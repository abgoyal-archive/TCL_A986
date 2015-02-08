


#ifndef LOOPFILTER_X86_H
#define LOOPFILTER_X86_H


#if HAVE_MMX
extern prototype_loopfilter_block(vp8_loop_filter_mbv_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_bv_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_mbh_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_bh_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_mbvs_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_bvs_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_mbhs_mmx);
extern prototype_loopfilter_block(vp8_loop_filter_bhs_mmx);


#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_lf_normal_mb_v
#define vp8_lf_normal_mb_v vp8_loop_filter_mbv_mmx

#undef  vp8_lf_normal_b_v
#define vp8_lf_normal_b_v vp8_loop_filter_bv_mmx

#undef  vp8_lf_normal_mb_h
#define vp8_lf_normal_mb_h vp8_loop_filter_mbh_mmx

#undef  vp8_lf_normal_b_h
#define vp8_lf_normal_b_h vp8_loop_filter_bh_mmx

#undef  vp8_lf_simple_mb_v
#define vp8_lf_simple_mb_v vp8_loop_filter_mbvs_mmx

#undef  vp8_lf_simple_b_v
#define vp8_lf_simple_b_v vp8_loop_filter_bvs_mmx

#undef  vp8_lf_simple_mb_h
#define vp8_lf_simple_mb_h vp8_loop_filter_mbhs_mmx

#undef  vp8_lf_simple_b_h
#define vp8_lf_simple_b_h vp8_loop_filter_bhs_mmx
#endif
#endif


#if HAVE_SSE2
extern prototype_loopfilter_block(vp8_loop_filter_mbv_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_bv_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_mbh_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_bh_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_mbvs_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_bvs_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_mbhs_sse2);
extern prototype_loopfilter_block(vp8_loop_filter_bhs_sse2);


#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_lf_normal_mb_v
#define vp8_lf_normal_mb_v vp8_loop_filter_mbv_sse2

#undef  vp8_lf_normal_b_v
#define vp8_lf_normal_b_v vp8_loop_filter_bv_sse2

#undef  vp8_lf_normal_mb_h
#define vp8_lf_normal_mb_h vp8_loop_filter_mbh_sse2

#undef  vp8_lf_normal_b_h
#define vp8_lf_normal_b_h vp8_loop_filter_bh_sse2

#undef  vp8_lf_simple_mb_v
#define vp8_lf_simple_mb_v vp8_loop_filter_mbvs_sse2

#undef  vp8_lf_simple_b_v
#define vp8_lf_simple_b_v vp8_loop_filter_bvs_sse2

#undef  vp8_lf_simple_mb_h
#define vp8_lf_simple_mb_h vp8_loop_filter_mbhs_sse2

#undef  vp8_lf_simple_b_h
#define vp8_lf_simple_b_h vp8_loop_filter_bhs_sse2
#endif
#endif


#endif
