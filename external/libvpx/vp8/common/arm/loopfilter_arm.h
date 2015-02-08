


#ifndef LOOPFILTER_ARM_H
#define LOOPFILTER_ARM_H

#if HAVE_ARMV6
extern prototype_loopfilter_block(vp8_loop_filter_mbv_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_bv_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_mbh_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_bh_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_mbvs_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_bvs_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_mbhs_armv6);
extern prototype_loopfilter_block(vp8_loop_filter_bhs_armv6);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_lf_normal_mb_v
#define vp8_lf_normal_mb_v vp8_loop_filter_mbv_armv6

#undef  vp8_lf_normal_b_v
#define vp8_lf_normal_b_v vp8_loop_filter_bv_armv6

#undef  vp8_lf_normal_mb_h
#define vp8_lf_normal_mb_h vp8_loop_filter_mbh_armv6

#undef  vp8_lf_normal_b_h
#define vp8_lf_normal_b_h vp8_loop_filter_bh_armv6

#undef  vp8_lf_simple_mb_v
#define vp8_lf_simple_mb_v vp8_loop_filter_mbvs_armv6

#undef  vp8_lf_simple_b_v
#define vp8_lf_simple_b_v vp8_loop_filter_bvs_armv6

#undef  vp8_lf_simple_mb_h
#define vp8_lf_simple_mb_h vp8_loop_filter_mbhs_armv6

#undef  vp8_lf_simple_b_h
#define vp8_lf_simple_b_h vp8_loop_filter_bhs_armv6
#endif
#endif

#if HAVE_ARMV7
extern prototype_loopfilter_block(vp8_loop_filter_mbv_neon);
extern prototype_loopfilter_block(vp8_loop_filter_bv_neon);
extern prototype_loopfilter_block(vp8_loop_filter_mbh_neon);
extern prototype_loopfilter_block(vp8_loop_filter_bh_neon);
extern prototype_loopfilter_block(vp8_loop_filter_mbvs_neon);
extern prototype_loopfilter_block(vp8_loop_filter_bvs_neon);
extern prototype_loopfilter_block(vp8_loop_filter_mbhs_neon);
extern prototype_loopfilter_block(vp8_loop_filter_bhs_neon);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_lf_normal_mb_v
#define vp8_lf_normal_mb_v vp8_loop_filter_mbv_neon

#undef  vp8_lf_normal_b_v
#define vp8_lf_normal_b_v vp8_loop_filter_bv_neon

#undef  vp8_lf_normal_mb_h
#define vp8_lf_normal_mb_h vp8_loop_filter_mbh_neon

#undef  vp8_lf_normal_b_h
#define vp8_lf_normal_b_h vp8_loop_filter_bh_neon

#undef  vp8_lf_simple_mb_v
#define vp8_lf_simple_mb_v vp8_loop_filter_mbvs_neon

#undef  vp8_lf_simple_b_v
#define vp8_lf_simple_b_v vp8_loop_filter_bvs_neon

#undef  vp8_lf_simple_mb_h
#define vp8_lf_simple_mb_h vp8_loop_filter_mbhs_neon

#undef  vp8_lf_simple_b_h
#define vp8_lf_simple_b_h vp8_loop_filter_bhs_neon
#endif
#endif

#endif
