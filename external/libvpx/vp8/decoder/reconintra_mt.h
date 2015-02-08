


#ifndef __INC_RECONINTRA_MT_H
#define __INC_RECONINTRA_MT_H

/* reconintra functions used in multi-threaded decoder */
#if CONFIG_MULTITHREAD
extern void vp8mt_build_intra_predictors_mby(VP8D_COMP *pbi, MACROBLOCKD *x, int mb_row, int mb_col);
extern void vp8mt_build_intra_predictors_mby_s(VP8D_COMP *pbi, MACROBLOCKD *x, int mb_row, int mb_col);
extern void vp8mt_build_intra_predictors_mbuv(VP8D_COMP *pbi, MACROBLOCKD *x, int mb_row, int mb_col);
extern void vp8mt_build_intra_predictors_mbuv_s(VP8D_COMP *pbi, MACROBLOCKD *x, int mb_row, int mb_col);

extern void vp8mt_predict_intra4x4(VP8D_COMP *pbi, MACROBLOCKD *x, int b_mode, unsigned char *predictor, int mb_row, int mb_col, int num);
extern void vp8mt_intra_prediction_down_copy(VP8D_COMP *pbi, MACROBLOCKD *x, int mb_row, int mb_col);
#endif

#endif
