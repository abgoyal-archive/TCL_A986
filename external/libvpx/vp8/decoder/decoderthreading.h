





#ifndef _DECODER_THREADING_H
#define _DECODER_THREADING_H

#if CONFIG_MULTITHREAD
extern void vp8mt_decode_mb_rows(VP8D_COMP *pbi, MACROBLOCKD *xd);
extern void vp8_decoder_remove_threads(VP8D_COMP *pbi);
extern void vp8_decoder_create_threads(VP8D_COMP *pbi);
extern int vp8mt_alloc_temp_buffers(VP8D_COMP *pbi, int width, int prev_mb_rows);
extern void vp8mt_de_alloc_temp_buffers(VP8D_COMP *pbi, int mb_rows);
#endif

#endif
