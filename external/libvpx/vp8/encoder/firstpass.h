


#if !defined __INC_FIRSTPASS_H
#define      __INC_FIRSTPASS_H

extern void vp8_init_first_pass(VP8_COMP *cpi);
extern void vp8_first_pass(VP8_COMP *cpi);
extern void vp8_end_first_pass(VP8_COMP *cpi);

extern void vp8_init_second_pass(VP8_COMP *cpi);
extern void vp8_second_pass(VP8_COMP *cpi);
extern void vp8_end_second_pass(VP8_COMP *cpi);

extern size_t vp8_firstpass_stats_sz(unsigned int mb_count);
#endif
