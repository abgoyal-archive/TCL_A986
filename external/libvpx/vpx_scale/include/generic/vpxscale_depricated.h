


#ifndef VPXSCALE_H
#define VPXSCALE_H

extern void (*vp8_vertical_band_4_5_scale)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
extern void (*vp8_last_vertical_band_4_5_scale)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
extern void (*vp8_vertical_band_3_5_scale)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
extern void (*vp8_last_vertical_band_3_5_scale)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
extern void (*vp8_horizontal_line_1_2_scale)(const unsigned char *source, unsigned int source_width, unsigned char *dest, unsigned int dest_width);
extern void (*vp8_horizontal_line_3_5_scale)(const unsigned char *source, unsigned int source_width, unsigned char *dest, unsigned int dest_width);
extern void (*vp8_horizontal_line_4_5_scale)(const unsigned char *source, unsigned int source_width, unsigned char *dest, unsigned int dest_width);
extern void (*vp8_vertical_band_1_2_scale)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
extern void (*vp8_last_vertical_band_1_2_scale)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);

extern void  dmachine_specific_config(int mmx_enabled, int xmm_enabled, int wmt_enabled);

#endif
