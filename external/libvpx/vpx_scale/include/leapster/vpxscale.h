


#ifndef VPXSCALE_H
#define VPXSCALE_H


// fwg 2004-10-14
typedef void (*vpxvertical_band_4_5_scale_lf)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
typedef void (*vpxlast_vertical_band_4_5_scale_lf)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
typedef void (*vpxvertical_band_3_5_scale_lf)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
typedef void (*vpxlast_vertical_band_3_5_scale_lf)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
typedef void (*vpxhorizontal_line_1_2_scale_lf)(const unsigned char *source, unsigned int source_width, unsigned char *dest, unsigned int dest_width);
typedef void (*vpxhorizontal_line_3_5_scale_lf)(const unsigned char *source, unsigned int source_width, unsigned char *dest, unsigned int dest_width);
typedef void (*vpxhorizontal_line_4_5_scale_lf)(const unsigned char *source, unsigned int source_width, unsigned char *dest, unsigned int dest_width);
typedef void (*vpxvertical_band_1_2_scale_lf)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);
typedef void (*vpxlast_vertical_band_1_2_scale_lf)(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width);


typedef struct vpxglobal_scalling_ptrs_t
{
    vpxvertical_band_4_5_scale_lf        vpxvertical_band_4_5_scale_t;
    vpxlast_vertical_band_4_5_scale_lf    vpxlast_vertical_band_4_5_scale_t;
    vpxvertical_band_3_5_scale_lf        vpxvertical_band_3_5_scale_t;
    vpxlast_vertical_band_3_5_scale_lf    vpxlast_vertical_band_3_5_scale_t;
    vpxhorizontal_line_1_2_scale_lf      vpxhorizontal_line_1_2_scale_t;
    vpxhorizontal_line_3_5_scale_lf      vpxhorizontal_line_3_5_scale_t;
    vpxhorizontal_line_4_5_scale_lf      vpxhorizontal_line_4_5_scale_t;
    vpxvertical_band_1_2_scale_lf        vpxvertical_band_1_2_scale_t;
    vpxlast_vertical_band_1_2_scale_lf    vpxlast_vertical_band_1_2_scale_t;
} vpxglobal_scalling_ptrs;

extern struct vpxglobal_scalling_ptrs_t *g_scaling_ptrs;


#endif
