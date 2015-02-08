


#include "vpx_scale/vpxscale.h"


void vp8_scale_machine_specific_config()
{
    vp8_horizontal_line_1_2_scale        = horizontal_line_1_2_scale_armv4;
    vp8_vertical_band_1_2_scale          = vertical_band_1_2_scale_armv4;
    vp8_last_vertical_band_1_2_scale      = vp8cx_last_vertical_band_1_2_scale_c;
    vp8_horizontal_line_3_5_scale        = horizontal_line_3_5_scale_armv4;
    vp8_vertical_band_3_5_scale          = vertical_band_3_5_scale_armv4;
    vp8_last_vertical_band_3_5_scale      = vp8cx_last_vertical_band_3_5_scale_c;
    vp8_horizontal_line_3_4_scale        = horizontal_line_3_4_scale_armv4;
    vp8_vertical_band_3_4_scale          = vertical_band_3_4_scale_armv4;
    vp8_last_vertical_band_3_4_scale      = vp8cx_last_vertical_band_3_4_scale_c;
    vp8_horizontal_line_2_3_scale        = horizontal_line_2_3_scale_armv4;
    vp8_vertical_band_2_3_scale          = vertical_band_2_3_scale_armv4;
    vp8_last_vertical_band_2_3_scale      = vp8cx_last_vertical_band_2_3_scale_c;
    vp8_horizontal_line_4_5_scale        = horizontal_line_4_5_scale_armv4;
    vp8_vertical_band_4_5_scale          = vertical_band_4_5_scale_armv4;
    vp8_last_vertical_band_4_5_scale      = vp8cx_last_vertical_band_4_5_scale_c;


    vp8_vertical_band_5_4_scale           = vp8cx_vertical_band_5_4_scale_c;
    vp8_vertical_band_5_3_scale           = vp8cx_vertical_band_5_3_scale_c;
    vp8_vertical_band_2_1_scale           = vp8cx_vertical_band_2_1_scale_c;
    vp8_vertical_band_2_1_scale_i         = vp8cx_vertical_band_2_1_scale_i_c;
    vp8_horizontal_line_2_1_scale         = vp8cx_horizontal_line_2_1_scale_c;
    vp8_horizontal_line_5_3_scale         = vp8cx_horizontal_line_5_3_scale_c;
    vp8_horizontal_line_5_4_scale         = vp8cx_horizontal_line_5_4_scale_c;
}
