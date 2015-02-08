



#include "vpx_scale/vpxscale.h"
#include "cpuidlib.h"

extern void register_generic_scalers(void);
extern void register_mmxscalers(void);

void
vp8_scale_machine_specific_config(void)
{
    // If MMX supported then set to use MMX versions of functions else
    // use original 'C' versions.
    int mmx_enabled;
    int xmm_enabled;
    int wmt_enabled;

    vpx_get_processor_flags(&mmx_enabled, &xmm_enabled, &wmt_enabled);

    if (mmx_enabled || xmm_enabled || wmt_enabled)
    {
        register_mmxscalers();
    }
    else
    {
        vp8_horizontal_line_1_2_scale        = vp8cx_horizontal_line_1_2_scale_c;
        vp8_vertical_band_1_2_scale          = vp8cx_vertical_band_1_2_scale_c;
        vp8_last_vertical_band_1_2_scale      = vp8cx_last_vertical_band_1_2_scale_c;
        vp8_horizontal_line_3_5_scale        = vp8cx_horizontal_line_3_5_scale_c;
        vp8_vertical_band_3_5_scale          = vp8cx_vertical_band_3_5_scale_c;
        vp8_last_vertical_band_3_5_scale      = vp8cx_last_vertical_band_3_5_scale_c;
        vp8_horizontal_line_3_4_scale        = vp8cx_horizontal_line_3_4_scale_c;
        vp8_vertical_band_3_4_scale          = vp8cx_vertical_band_3_4_scale_c;
        vp8_last_vertical_band_3_4_scale      = vp8cx_last_vertical_band_3_4_scale_c;
        vp8_horizontal_line_2_3_scale        = vp8cx_horizontal_line_2_3_scale_c;
        vp8_vertical_band_2_3_scale          = vp8cx_vertical_band_2_3_scale_c;
        vp8_last_vertical_band_2_3_scale      = vp8cx_last_vertical_band_2_3_scale_c;
        vp8_horizontal_line_4_5_scale        = vp8cx_horizontal_line_4_5_scale_c;
        vp8_vertical_band_4_5_scale          = vp8cx_vertical_band_4_5_scale_c;
        vp8_last_vertical_band_4_5_scale      = vp8cx_last_vertical_band_4_5_scale_c;


        vp8_vertical_band_5_4_scale           = vp8cx_vertical_band_5_4_scale_c;
        vp8_vertical_band_5_3_scale           = vp8cx_vertical_band_5_3_scale_c;
        vp8_vertical_band_2_1_scale           = vp8cx_vertical_band_2_1_scale_c;
        vp8_vertical_band_2_1_scale_i         = vp8cx_vertical_band_2_1_scale_i_c;
        vp8_horizontal_line_2_1_scale         = vp8cx_horizontal_line_2_1_scale_c;
        vp8_horizontal_line_5_3_scale         = vp8cx_horizontal_line_5_3_scale_c;
        vp8_horizontal_line_5_4_scale         = vp8cx_horizontal_line_5_4_scale_c;

    }
}
