



#include "vpx_scale/vpxscale.h"
#include "cpuidlib.h"

extern void register_generic_scalers(void);
extern void register_mmxscalers(void);

void
vp8_scale_machine_specific_config(void)
{
    int wmt_enabled = 1;

    if (wmt_enabled)
    {
        register_mmxscalers();
    }
    else
    {
        register_generic_scalers();
    }
}
