



#include "vpx_scale/vpxscale.h"

extern int register_generic_scalers(void);
extern int de_register_generic_scalers(void);

int
vp8_scale_machine_specific_config()
{
    return register_generic_scalers();
}

int
scale_machine_specific_de_config()
{
    return de_register_generic_scalers();
}
