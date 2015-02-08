
#include <cust_mag.h>
#include <mach/mt6573_pll.h>
static struct mag_hw cust_mag_hw = {
    .i2c_num = 0,
    .direction = 3,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
};
struct mag_hw* get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}
