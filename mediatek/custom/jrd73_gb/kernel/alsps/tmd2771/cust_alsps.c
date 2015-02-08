
#include <cust_alsps.h>
#include <mach/mt6573_pll.h>
static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/    
    .i2c_addr   = {0x72},
    .als_level  = { 2, 15,  50,   100,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
    .als_value  = {3, 20, 40,  80, 120, 280,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    .ps_threshold = 700,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
