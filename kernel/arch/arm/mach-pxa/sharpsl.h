

#include <mach/sharpsl_pm.h>

#define READ_GPIO_BIT(x)    (GPLR(x) & GPIO_bit(x))

/* MAX1111 Channel Definitions */
#define MAX1111_BATT_VOLT   4u
#define MAX1111_BATT_TEMP   2u
#define MAX1111_ACIN_VOLT   6u

extern struct battery_thresh sharpsl_battery_levels_acin[];
extern struct battery_thresh sharpsl_battery_levels_noac[];
int sharpsl_pm_pxa_read_max1111(int channel);


