

#include <mach/mt6573_typedefs.h>
#include <mt6573_kpd.h>

#if KPD_DRV_CTRL_BACKLIGHT
void kpd_enable_backlight(void)
{
	mt6326_kpled_dim_duty_Full();
	mt6326_kpled_Enable();
}

void kpd_disable_backlight(void)
{
	mt6326_kpled_dim_duty_0();
	mt6326_kpled_Disable();
}
#endif

/* for META tool */
void kpd_set_backlight(bool onoff, void *val1, void *val2)
{
}
