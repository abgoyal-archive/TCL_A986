


#include "android/android.h"
#include "android/globals.h"
#include "android/hw-control.h"
#include "android/ui-core-protocol.h"

int
android_core_get_hw_lcd_density(void)
{
    return android_hw->hw_lcd_density;
}

void
android_core_set_brightness_change_callback(AndroidHwLightBrightnessCallback callback,
                                            void* opaque)
{
    AndroidHwControlFuncs  funcs;

    funcs.light_brightness = callback;
    android_hw_control_init( opaque, &funcs );
}

int
android_core_get_base_port(void)
{
    return android_base_port;
}
