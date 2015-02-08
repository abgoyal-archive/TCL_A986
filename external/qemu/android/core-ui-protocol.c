


#include "android/globals.h"
#include "android/core-ui-protocol.h"

extern void  android_emulator_set_window_scale( double, int );

void
android_ui_set_window_scale(double scale, int is_dpi)
{
#if !defined(CONFIG_STANDALONE_CORE)
    android_emulator_set_window_scale(scale, is_dpi);
#endif
}
