


#ifndef QEMU_ANDROID_UI_CORE_PROTOCOL_H
#define QEMU_ANDROID_UI_CORE_PROTOCOL_H

/* Gets LCD density property from the core properties. */
int android_core_get_hw_lcd_density(void);

typedef void  (*AndroidHwLightBrightnessCallback)( void*       opaque,
                                               const char* light,
                                               int         brightness );

/* Registers a UI callback to be called when brightness is changed by the core. */
void android_core_set_brightness_change_callback(AndroidHwLightBrightnessCallback callback,
                                                 void* opaque);

/* Returns base port assigned for the emulated system. */
int android_core_get_base_port(void);

#endif  // QEMU_ANDROID_UI_CORE_PROTOCOL_H
