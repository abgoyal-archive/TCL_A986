
#ifndef _ANDROID_UTILS_DEBUG_H
#define _ANDROID_UTILS_DEBUG_H

#include <stdarg.h>

#define  VERBOSE_TAG_LIST    \
    _VERBOSE_TAG(init,         "emulator initialization")  \
    _VERBOSE_TAG(console,      "control console")  \
    _VERBOSE_TAG(modem,        "emulated GSM modem")  \
    _VERBOSE_TAG(radio,        "emulated GSM AT Command channel") \
    _VERBOSE_TAG(keys,         "key bindings & presses") \
    _VERBOSE_TAG(slirp,        "internal router/firewall") \
    _VERBOSE_TAG(timezone,     "host timezone detection" ) \
    _VERBOSE_TAG(socket,       "network sockets") \
    _VERBOSE_TAG(proxy,        "network proxy support") \
    _VERBOSE_TAG(audio,        "audio sub-system") \
    _VERBOSE_TAG(audioin,      "audio input backend") \
    _VERBOSE_TAG(audioout,     "audio output backend") \
    _VERBOSE_TAG(surface,      "video surface support") \
    _VERBOSE_TAG(qemud,        "qemud multiplexer daemon") \
    _VERBOSE_TAG(gps,          "emulated GPS") \
    _VERBOSE_TAG(nand_limits,  "nand/flash read/write thresholding") \
    _VERBOSE_TAG(hw_control,   "emulated power/flashlight/led/vibrator") \
    _VERBOSE_TAG(avd_config,   "android virtual device configuration") \
    _VERBOSE_TAG(sensors,      "emulated sensors") \
    _VERBOSE_TAG(memcheck,     "memory checker") \

#define  _VERBOSE_TAG(x,y)  VERBOSE_##x,
typedef enum {
    VERBOSE_TAG_LIST
    VERBOSE_MAX  /* do not remove */
} VerboseTag;
#undef  _VERBOSE_TAG

/* defined in android_main.c */
extern unsigned long  android_verbose;

#define  VERBOSE_ENABLE(tag)    \
    android_verbose |= (1 << VERBOSE_##tag)

#define  VERBOSE_DISABLE(tag)   \
    android_verbose &= (1 << VERBOSE_##tag)

#define  VERBOSE_CHECK(tag)    \
    ((android_verbose & (1 << VERBOSE_##tag)) != 0)

#define  VERBOSE_CHECK_ANY()    \
    (android_verbose != 0)

#define  VERBOSE_PRINT(tag,...)  \
    do { if (VERBOSE_CHECK(tag)) dprint(__VA_ARGS__); } while (0)


extern void   dprint( const char*  format, ... );
extern void   dprintn( const char*  format, ... );
extern void   dprintnv( const char*  format, va_list  args );
extern void   dwarning( const char*  format, ... );
extern void   derror( const char*  format, ... );


extern void  stdio_disable( void );
extern void  stdio_enable( void );

/* */

#endif /* _ANDROID_UTILS_DEBUG_H */
