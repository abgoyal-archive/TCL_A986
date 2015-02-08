
#include "android/gps.h"
#include "android/utils/debug.h"
#include "qemu-char.h"

CharDriverState*   android_gps_cs;

#define  D(...)  VERBOSE_PRINT(gps,__VA_ARGS__)

void
android_gps_send_nmea( const char*  sentence )
{
    if (sentence == NULL)
        return;

    D("sending '%s'", sentence);

    if (android_gps_cs == NULL) {
        D("missing GPS channel, ignored");
        return;
    }

    qemu_chr_write( android_gps_cs, (const void*)sentence, strlen(sentence) );
    qemu_chr_write( android_gps_cs, (const void*)"\n", 1 );
}


