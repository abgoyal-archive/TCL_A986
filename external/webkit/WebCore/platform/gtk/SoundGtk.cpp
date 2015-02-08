

#include "config.h"

#include "Sound.h"

#include <gdk/gdk.h>

namespace WebCore {

void systemBeep()
{
    gdk_beep();
}

}

// vim: ts=4 sw=4 et
