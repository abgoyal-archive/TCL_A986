

#include "config.h"
#include "SystemTime.h"

#include <float.h>

#include "NotImplemented.h"

namespace WebCore {

float userIdleTime()
{
    notImplemented();
    // return an arbitrarily high userIdleTime so that releasing pages from the page cache isn't postponed
    return FLT_MAX;
}

}

