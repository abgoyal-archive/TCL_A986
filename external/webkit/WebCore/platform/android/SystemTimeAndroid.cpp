

#include "config.h"
#include "SystemTime.h"

namespace WebCore {

float userIdleTime()
{
    // return an arbitrarily high userIdleTime so that releasing pages from the
    // page cache isn't postponed.
    return 1000.0F;
}

}  // namespace WebCore
