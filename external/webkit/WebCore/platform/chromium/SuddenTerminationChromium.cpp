

#include "config.h"
#include "SuddenTermination.h"

#include "ChromiumBridge.h"

namespace WebCore {

void disableSuddenTermination()
{
    ChromiumBridge::suddenTerminationChanged(false);
}

void enableSuddenTermination()
{
    ChromiumBridge::suddenTerminationChanged(true);
}

} // namespace WebCore
