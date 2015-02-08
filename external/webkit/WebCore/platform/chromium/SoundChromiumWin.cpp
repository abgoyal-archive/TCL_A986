

#include "config.h"
#include "Sound.h"

#include <windows.h>

namespace WebCore {

void systemBeep()
{
    MessageBeep(static_cast<UINT>(-1));
}

} // namespace WebCore
