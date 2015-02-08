

#include "config.h"
#include "Sound.h"

#include <Windows.h>

namespace WebCore {

void systemBeep() { MessageBeep(static_cast<UINT>(-1)); }

} // namespace WebCore

