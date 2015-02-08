

#ifndef ScriptProfile_h
#define ScriptProfile_h

#if ENABLE(JAVASCRIPT_DEBUGGER)
#include <profiler/Profile.h>

namespace WebCore {

typedef JSC::Profile ScriptProfile;

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)

#endif // ScriptProfile_h
