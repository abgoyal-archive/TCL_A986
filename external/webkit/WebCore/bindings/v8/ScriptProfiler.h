

#ifndef ScriptProfiler_h
#define ScriptProfiler_h

#include "PlatformString.h"
#include "ScriptProfile.h"
#include "ScriptState.h"

#include <wtf/Noncopyable.h>

namespace WebCore {

class ScriptProfiler : public Noncopyable {
public:
    static void start(ScriptState* state, const String& title);
    static PassRefPtr<ScriptProfile> stop(ScriptState* state, const String& title);
};

} // namespace WebCore

#endif // ScriptProfiler_h
