

#include "config.h"

#if ENABLE(JAVASCRIPT_DEBUGGER)

#include "ScriptProfiler.h"

#include <profiler/Profiler.h>

namespace WebCore {

void ScriptProfiler::start(ScriptState* state, const String& title)
{
    JSC::Profiler::profiler()->startProfiling(state, title);
}

PassRefPtr<ScriptProfile> ScriptProfiler::stop(ScriptState* state, const String& title)
{
    return JSC::Profiler::profiler()->stopProfiling(state, title);
}

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)
