

#include "config.h"

#include "ScriptProfiler.h"

namespace WebCore {

void ScriptProfiler::start(ScriptState* state, const String& title)
{
    v8::HandleScope scope;
    v8::Context::Scope contextScope(v8::Context::GetCurrent());
    v8::V8::ResumeProfiler();
}

PassRefPtr<ScriptProfile> ScriptProfiler::stop(ScriptState* state, const String& title)
{
    v8::V8::PauseProfiler();
    return 0;
}

} // namespace WebCore
