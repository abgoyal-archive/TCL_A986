

#include "config.h"
#include "JSProfilerPrivate.h"

#include "APICast.h"
#include "OpaqueJSString.h"
#include "Profiler.h"

using namespace JSC;

void JSStartProfiling(JSContextRef ctx, JSStringRef title)
{
    Profiler::profiler()->startProfiling(toJS(ctx), title->ustring());
}

void JSEndProfiling(JSContextRef ctx, JSStringRef title)
{
    ExecState* exec = toJS(ctx);
    Profiler* profiler = Profiler::profiler();
    profiler->stopProfiling(exec, title->ustring());
}

