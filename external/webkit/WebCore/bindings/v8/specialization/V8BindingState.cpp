

#include "config.h"
#include "V8BindingState.h"

#include "V8Proxy.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

State<V8Binding>* State<V8Binding>::Only()
{
    DEFINE_STATIC_LOCAL(State, globalV8BindingState, ());
    return &globalV8BindingState;
}

DOMWindow* State<V8Binding>::getActiveWindow()
{
    v8::Local<v8::Context> activeContext = v8::Context::GetCalling();
    if (activeContext.IsEmpty()) {
        // There is a single activation record on the stack, so that must
        // be the activeContext.
        activeContext = v8::Context::GetCurrent();
    }
    return V8Proxy::retrieveWindow(activeContext);
}

void State<V8Binding>::immediatelyReportUnsafeAccessTo(Frame* target)
{
    V8Proxy::reportUnsafeAccessTo(target, V8Proxy::ReportNow);
}

} // namespace WebCore
