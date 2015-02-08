

#ifndef JavaScriptProfileNode_h
#define JavaScriptProfileNode_h

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)

#include <runtime/JSValue.h>
#include <JavaScriptCore/JSBase.h>

namespace JSC {
    class ExecState;
    class ProfileNode;
}

namespace WebCore {

    JSClassRef ProfileNodeClass();
    JSC::JSValue toJS(JSC::ExecState*, JSC::ProfileNode*);

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)

#endif
