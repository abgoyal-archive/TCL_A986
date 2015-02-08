

#ifndef JavaScriptProfile_h
#define JavaScriptProfile_h

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)

#include <runtime/JSValue.h>

namespace JSC {
    class ExecState;
    class Profile;
}

namespace WebCore {

    JSC::JSValue toJS(JSC::ExecState*, JSC::Profile*);

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)

#endif
