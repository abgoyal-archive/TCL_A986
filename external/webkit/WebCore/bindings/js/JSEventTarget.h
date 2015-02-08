

#ifndef JSEventTarget_h
#define JSEventTarget_h

#include <runtime/JSValue.h>

namespace JSC {
    class ExecState;
}

namespace WebCore {

    class EventTarget;
    class JSDOMGlobalObject;

    JSC::JSValue toJS(JSC::ExecState*, JSDOMGlobalObject*, EventTarget*);
    EventTarget* toEventTarget(JSC::JSValue);

} // namespace WebCore

#endif // JSEventTarget_h
