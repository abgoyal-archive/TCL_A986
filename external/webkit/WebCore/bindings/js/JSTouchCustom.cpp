

#include "config.h"
#include "JSTouch.h"

#if ENABLE(TOUCH_EVENTS)

#include "Touch.h"

using namespace JSC;

namespace WebCore {

JSValue toJSNewlyCreated(ExecState* exec, JSDOMGlobalObject* globalObject, Touch* touch)
{
    if (!touch)
        return jsNull();

    return CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, Touch, touch);
}

} // namespace WebCore

#endif
