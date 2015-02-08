

#include "config.h"
#include "JSTouchList.h"

#if ENABLE(TOUCH_EVENTS)

#include "TouchList.h"

using namespace JSC;

namespace WebCore {

JSValue toJSNewlyCreated(ExecState* exec, JSDOMGlobalObject* globalObject, TouchList* touchList)
{
    if (!touchList)
        return jsNull();

    return CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, TouchList, touchList);
}

} // namespace WebCore

#endif
