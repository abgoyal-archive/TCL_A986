

#include "config.h"
#include "JSText.h"

#include "Text.h"

using namespace JSC;

namespace WebCore {

JSValue toJSNewlyCreated(ExecState* exec, JSDOMGlobalObject* globalObject, Text* text)
{
    if (!text)
        return jsNull();
    
    return CREATE_DOM_NODE_WRAPPER(exec, globalObject, Text, text);
}

} // namespace WebCore
