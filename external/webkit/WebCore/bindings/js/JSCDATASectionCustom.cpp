

#include "config.h"
#include "JSCDATASection.h"

#include "CDATASection.h"

using namespace JSC;

namespace WebCore {

JSValue toJSNewlyCreated(ExecState* exec, JSDOMGlobalObject* globalObject, CDATASection* section)
{
    if (!section)
        return jsNull();

    return CREATE_DOM_NODE_WRAPPER(exec, globalObject, CDATASection, section);
}

} // namespace WebCore
