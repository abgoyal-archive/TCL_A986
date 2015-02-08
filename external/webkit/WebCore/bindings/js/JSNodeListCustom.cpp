

#include "config.h"
#include "JSNodeList.h"

#include "AtomicString.h"
#include "JSNode.h"
#include "Node.h"
#include "NodeList.h"

using namespace JSC;

namespace WebCore {

// Need to support call so that list(0) works.
static JSValue JSC_HOST_CALL callNodeList(ExecState* exec, JSObject* function, JSValue, const ArgList& args)
{
    bool ok;
    unsigned index = args.at(0).toString(exec).toUInt32(&ok);
    if (!ok)
        return jsUndefined();
    return toJS(exec, static_cast<JSNodeList*>(function)->impl()->item(index));
}

CallType JSNodeList::getCallData(CallData& callData)
{
    callData.native.function = callNodeList;
    return CallTypeHost;
}

bool JSNodeList::canGetItemsForName(ExecState*, NodeList* impl, const Identifier& propertyName)
{
    return impl->itemWithName(propertyName);
}

JSValue JSNodeList::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSNodeList* thisObj = static_cast<JSNodeList*>(asObject(slot.slotBase()));
    return toJS(exec, thisObj->impl()->itemWithName(propertyName));
}

} // namespace WebCore
