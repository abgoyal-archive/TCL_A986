

#include "config.h"
#include "PropertySlot.h"

#include "JSFunction.h"
#include "JSGlobalObject.h"

namespace JSC {

JSValue PropertySlot::functionGetter(ExecState* exec, const Identifier&, const PropertySlot& slot)
{
    // Prevent getter functions from observing execution if an exception is pending.
    if (exec->hadException())
        return exec->exception();

    CallData callData;
    CallType callType = slot.m_data.getterFunc->getCallData(callData);
    if (callType == CallTypeHost)
        return callData.native.function(exec, slot.m_data.getterFunc, slot.slotBase(), exec->emptyList());
    ASSERT(callType == CallTypeJS);
    // FIXME: Can this be done more efficiently using the callData?
    return asFunction(slot.m_data.getterFunc)->call(exec, slot.slotBase(), exec->emptyList());
}

} // namespace JSC
