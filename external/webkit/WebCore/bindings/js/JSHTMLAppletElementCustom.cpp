

#include "config.h"
#include "JSHTMLAppletElementCustom.h"

#include "HTMLAppletElement.h"
#include "JSPluginElementFunctions.h"

namespace WebCore {

using namespace JSC;

bool JSHTMLAppletElement::getOwnPropertySlotDelegate(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return runtimeObjectCustomGetOwnPropertySlot(exec, propertyName, slot, this);
}

bool JSHTMLAppletElement::getOwnPropertyDescriptorDelegate(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return runtimeObjectCustomGetOwnPropertyDescriptor(exec, propertyName, descriptor, this);
}

bool JSHTMLAppletElement::putDelegate(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    return runtimeObjectCustomPut(exec, propertyName, value, impl(), slot);
}

CallType JSHTMLAppletElement::getCallData(CallData& callData)
{
    return runtimeObjectGetCallData(impl(), callData);
}

bool JSHTMLAppletElement::canGetItemsForName(ExecState*, HTMLAppletElement*, const Identifier& propertyName)
{
    return propertyName == "__apple_runtime_object";
}

JSValue JSHTMLAppletElement::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    return runtimeObjectGetter(exec, propertyName, slot);
}

} // namespace WebCore
