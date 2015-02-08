

#include "config.h"
#include "JSHTMLObjectElementCustom.h"

#include "HTMLObjectElement.h"
#include "JSPluginElementFunctions.h"

namespace WebCore {

using namespace JSC;

bool JSHTMLObjectElement::getOwnPropertySlotDelegate(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return runtimeObjectCustomGetOwnPropertySlot(exec, propertyName, slot, this);
}

bool JSHTMLObjectElement::getOwnPropertyDescriptorDelegate(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return runtimeObjectCustomGetOwnPropertyDescriptor(exec, propertyName, descriptor, this);
}

bool JSHTMLObjectElement::putDelegate(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    return runtimeObjectCustomPut(exec, propertyName, value, impl(), slot);
}

CallType JSHTMLObjectElement::getCallData(CallData& callData)
{
    return runtimeObjectGetCallData(impl(), callData);
}

bool JSHTMLObjectElement::canGetItemsForName(ExecState*, HTMLObjectElement*, const Identifier& propertyName)
{
    return propertyName == "__apple_runtime_object";
}

JSValue JSHTMLObjectElement::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    return runtimeObjectGetter(exec, propertyName, slot);
}

} // namespace WebCore
