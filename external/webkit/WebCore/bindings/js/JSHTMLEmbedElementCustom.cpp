

#include "config.h"
#include "JSHTMLEmbedElementCustom.h"

#include "HTMLEmbedElement.h"
#include "JSPluginElementFunctions.h"

namespace WebCore {

using namespace JSC;

bool JSHTMLEmbedElement::getOwnPropertySlotDelegate(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return runtimeObjectCustomGetOwnPropertySlot(exec, propertyName, slot, this);
}

bool JSHTMLEmbedElement::getOwnPropertyDescriptorDelegate(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return runtimeObjectCustomGetOwnPropertyDescriptor(exec, propertyName, descriptor, this);
}

bool JSHTMLEmbedElement::putDelegate(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    return runtimeObjectCustomPut(exec, propertyName, value, impl(), slot);
}

CallType JSHTMLEmbedElement::getCallData(CallData& callData)
{
    return runtimeObjectGetCallData(impl(), callData);
}

bool JSHTMLEmbedElement::canGetItemsForName(ExecState*, HTMLEmbedElement*, const Identifier& propertyName)
{
    return propertyName == "__apple_runtime_object";
}

JSValue JSHTMLEmbedElement::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    return runtimeObjectGetter(exec, propertyName, slot);
}

} // namespace WebCore
