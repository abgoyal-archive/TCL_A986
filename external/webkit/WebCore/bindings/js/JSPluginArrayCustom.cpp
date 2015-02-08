

#include "config.h"
#include "JSPluginArray.h"

#include "AtomicString.h"
#include "JSPlugin.h"
#include "PluginArray.h"

namespace WebCore {

using namespace JSC;

bool JSPluginArray::canGetItemsForName(ExecState*, PluginArray* pluginArray, const Identifier& propertyName)
{
    return pluginArray->canGetItemsForName(propertyName);
}

JSValue JSPluginArray::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSPluginArray* thisObj = static_cast<JSPluginArray*>(asObject(slot.slotBase()));
    return toJS(exec, thisObj->impl()->namedItem(propertyName));
}

} // namespace WebCore
