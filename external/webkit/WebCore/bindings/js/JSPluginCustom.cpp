

#include "config.h"
#include "JSPlugin.h"

#include "AtomicString.h"
#include "JSMimeType.h"
#include "Plugin.h"

namespace WebCore {

using namespace JSC;

bool JSPlugin::canGetItemsForName(ExecState*, Plugin* plugin, const Identifier& propertyName)
{
    return plugin->canGetItemsForName(propertyName);
}

JSValue JSPlugin::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSPlugin* thisObj = static_cast<JSPlugin*>(asObject(slot.slotBase()));
    return toJS(exec, thisObj->impl()->namedItem(propertyName));
}

} // namespace WebCore
