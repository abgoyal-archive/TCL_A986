

#include "config.h"
#include "JSNamedNodeMap.h"

#include "JSNode.h"

#include "Element.h"
#include "NamedNodeMap.h"

using namespace JSC;

namespace WebCore {

bool JSNamedNodeMap::canGetItemsForName(ExecState*, NamedNodeMap* impl, const Identifier& propertyName)
{
    return impl->getNamedItem(propertyName);
}

JSValue JSNamedNodeMap::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSNamedNodeMap* thisObj = static_cast<JSNamedNodeMap*>(asObject(slot.slotBase()));
    return toJS(exec, thisObj->impl()->getNamedItem(propertyName));
}

void JSNamedNodeMap::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    // Mark the element so that this will work to access the attribute even if the last
    // other reference goes away.
    if (Element* element = impl()->element())
        markDOMNodeWrapper(markStack, element->document(), element);
}

} // namespace WebCore
