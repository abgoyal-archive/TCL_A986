

#include "config.h"
#include "JSStyleSheetList.h"

#include "HTMLStyleElement.h"
#include "JSStyleSheet.h"
#include "StyleSheet.h"
#include "StyleSheetList.h"

using namespace JSC;

namespace WebCore {

void JSStyleSheetList::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    StyleSheetList* list = impl();
    JSGlobalData& globalData = *Heap::heap(this)->globalData();

    unsigned length = list->length();
    for (unsigned i = 0; i < length; ++i)
        markDOMObjectWrapper(markStack, globalData, list->item(i));
}

bool JSStyleSheetList::canGetItemsForName(ExecState*, StyleSheetList* styleSheetList, const Identifier& propertyName)
{
    return styleSheetList->getNamedItem(propertyName);
}

JSValue JSStyleSheetList::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSStyleSheetList* thisObj = static_cast<JSStyleSheetList*>(asObject(slot.slotBase()));
    HTMLStyleElement* element = thisObj->impl()->getNamedItem(propertyName);
    ASSERT(element);
    return toJS(exec, element->sheet());
}

} // namespace WebCore
