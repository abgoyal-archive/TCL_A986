

#include "config.h"
#include "JSCSSRuleList.h"

#include "CSSRuleList.h"

using namespace JSC;

namespace WebCore {

void JSCSSRuleList::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    CSSRuleList* list = impl();
    JSGlobalData& globalData = *Heap::heap(this)->globalData();

    unsigned length = list->length();
    for (unsigned i = 0; i < length; ++i)
        markDOMObjectWrapper(markStack, globalData, list->item(i));
}

}
