

#include "config.h"
#include "JSStyleSheet.h"

#include "CSSStyleSheet.h"
#include "Node.h"
#include "JSCSSStyleSheet.h"
#include "JSNode.h"

using namespace JSC;

namespace WebCore {

JSValue toJS(ExecState* exec, JSDOMGlobalObject* globalObject, StyleSheet* styleSheet)
{
    if (!styleSheet)
        return jsNull();

    DOMObject* wrapper = getCachedDOMObjectWrapper(exec, styleSheet);
    if (wrapper)
        return wrapper;

    if (styleSheet->isCSSStyleSheet())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSStyleSheet, styleSheet);
    else
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, StyleSheet, styleSheet);

    return wrapper;
}

void JSStyleSheet::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    StyleSheet* sheet = impl();
    JSGlobalData& globalData = *Heap::heap(this)->globalData();

    unsigned length = sheet->length();
    for (unsigned i = 0; i < length; ++i)
        markDOMObjectWrapper(markStack, globalData, sheet->item(i));

    // This prevents us from having a style sheet with a dangling ownerNode pointer.
    // A better solution would be to handle this on the DOM side -- if the style sheet
    // is kept around, then we want the node to stay around too. One possibility would
    // be to make ref/deref on the style sheet ref/deref the node instead, but there's
    // a lot of disentangling of the CSS DOM objects that would need to happen first.
    if (Node* ownerNode = sheet->ownerNode())
        markDOMNodeWrapper(markStack, ownerNode->document(), ownerNode);
}

} // namespace WebCore
