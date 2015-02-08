

#include "config.h"
#include "JSHTMLFrameSetElement.h"

#include "Document.h"
#include "HTMLCollection.h"
#include "HTMLFrameElement.h"
#include "HTMLFrameSetElement.h"
#include "HTMLNames.h"
#include "JSDOMWindow.h"
#include "JSDOMWindowShell.h"
#include "JSDOMBinding.h"

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

bool JSHTMLFrameSetElement::canGetItemsForName(ExecState*, HTMLFrameSetElement* frameSet, const Identifier& propertyName)
{
    Node* frame = frameSet->children()->namedItem(propertyName);
    return frame && frame->hasTagName(frameTag);
}

JSValue JSHTMLFrameSetElement::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSHTMLElement* thisObj = static_cast<JSHTMLElement*>(asObject(slot.slotBase()));
    HTMLElement* element = static_cast<HTMLElement*>(thisObj->impl());

    Node* frame = element->children()->namedItem(propertyName);
    if (Document* doc = static_cast<HTMLFrameElement*>(frame)->contentDocument()) {
        if (JSDOMWindowShell* window = toJSDOMWindowShell(doc->frame(), currentWorld(exec)))
            return window;
    }

    return jsUndefined();
}

} // namespace WebCore
