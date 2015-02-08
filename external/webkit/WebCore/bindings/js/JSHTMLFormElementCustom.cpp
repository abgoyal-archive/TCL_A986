

#include "config.h"
#include "JSHTMLFormElement.h"

#include "Frame.h"
#include "HTMLCollection.h"
#include "HTMLFormElement.h"
#include "JSDOMWindowCustom.h"
#include "JSNodeList.h"
#include "StaticNodeList.h"

using namespace JSC;

namespace WebCore {

bool JSHTMLFormElement::canGetItemsForName(ExecState*, HTMLFormElement* form, const Identifier& propertyName)
{
    Vector<RefPtr<Node> > namedItems;
    form->getNamedElements(propertyName, namedItems);
    return namedItems.size();
}

JSValue JSHTMLFormElement::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSHTMLElement* jsForm = static_cast<JSHTMLFormElement*>(asObject(slot.slotBase()));
    HTMLFormElement* form = static_cast<HTMLFormElement*>(jsForm->impl());

    Vector<RefPtr<Node> > namedItems;
    form->getNamedElements(propertyName, namedItems);
    
    if (namedItems.isEmpty())
        return jsUndefined();
    if (namedItems.size() == 1)
        return toJS(exec, namedItems[0].get());

    // FIXME: HTML5 specifies that this should be a RadioNodeList.
    return toJS(exec, jsForm->globalObject(), StaticNodeList::adopt(namedItems).get());
}

JSValue JSHTMLFormElement::submit(ExecState* exec, const ArgList&)
{
    Frame* activeFrame = asJSDOMWindow(exec->dynamicGlobalObject())->impl()->frame();
    if (!activeFrame)
        return jsUndefined();
    static_cast<HTMLFormElement*>(impl())->submit(activeFrame);
    return jsUndefined();
}

}
