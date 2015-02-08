

#include "config.h"
#include "JSHTMLAllCollection.h"

#include "AtomicString.h"
#include "HTMLAllCollection.h"
#include "JSDOMBinding.h"
#include "JSHTMLAllCollection.h"
#include "JSNode.h"
#include "JSNodeList.h"
#include "Node.h"
#include "StaticNodeList.h"
#include <wtf/Vector.h>

using namespace JSC;

namespace WebCore {

static JSValue getNamedItems(ExecState* exec, JSHTMLAllCollection* collection, const Identifier& propertyName)
{
    Vector<RefPtr<Node> > namedItems;
    collection->impl()->namedItems(propertyName, namedItems);

    if (namedItems.isEmpty())
        return jsUndefined();
    if (namedItems.size() == 1)
        return toJS(exec, collection->globalObject(), namedItems[0].get());

    // FIXME: HTML5 specifies that this should be a DynamicNodeList.
    // FIXME: HTML5 specifies that non-HTMLOptionsCollection collections should return
    // the first matching item instead of a NodeList.
    return toJS(exec, collection->globalObject(), StaticNodeList::adopt(namedItems).get());
}

// HTMLCollections are strange objects, they support both get and call,
// so that document.forms.item(0) and document.forms(0) both work.
static JSValue JSC_HOST_CALL callHTMLAllCollection(ExecState* exec, JSObject* function, JSValue, const ArgList& args)
{
    if (args.size() < 1)
        return jsUndefined();

    // Do not use thisObj here. It can be the JSHTMLDocument, in the document.forms(i) case.
    JSHTMLAllCollection* jsCollection = static_cast<JSHTMLAllCollection*>(function);
    HTMLAllCollection* collection = static_cast<HTMLAllCollection*>(jsCollection->impl());

    // Also, do we need the TypeError test here ?

    if (args.size() == 1) {
        // Support for document.all(<index>) etc.
        bool ok;
        UString string = args.at(0).toString(exec);
        unsigned index = string.toUInt32(&ok, false);
        if (ok)
            return toJS(exec, jsCollection->globalObject(), collection->item(index));

        // Support for document.images('<name>') etc.
        return getNamedItems(exec, jsCollection, Identifier(exec, string));
    }

    // The second arg, if set, is the index of the item we want
    bool ok;
    UString string = args.at(0).toString(exec);
    unsigned index = args.at(1).toString(exec).toUInt32(&ok, false);
    if (ok) {
        String pstr = string;
        Node* node = collection->namedItem(pstr);
        while (node) {
            if (!index)
                return toJS(exec, jsCollection->globalObject(), node);
            node = collection->nextNamedItem(pstr);
            --index;
        }
    }

    return jsUndefined();
}

CallType JSHTMLAllCollection::getCallData(CallData& callData)
{
    callData.native.function = callHTMLAllCollection;
    return CallTypeHost;
}

bool JSHTMLAllCollection::canGetItemsForName(ExecState*, HTMLAllCollection* collection, const Identifier& propertyName)
{
    Vector<RefPtr<Node> > namedItems;
    collection->namedItems(propertyName, namedItems);
    return !namedItems.isEmpty();
}

JSValue JSHTMLAllCollection::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSHTMLAllCollection* thisObj = static_cast<JSHTMLAllCollection*>(asObject(slot.slotBase()));
    return getNamedItems(exec, thisObj, propertyName);
}

JSValue JSHTMLAllCollection::item(ExecState* exec, const ArgList& args)
{
    bool ok;
    uint32_t index = args.at(0).toString(exec).toUInt32(&ok, false);
    if (ok)
        return toJS(exec, globalObject(), impl()->item(index));
    return getNamedItems(exec, this, Identifier(exec, args.at(0).toString(exec)));
}

JSValue JSHTMLAllCollection::namedItem(ExecState* exec, const ArgList& args)
{
    return getNamedItems(exec, this, Identifier(exec, args.at(0).toString(exec)));
}

} // namespace WebCore
