

#include "config.h"
#include "V8HTMLAllCollection.h"

#include "HTMLAllCollection.h"

#include "V8Binding.h"
#include "V8NamedNodesCollection.h"
#include "V8Node.h"
#include "V8NodeList.h"
#include "V8Proxy.h"

namespace WebCore {

static v8::Handle<v8::Value> getNamedItems(HTMLAllCollection* collection, AtomicString name)
{
    Vector<RefPtr<Node> > namedItems;
    collection->namedItems(name, namedItems);

    if (!namedItems.size())
        return v8::Handle<v8::Value>();

    if (namedItems.size() == 1)
        return toV8(namedItems.at(0).release());

    NodeList* list = new V8NamedNodesCollection(namedItems);
    return toV8(list);
}

static v8::Handle<v8::Value> getItem(HTMLAllCollection* collection, v8::Handle<v8::Value> argument)
{
    v8::Local<v8::Uint32> index = argument->ToArrayIndex();
    if (index.IsEmpty()) {
        v8::Handle<v8::Value> result = getNamedItems(collection, toWebCoreString(argument->ToString()));

        if (result.IsEmpty())
            return v8::Undefined();

        return result;
    }

    RefPtr<Node> result = collection->item(index->Uint32Value());
    return toV8(result.release());
}

v8::Handle<v8::Value> V8HTMLAllCollection::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.HTMLAllCollection.NamedPropertyGetter");
    // Search the prototype chain first.
    v8::Handle<v8::Value> value = info.Holder()->GetRealNamedPropertyInPrototypeChain(name);

    if (!value.IsEmpty())
        return value;

    // Search local callback properties next to find IDL defined
    // properties.
    if (info.Holder()->HasRealNamedCallbackProperty(name))
        return v8::Handle<v8::Value>();

    // Finally, search the DOM structure.
    HTMLAllCollection* imp = V8HTMLAllCollection::toNative(info.Holder());
    return getNamedItems(imp, v8StringToAtomicWebCoreString(name));
}

v8::Handle<v8::Value> V8HTMLAllCollection::itemCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLAllCollection.item()");
    HTMLAllCollection* imp = V8HTMLAllCollection::toNative(args.Holder());
    return getItem(imp, args[0]);
}

v8::Handle<v8::Value> V8HTMLAllCollection::namedItemCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLAllCollection.namedItem()");
    HTMLAllCollection* imp = V8HTMLAllCollection::toNative(args.Holder());
    v8::Handle<v8::Value> result = getNamedItems(imp, toWebCoreString(args[0]));

    if (result.IsEmpty())
        return v8::Undefined();

    return result;
}

v8::Handle<v8::Value> V8HTMLAllCollection::callAsFunctionCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLAllCollection.callAsFunction()");
    if (args.Length() < 1)
        return v8::Undefined();

    HTMLAllCollection* imp = V8HTMLAllCollection::toNative(args.Holder());

    if (args.Length() == 1)
        return getItem(imp, args[0]);

    // If there is a second argument it is the index of the item we want.
    String name = toWebCoreString(args[0]);
    v8::Local<v8::Uint32> index = args[1]->ToArrayIndex();
    if (index.IsEmpty())
        return v8::Undefined();

    unsigned current = index->Uint32Value();
    Node* node = imp->namedItem(name);
    while (node) {
        if (!current)
            return toV8(node);

        node = imp->nextNamedItem(name);
        current--;
    }

    return v8::Undefined();
}

} // namespace WebCore
