

#include "config.h"
#include "V8HTMLSelectElementCustom.h"

#include "HTMLSelectElement.h"
#include "HTMLOptionElement.h"
#include "HTMLOptionsCollection.h"

#include "V8Binding.h"
#include "V8Collection.h"
#include "V8HTMLOptionElement.h"
#include "V8HTMLSelectElement.h"
#include "V8NamedNodesCollection.h"
#include "V8Node.h"
#include "V8NodeList.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8HTMLSelectElement::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.HTMLSelectElement.NamedPropertyGetter");
    HTMLSelectElement* select = V8HTMLSelectElement::toNative(info.Holder());
    v8::Handle<v8::Value> value = info.Holder()->GetRealNamedPropertyInPrototypeChain(name);

    if (!value.IsEmpty())
        return value;

    // Search local callback properties next to find IDL defined properties.
    if (info.Holder()->HasRealNamedCallbackProperty(name))
        return notHandledByInterceptor();

    PassRefPtr<HTMLOptionsCollection> collection = select->options();

    Vector<RefPtr<Node> > items;
    collection->namedItems(v8StringToAtomicWebCoreString(name), items);

    if (!items.size())
        return notHandledByInterceptor();

    if (items.size() == 1)
        return toV8(items.at(0).release());

    NodeList* list = new V8NamedNodesCollection(items);
    return toV8(list);
}

v8::Handle<v8::Value> V8HTMLSelectElement::indexedPropertyGetter(uint32_t index, const v8::AccessorInfo& info)
{
    ASSERT(V8DOMWrapper::maybeDOMWrapper(info.Holder()));
    RefPtr<Node> result = V8HTMLSelectElement::toNative(info.Holder())->item(index);
    if (!result)
        return notHandledByInterceptor();

    return toV8(result.release());
}

v8::Handle<v8::Value> V8HTMLSelectElement::indexedPropertySetter(uint32_t index, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.HTMLSelectElement.IndexedPropertySetter");
    HTMLSelectElement* select = V8HTMLSelectElement::toNative(info.Holder());
    return toOptionsCollectionSetter(index, value, select);
}

v8::Handle<v8::Value> V8HTMLSelectElement::removeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLSelectElement.remove");
    HTMLSelectElement* imp = V8HTMLSelectElement::toNative(args.Holder());
    return removeElement(imp, args);
}

v8::Handle<v8::Value> removeElement(HTMLSelectElement* imp, const v8::Arguments& args) 
{
    if (V8HTMLOptionElement::HasInstance(args[0])) {
        HTMLOptionElement* element = V8HTMLOptionElement::toNative(v8::Handle<v8::Object>::Cast(args[0]));
        imp->remove(element->index());
        return v8::Undefined();
    }

    imp->remove(toInt32(args[0]));
    return v8::Undefined();
}

} // namespace WebCore
