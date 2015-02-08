

#include "config.h"
#include "V8HTMLFormElement.h"

#include "HTMLCollection.h"
#include "HTMLFormElement.h"
#include "V8Binding.h"
#include "V8NamedNodesCollection.h"
#include "V8Node.h"
#include "V8NodeList.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8HTMLFormElement::indexedPropertyGetter(uint32_t index, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.HTMLFormElement.IndexedPropertyGetter");
    HTMLFormElement* form = V8HTMLFormElement::toNative(info.Holder());

    RefPtr<Node> formElement = form->elements()->item(index);
    if (!formElement)
        return notHandledByInterceptor();
    return toV8(formElement.release());
}

v8::Handle<v8::Value> V8HTMLFormElement::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.HTMLFormElement.NamedPropertyGetter");
    HTMLFormElement* imp = V8HTMLFormElement::toNative(info.Holder());
    AtomicString v = v8StringToAtomicWebCoreString(name);

    // Call getNamedElements twice, first time check if it has a value
    // and let HTMLFormElement update its cache.
    // See issue: 867404
    {
        Vector<RefPtr<Node> > elements;
        imp->getNamedElements(v, elements);
        if (elements.isEmpty())
            return notHandledByInterceptor();
    }

    // Second call may return different results from the first call,
    // but if the first the size cannot be zero.
    Vector<RefPtr<Node> > elements;
    imp->getNamedElements(v, elements);
    ASSERT(!elements.isEmpty());

    if (elements.size() == 1)
        return toV8(elements.at(0).release());

    NodeList* collection = new V8NamedNodesCollection(elements);
    return toV8(collection);
}

v8::Handle<v8::Value> V8HTMLFormElement::submitCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLFormElement.submit()");
    HTMLFormElement* form = V8HTMLFormElement::toNative(args.Holder());
    Frame* frame = V8Proxy::retrieveFrameForEnteredContext();
    if (!frame)
        return v8::Undefined();

    form->submit(frame);
    return v8::Undefined();
}

} // namespace WebCore
