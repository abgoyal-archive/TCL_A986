

#include "config.h"
#include "V8NodeList.h" 

#include "NodeList.h"
#include "V8Binding.h"
#include "V8Node.h"
#include "V8Proxy.h"

#include <wtf/RefPtr.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

v8::Handle<v8::Value> V8NodeList::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.NodeList.NamedPropertyGetter");
    NodeList* list = V8NodeList::toNative(info.Holder());
    String key = toWebCoreString(name);

    // Length property cannot be overridden.
    DEFINE_STATIC_LOCAL(const AtomicString, length, ("length"));
    if (key == length)
        return v8::Integer::New(list->length());

    RefPtr<Node> result = list->itemWithName(key);
    if (!result)
        return notHandledByInterceptor();

    return toV8(result.release());
}

// Need to support call so that list(0) works.
v8::Handle<v8::Value> V8NodeList::callAsFunctionCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.NodeList.callAsFunction()");
    if (args.Length() < 1)
        return v8::Undefined();

    NodeList* list = V8NodeList::toNative(args.Holder());

    // The first argument must be a number.
    v8::Local<v8::Uint32> index = args[0]->ToArrayIndex();
    if (index.IsEmpty())
        return v8::Undefined();

    RefPtr<Node> result = list->item(index->Uint32Value());
    return toV8(result.release());
}

} // namespace WebCore
