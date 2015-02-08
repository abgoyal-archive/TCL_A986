

#include "config.h"
#include "V8HTMLImageElementConstructor.h"

#include "HTMLImageElement.h"
#include "Document.h"
#include "Frame.h"
#include "HTMLNames.h"
#include "V8Binding.h"
#include "V8Document.h"
#include "V8HTMLImageElement.h"
#include "V8Proxy.h"

#include <wtf/RefPtr.h>

namespace WebCore {

static v8::Handle<v8::Value> v8HTMLImageElementConstructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLImageElement.Contructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    Frame* frame = V8Proxy::retrieveFrameForCurrentContext();
    if (!frame)
        return throwError("Image constructor associated frame is unavailable", V8Proxy::ReferenceError);

    Document* document = frame->document();
    if (!document)
        return throwError("Image constructor associated document is unavailable", V8Proxy::ReferenceError);

    // Make sure the document is added to the DOM Node map. Otherwise, the HTMLImageElement instance
    // may end up being the only node in the map and get garbage-ccollected prematurely.
    toV8(document);

    RefPtr<HTMLImageElement> image = new HTMLImageElement(HTMLNames::imgTag, document);
    if (args.Length() > 0) {
        image->setWidth(toInt32(args[0]));
        if (args.Length() > 1)
            image->setHeight(toInt32(args[1]));
    }

    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::IMAGE), image.get());
    image->ref();
    V8DOMWrapper::setJSWrapperForDOMNode(image.get(), v8::Persistent<v8::Object>::New(args.Holder()));
    return args.Holder();
}

v8::Persistent<v8::FunctionTemplate> V8HTMLImageElementConstructor::GetTemplate()
{
    static v8::Persistent<v8::FunctionTemplate> cachedTemplate;
    if (!cachedTemplate.IsEmpty())
        return cachedTemplate;

    v8::HandleScope scope;
    v8::Local<v8::FunctionTemplate> result = v8::FunctionTemplate::New(v8HTMLImageElementConstructorCallback);

    v8::Local<v8::ObjectTemplate> instance = result->InstanceTemplate();
    instance->SetInternalFieldCount(V8HTMLImageElement::internalFieldCount);
    result->SetClassName(v8::String::New("HTMLImageElement"));
    result->Inherit(V8HTMLImageElement::GetTemplate());

    cachedTemplate = v8::Persistent<v8::FunctionTemplate>::New(result);
    return cachedTemplate;
}

} // namespace WebCore
