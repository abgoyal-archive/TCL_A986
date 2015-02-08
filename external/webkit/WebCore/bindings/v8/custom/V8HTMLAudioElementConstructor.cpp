

#include "config.h"
#include "V8HTMLAudioElementConstructor.h"

#include "HTMLAudioElement.h"
#include "Document.h"
#include "Frame.h"
#include "HTMLNames.h"
#include "V8Binding.h"
#include "V8Document.h"
#include "V8HTMLAudioElement.h"
#include "V8Proxy.h"

#include <wtf/RefPtr.h>

namespace WebCore {

static v8::Handle<v8::Value> v8HTMLAudioElementConstructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.HTMLAudioElement.Contructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    Frame* frame = V8Proxy::retrieveFrameForCurrentContext();
    if (!frame)
        return throwError("Audio constructor associated frame is unavailable", V8Proxy::ReferenceError);

    Document* document = frame->document();
    if (!document)
        return throwError("Audio constructor associated document is unavailable", V8Proxy::ReferenceError);

    // Make sure the document is added to the DOM Node map. Otherwise, the HTMLAudioElement instance
    // may end up being the only node in the map and get garbage-ccollected prematurely.
    toV8(document);

    RefPtr<HTMLAudioElement> audio = new HTMLAudioElement(HTMLNames::audioTag, document);
    audio->setAutobuffer(true);
    if (args.Length() > 0) {
        audio->setSrc(toWebCoreString(args[0]));
        audio->scheduleLoad();
    }

    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::AUDIO), audio.get());
    audio->ref();
    V8DOMWrapper::setJSWrapperForDOMNode(audio.get(), v8::Persistent<v8::Object>::New(args.Holder()));
    return args.Holder();
}

v8::Persistent<v8::FunctionTemplate> V8HTMLAudioElementConstructor::GetTemplate()
{
    static v8::Persistent<v8::FunctionTemplate> cachedTemplate;
    if (!cachedTemplate.IsEmpty())
        return cachedTemplate;

    v8::HandleScope scope;
    v8::Local<v8::FunctionTemplate> result = v8::FunctionTemplate::New(v8HTMLAudioElementConstructorCallback);

    v8::Local<v8::ObjectTemplate> instance = result->InstanceTemplate();
    instance->SetInternalFieldCount(V8HTMLAudioElement::internalFieldCount);
    result->SetClassName(v8::String::New("HTMLAudioElement"));
    result->Inherit(V8HTMLAudioElement::GetTemplate());

    cachedTemplate = v8::Persistent<v8::FunctionTemplate>::New(result);
    return cachedTemplate;
}

} // namespace WebCore
