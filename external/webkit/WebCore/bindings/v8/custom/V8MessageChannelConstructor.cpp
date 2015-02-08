

#include "config.h"
#include "V8MessageChannel.h"

#include "Document.h"
#include "Frame.h"
#include "MessageChannel.h"
#include "V8Binding.h"
#include "V8MessagePort.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"

#include <wtf/RefPtr.h>

namespace WebCore {

v8::Handle<v8::Value> V8MessageChannel::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.MessageChannel.Constructor");
    // FIXME: The logic here is almost exact duplicate of V8::constructDOMObject.
    // Consider refactoring to reduce duplication.
    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    // Get the ScriptExecutionContext (WorkerContext or Document)
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return v8::Undefined();

    // Note: it's OK to let this RefPtr go out of scope because we also call
    // SetDOMWrapper(), which effectively holds a reference to obj.
    RefPtr<MessageChannel> obj = MessageChannel::create(context);

    v8::Local<v8::Object> messageChannel = args.Holder();

    // Create references from the MessageChannel wrapper to the two
    // MessagePort wrappers to make sure that the MessagePort wrappers
    // stay alive as long as the MessageChannel wrapper is around.
    messageChannel->SetInternalField(V8MessageChannel::port1Index, toV8(obj->port1()));
    messageChannel->SetInternalField(V8MessageChannel::port2Index, toV8(obj->port2()));

    // Setup the standard wrapper object internal fields.
    V8DOMWrapper::setDOMWrapper(messageChannel, V8ClassIndex::MESSAGECHANNEL, obj.get());
    return toV8(obj.release(), messageChannel);
}


} // namespace WebCore
