

#include "config.h"

#include "ExceptionCode.h"
#include "MessagePort.h"
#include "SerializedScriptValue.h"
#include "V8Binding.h"
#include "V8MessagePortCustom.h"
#include "V8MessagePort.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8MessagePort::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.MessagePort.addEventListener()");
    MessagePort* messagePort = V8MessagePort::toNative(args.Holder());
    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(messagePort, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        messagePort->addEventListener(type, listener, useCapture);

        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8MessagePort::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.MessagePort.removeEventListener()");
    MessagePort* messagePort = V8MessagePort::toNative(args.Holder());
    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(messagePort, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        messagePort->removeEventListener(type, listener.get(), useCapture);

        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

v8::Handle<v8::Value> V8MessagePort::postMessageCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.MessagePort.postMessage");
    MessagePort* messagePort = V8MessagePort::toNative(args.Holder());
    RefPtr<SerializedScriptValue> message = SerializedScriptValue::create(args[0]);
    MessagePortArray portArray;
    if (args.Length() > 1) {
        if (!getMessagePortArray(args[1], portArray))
            return v8::Undefined();
    }
    ExceptionCode ec = 0;
    messagePort->postMessage(message.release(), &portArray, ec);
    return throwError(ec);
}

bool getMessagePortArray(v8::Local<v8::Value> value, MessagePortArray& portArray)
{
    if (isUndefinedOrNull(value)) {
        portArray.resize(0);
        return true;
    }

    if (!value->IsObject()) {
        throwError("MessagePortArray argument must be an object");
        return false;
    }
    uint32_t length = 0;
    v8::Local<v8::Object> ports = v8::Local<v8::Object>::Cast(value);

    if (value->IsArray()) {
        v8::Local<v8::Array> array = v8::Local<v8::Array>::Cast(value);
        length = array->Length();
    } else {
        // Sequence-type object - get the length attribute
        v8::Local<v8::Value> sequenceLength = ports->Get(v8::String::New("length"));
        if (!sequenceLength->IsNumber()) {
            throwError("MessagePortArray argument has no length attribute");
            return false;
        }
        length = sequenceLength->Uint32Value();
    }
    portArray.resize(length);

    for (unsigned int i = 0; i < length; ++i) {
        v8::Local<v8::Value> port = ports->Get(v8::Integer::New(i));
        // Validation of non-null objects, per HTML5 spec 8.3.3.
        if (isUndefinedOrNull(port)) {
            throwError(INVALID_STATE_ERR);
            return false;
        }
        // Validation of Objects implementing an interface, per WebIDL spec 4.1.15.
        if (!V8MessagePort::HasInstance(port)) {
            throwError("MessagePortArray argument must contain only MessagePorts");
            return false;
        }
        portArray[i] = V8MessagePort::toNative(v8::Handle<v8::Object>::Cast(port));
    }
    return true;
}

} // namespace WebCore
