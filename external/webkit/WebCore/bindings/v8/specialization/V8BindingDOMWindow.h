

#ifndef V8BindingDOMWindow_h
#define V8BindingDOMWindow_h

#include "BindingDOMWindow.h"
#include "GenericBinding.h"
#include "V8Proxy.h"

namespace WebCore {

class V8Binding;

class V8BindingDOMWindow : public BindingDOMWindow<V8Binding> {
public:
    static void storeDialogArgs(State<V8Binding>*, Frame* newFrame, v8::Handle<v8::Value> dialogArgs)
    {
        // Set dialog arguments on the global object of the new frame.
        if (!dialogArgs.IsEmpty()) {
            v8::Local<v8::Context> context = V8Proxy::context(newFrame);
            if (!context.IsEmpty()) {
                v8::Context::Scope scope(context);
                context->Global()->Set(v8::String::New("dialogArguments"), dialogArgs);
            }
        }
    }
};

} // namespace WebCore

#endif // V8BindingDOMWindow_h
