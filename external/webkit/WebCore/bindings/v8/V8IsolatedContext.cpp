

#include "config.h"

#include "V8IsolatedContext.h"

#include "Frame.h"
#include "FrameLoaderClient.h"
#include "HashMap.h"
#include "ScriptController.h"
#include "V8DOMWindow.h"
#include "V8HiddenPropertyName.h"
#include <v8.h>

namespace WebCore {


void V8IsolatedContext::contextWeakReferenceCallback(v8::Persistent<v8::Value> object, void* isolatedContext)
{
    // Our context is going away.  Time to clean up the world.
    V8IsolatedContext* context = static_cast<V8IsolatedContext*>(isolatedContext);
    delete context;
}

V8IsolatedContext::V8IsolatedContext(V8Proxy* proxy, int extensionGroup)
    : m_world(IsolatedWorld::create())
{
    v8::HandleScope scope;
    // FIXME: We should be creating a new V8DOMWindowShell here instead of riping out the context.
    m_context = SharedPersistent<v8::Context>::create(proxy->windowShell()->createNewContext(v8::Handle<v8::Object>(), extensionGroup));
    if (m_context->get().IsEmpty())
        return;

    // Run code in the new context.
    v8::Context::Scope contextScope(m_context->get());

    getGlobalObject(m_context->get())->SetPointerInInternalField(V8DOMWindow::enteredIsolatedWorldIndex, this);

    V8DOMWindowShell::installHiddenObjectPrototype(m_context->get());
    // FIXME: This will go away once we have a windowShell for the isolated world.
    proxy->windowShell()->installDOMWindow(m_context->get(), proxy->frame()->domWindow());

    // Using the default security token means that the canAccess is always
    // called, which is slow.
    // FIXME: Use tokens where possible. This will mean keeping track of all
    //        created contexts so that they can all be updated when the
    //        document domain
    //        changes.
    m_context->get()->UseDefaultSecurityToken();

    proxy->frame()->loader()->client()->didCreateIsolatedScriptContext();
}

void V8IsolatedContext::destroy()
{
    m_context->get().MakeWeak(this, &contextWeakReferenceCallback);
}

V8IsolatedContext::~V8IsolatedContext()
{
    m_context->disposeHandle();
}

} // namespace WebCore
