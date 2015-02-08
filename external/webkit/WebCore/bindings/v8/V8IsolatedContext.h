

#ifndef V8IsolatedContext_h
#define V8IsolatedContext_h

#include "IsolatedWorld.h"
#include "ScriptSourceCode.h" // for WebCore::ScriptSourceCode
#include "V8DOMWindow.h"
#include "V8Index.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include <v8.h>

namespace WebCore {

class V8Proxy;

// V8IsolatedContext
//
// V8IsolatedContext represents a isolated execution environment for
// JavaScript.  Each isolated world executes in parallel with the main
// JavaScript world.  An isolated world has access to the same DOM data
// structures as the main world but none of the JavaScript pointers.
//
// It is an error to ever share a JavaScript pointer between two isolated
// worlds or between an isolated world and the main world.  Because
// isolated worlds have access to the DOM, they need their own DOM wrappers
// to avoid having pointers to the main world's DOM wrappers (which are
// JavaScript objects).
class V8IsolatedContext {
public:
    // Creates an isolated world. To destroy it, call destroy().
    // This will delete the isolated world when the context it owns is GC'd.
    V8IsolatedContext(V8Proxy* proxy, int extensionGroup);
    ~V8IsolatedContext();

    // Call this to destroy the isolated world. It will be deleted sometime
    // after this call, once all script references to the world's context
    // have been dropped.
    void destroy();

    // Returns the isolated world associated with
    // v8::Context::GetEntered().  Because worlds are isolated, the entire
    // JavaScript call stack should be from the same isolated world.
    // Returns 0 if the entered context is from the main world.
    //
    // FIXME: Consider edge cases with DOM mutation events that might
    // violate this invariant.
    //
    static V8IsolatedContext* getEntered()
    {
        // This is a temporary performance optimization.   Essentially,
        // GetHiddenValue is too slow for this code path.  We need to get the
        // V8 team to add a real property to v8::Context for isolated worlds.
        // Until then, we optimize the common case of not having any isolated
        // worlds at all.
        if (!IsolatedWorld::count())
            return 0;
        if (!v8::Context::InContext())
            return 0;
        return reinterpret_cast<V8IsolatedContext*>(getGlobalObject(v8::Context::GetEntered())->GetPointerFromInternalField(V8DOMWindow::enteredIsolatedWorldIndex));
    }

    v8::Handle<v8::Context> context() { return m_context->get(); }
    PassRefPtr<SharedPersistent<v8::Context> > sharedContext() { return m_context; }

    IsolatedWorld* world() const { return m_world.get(); }

private:
    static v8::Handle<v8::Object> getGlobalObject(v8::Handle<v8::Context> context)
    {
        return v8::Handle<v8::Object>::Cast(context->Global()->GetPrototype());
    }

    // Called by the garbage collector when our JavaScript context is about
    // to be destroyed.
    static void contextWeakReferenceCallback(v8::Persistent<v8::Value> object, void* isolatedContext);

    // The underlying v8::Context. This object is keep on the heap as
    // long as |m_context| has not been garbage collected.
    RefPtr<SharedPersistent<v8::Context> > m_context;

    RefPtr<IsolatedWorld> m_world;
};

} // namespace WebCore

#endif // V8IsolatedContext_h
