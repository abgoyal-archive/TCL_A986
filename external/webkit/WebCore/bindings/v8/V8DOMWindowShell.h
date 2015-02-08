

#ifndef V8DOMWindowShell_h
#define V8DOMWindowShell_h

#include "V8Index.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class DOMWindow;
class Frame;
class String;

// V8WindowShell represents all the per-global object state for a Frame that
// persist between navigations.
class V8DOMWindowShell : public RefCounted<V8DOMWindowShell> {
public:
    static PassRefPtr<V8DOMWindowShell> create(Frame*);

    v8::Handle<v8::Context> context() const { return m_context; }

    // Update document object of the frame.
    void updateDocument();

    // Update the security origin of a document
    // (e.g., after setting docoument.domain).
    void updateSecurityOrigin();

    bool isContextInitialized();

    v8::Persistent<v8::Context> createNewContext(v8::Handle<v8::Object> global, int extensionGroup);
    void setContext(v8::Handle<v8::Context>);
    static bool installDOMWindow(v8::Handle<v8::Context> context, DOMWindow*);

    void initContextIfNeeded();
    void updateDocumentWrapper(v8::Handle<v8::Object> wrapper);

    void clearForNavigation();
    void clearForClose();

    void destroyGlobal();

    static v8::Handle<v8::Value> getHiddenObjectPrototype(v8::Handle<v8::Context>);
    // WARNING: Call |installHiddenObjectPrototype| only on fresh contexts!
    static void installHiddenObjectPrototype(v8::Handle<v8::Context>);

    // To create JS Wrapper objects, we create a cache of a 'boiler plate'
    // object, and then simply Clone that object each time we need a new one.
    // This is faster than going through the full object creation process.
    v8::Local<v8::Object> createWrapperFromCache(V8ClassIndex::V8WrapperType type)
    {
        int classIndex = V8ClassIndex::ToInt(type);
        v8::Local<v8::Object> clone(m_wrapperBoilerplates->CloneElementAt(classIndex));
        return clone.IsEmpty() ? createWrapperFromCacheSlowCase(type) : clone;
    }

    static void setLocation(DOMWindow*, const String& relativeURL);

private:
    V8DOMWindowShell(Frame*);

    void disposeContextHandles();

    void setSecurityToken();
    void clearDocumentWrapper();

    // The JavaScript wrapper for the document object is cached on the global
    // object for fast access. UpdateDocumentWrapperCache sets the wrapper
    // for the current document on the global object. ClearDocumentWrapperCache
    // deletes the document wrapper from the global object.
    void updateDocumentWrapperCache();
    void clearDocumentWrapperCache();

    v8::Local<v8::Object> createWrapperFromCacheSlowCase(V8ClassIndex::V8WrapperType);

    Frame* m_frame;

    // For each possible type of wrapper, we keep a boilerplate object.
    // The boilerplate is used to create additional wrappers of the same
    // type.  We keep a single persistent handle to an array of the
    // activated boilerplates.
    v8::Persistent<v8::Array> m_wrapperBoilerplates;

    v8::Persistent<v8::Context> m_context;
    v8::Persistent<v8::Object> m_global;
    v8::Persistent<v8::Object> m_document;
};

} // namespace WebCore

#endif // V8DOMWindowShell_h
