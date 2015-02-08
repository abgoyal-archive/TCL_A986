

#ifndef ScriptState_h
#define ScriptState_h

#include "DOMWrapperWorld.h"
#include <v8.h>
#include <wtf/Noncopyable.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    class DOMWrapperWorld;
    class Frame;
    class Node;
    class Page;

    class ScriptState : public Noncopyable {
    public:
        bool hadException() { return !m_exception.IsEmpty(); }
        void setException(v8::Local<v8::Value> exception)
        {
            m_exception = exception;
        }
        v8::Local<v8::Value> exception() { return m_exception; }

        v8::Local<v8::Context> context() const
        {
            return v8::Local<v8::Context>::New(m_context);
        }

        static ScriptState* forContext(v8::Local<v8::Context>);
        static ScriptState* current();

    protected:
        ScriptState() { }
        ~ScriptState();

    private:
        friend ScriptState* mainWorldScriptState(Frame*);
        explicit ScriptState(v8::Handle<v8::Context>);

        static void weakReferenceCallback(v8::Persistent<v8::Value> object, void* parameter);

        v8::Local<v8::Value> m_exception;
        v8::Persistent<v8::Context> m_context;
    };

    class EmptyScriptState : public ScriptState {
    public:
        EmptyScriptState() : ScriptState() { }
        ~EmptyScriptState() { }
    };

    ScriptState* mainWorldScriptState(Frame*);

    ScriptState* scriptStateFromNode(DOMWrapperWorld*, Node*);
    ScriptState* scriptStateFromPage(DOMWrapperWorld*, Page*);

    inline DOMWrapperWorld* debuggerWorld() { return mainThreadNormalWorld(); }
    inline DOMWrapperWorld* pluginWorld() { return mainThreadNormalWorld(); }

}

#endif // ScriptState_h
