

#ifndef ScriptScope_h
#define ScriptScope_h

#include <v8.h>

namespace WebCore {
    class ScriptState;

    class ScriptScope {
    public:
        ScriptScope(ScriptState* scriptState, bool reportExceptions = true);
        bool success();

        v8::Local<v8::Object> global() const { return m_context->Global(); }

    private:
        v8::HandleScope m_handleScope;
        v8::Local<v8::Context> m_context;
        v8::Context::Scope m_scope;
        v8::TryCatch m_exceptionCatcher;
        ScriptState* m_scriptState;
    };

}

#endif // ScriptScope_h
