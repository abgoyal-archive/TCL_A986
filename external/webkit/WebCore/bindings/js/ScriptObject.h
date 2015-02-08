

#ifndef ScriptObject_h
#define ScriptObject_h

#include "ScriptState.h"
#include "ScriptValue.h"

#include <runtime/JSObject.h>
#include <runtime/Protect.h>

namespace WebCore {
    class InjectedScriptHost;
    class InspectorBackend;
    class InspectorFrontendHost;

    class ScriptObject : public ScriptValue {
    public:
        ScriptObject(ScriptState*, JSC::JSObject*);
        ScriptObject() {}
        JSC::JSObject* jsObject() const { return asObject(jsValue()); }
        ScriptState* scriptState() const { return m_scriptState; }

        bool set(const String& name, const String&);
        bool set(const char* name, const ScriptObject&);
        bool set(const char* name, const String&);
        bool set(const char* name, double);
        bool set(const char* name, long);
        bool set(const char* name, long long);
        bool set(const char* name, int);
        bool set(const char* name, unsigned);
        bool set(const char* name, unsigned long);
        bool set(const char* name, bool);

        static ScriptObject createNew(ScriptState*);
    
    protected:
        ScriptState* m_scriptState;
    };

    class ScriptGlobalObject {
    public:
        static bool set(ScriptState*, const char* name, const ScriptObject&);
#if ENABLE(INSPECTOR)
        static bool set(ScriptState*, const char* name, InspectorBackend*);
        static bool set(ScriptState*, const char* name, InspectorFrontendHost*);
        static bool set(ScriptState*, const char* name, InjectedScriptHost*);
#endif
        static bool get(ScriptState*, const char* name, ScriptObject&);
        static bool remove(ScriptState*, const char* name);
    private:
        ScriptGlobalObject() { }
    };

}

#endif // ScriptObject_h
