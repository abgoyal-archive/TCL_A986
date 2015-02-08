

#ifndef ScriptFunctionCall_h
#define ScriptFunctionCall_h

#include "PlatformString.h"
#include "ScriptObject.h"
#include "ScriptState.h"

#include <runtime/ArgList.h>

namespace JSC {
    class UString;
    class JSValue;
}

namespace WebCore {
    class ScriptValue;
    class ScriptString;

    class ScriptFunctionCall {
    public:
        ScriptFunctionCall(const ScriptObject& thisObject, const String& name);
        virtual ~ScriptFunctionCall() {};

        void appendArgument(const ScriptObject&);
        void appendArgument(const ScriptString&);
        void appendArgument(const ScriptValue&);
        void appendArgument(const String&);
        void appendArgument(const char*);
        void appendArgument(const JSC::UString&);
        void appendArgument(JSC::JSValue);
        void appendArgument(long);
        void appendArgument(long long);
        void appendArgument(unsigned int);
        void appendArgument(unsigned long);
        void appendArgument(int);
        void appendArgument(bool);
        ScriptValue call(bool& hadException, bool reportExceptions = true);
        ScriptValue call();
        ScriptObject construct(bool& hadException, bool reportExceptions = true);

    protected:
        ScriptState* m_exec;
        ScriptObject m_thisObject;
        String m_name;
        JSC::MarkedArgumentBuffer m_arguments;

    private:
        // MarkedArgumentBuffer must be stack allocated, so prevent heap
        // alloc of ScriptFunctionCall as well.
        void* operator new(size_t) { ASSERT_NOT_REACHED(); return reinterpret_cast<void*>(0xbadbeef); }
        void* operator new[](size_t) { ASSERT_NOT_REACHED(); return reinterpret_cast<void*>(0xbadbeef); }
    };

} // namespace WebCore

#endif // ScriptFunctionCall
