

#ifndef ScriptFunctionCall_h
#define ScriptFunctionCall_h

#include "PlatformString.h"
#include "ScriptObject.h"

#include <wtf/Vector.h>

namespace WebCore {
    class ScriptValue;
    class ScriptState;
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
        ScriptState* m_scriptState;
        ScriptObject m_thisObject;
        String m_name;
        Vector<ScriptValue> m_arguments;
    };

} // namespace WebCore

#endif // ScriptFunctionCall
