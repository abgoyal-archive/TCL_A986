

#ifndef Completion_h
#define Completion_h

#include "JSValue.h"

namespace JSC {

    class ExecState;
    class ScopeChain;
    class SourceCode;

    enum ComplType { Normal, Break, Continue, ReturnValue, Throw, Interrupted };

    /*
     * Completion objects are used to convey the return status and value
     * from functions.
     */
    class Completion {
    public:
        Completion(ComplType type = Normal, JSValue value = JSValue())
            : m_type(type)
            , m_value(value)
        {
        }

        ComplType complType() const { return m_type; }
        JSValue value() const { return m_value; }
        void setValue(JSValue v) { m_value = v; }
        bool isValueCompletion() const { return m_value; }

    private:
        ComplType m_type;
        JSValue m_value;
    };

    Completion checkSyntax(ExecState*, const SourceCode&);
    Completion evaluate(ExecState*, ScopeChain&, const SourceCode&, JSValue thisValue = JSValue());

} // namespace JSC

#endif // Completion_h
