

#ifndef FunctionConstructor_h
#define FunctionConstructor_h

#include "InternalFunction.h"

namespace JSC {

    class FunctionPrototype;

    class FunctionConstructor : public InternalFunction {
    public:
        FunctionConstructor(ExecState*, NonNullPassRefPtr<Structure>, FunctionPrototype*);

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);
    };

    JSObject* constructFunction(ExecState*, const ArgList&, const Identifier& functionName, const UString& sourceURL, int lineNumber);
    JSObject* constructFunction(ExecState*, const ArgList&);

} // namespace JSC

#endif // FunctionConstructor_h
