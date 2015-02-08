

#ifndef DateConstructor_h
#define DateConstructor_h

#include "InternalFunction.h"

namespace JSC {

    class DatePrototype;

    class DateConstructor : public InternalFunction {
    public:
        DateConstructor(ExecState*, NonNullPassRefPtr<Structure>, Structure* prototypeFunctionStructure, DatePrototype*);

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);
    };

    JSObject* constructDate(ExecState*, const ArgList&);

} // namespace JSC

#endif // DateConstructor_h
