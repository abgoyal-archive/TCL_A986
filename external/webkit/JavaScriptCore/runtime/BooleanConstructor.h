

#ifndef BooleanConstructor_h
#define BooleanConstructor_h

#include "InternalFunction.h"

namespace JSC {

    class BooleanPrototype;

    class BooleanConstructor : public InternalFunction {
    public:
        BooleanConstructor(ExecState*, NonNullPassRefPtr<Structure>, BooleanPrototype*);

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);
    };

    JSObject* constructBooleanFromImmediateBoolean(ExecState*, JSValue);
    JSObject* constructBoolean(ExecState*, const ArgList&);

} // namespace JSC

#endif // BooleanConstructor_h
