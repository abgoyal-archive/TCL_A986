

#ifndef ArrayConstructor_h
#define ArrayConstructor_h

#include "InternalFunction.h"

namespace JSC {

    class ArrayPrototype;

    class ArrayConstructor : public InternalFunction {
    public:
        ArrayConstructor(ExecState*, NonNullPassRefPtr<Structure>, ArrayPrototype*, Structure*);

        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);
    };

} // namespace JSC

#endif // ArrayConstructor_h
