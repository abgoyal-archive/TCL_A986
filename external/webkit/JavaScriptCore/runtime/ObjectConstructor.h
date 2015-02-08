

#ifndef ObjectConstructor_h
#define ObjectConstructor_h

#include "InternalFunction.h"

namespace JSC {

    class ObjectPrototype;

    class ObjectConstructor : public InternalFunction {
    public:
        ObjectConstructor(ExecState*, NonNullPassRefPtr<Structure>, ObjectPrototype*, Structure* prototypeFunctionStructure);

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);
    };

} // namespace JSC

#endif // ObjectConstructor_h
