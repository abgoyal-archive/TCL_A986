

#ifndef PrototypeFunction_h
#define PrototypeFunction_h

#include "InternalFunction.h"
#include "CallData.h"

namespace JSC {

    class PrototypeFunction : public InternalFunction {
    public:
        PrototypeFunction(ExecState*, int length, const Identifier&, NativeFunction);
        PrototypeFunction(ExecState*, NonNullPassRefPtr<Structure>, int length, const Identifier&, NativeFunction);

    private:
        virtual CallType getCallData(CallData&);

        const NativeFunction m_function;
    };

} // namespace JSC

#endif // PrototypeFunction_h
