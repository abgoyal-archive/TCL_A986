

#ifndef ErrorConstructor_h
#define ErrorConstructor_h

#include "ErrorInstance.h"
#include "InternalFunction.h"

namespace JSC {

    class ErrorPrototype;

    class ErrorConstructor : public InternalFunction {
    public:
        ErrorConstructor(ExecState*, NonNullPassRefPtr<Structure>, ErrorPrototype*);

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);
    };

    ErrorInstance* constructError(ExecState*, const ArgList&);

} // namespace JSC

#endif // ErrorConstructor_h
