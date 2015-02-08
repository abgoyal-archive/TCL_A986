

#ifndef NativeErrorConstructor_h
#define NativeErrorConstructor_h

#include "InternalFunction.h"

namespace JSC {

    class ErrorInstance;
    class FunctionPrototype;
    class NativeErrorPrototype;

    class NativeErrorConstructor : public InternalFunction {
    public:
        NativeErrorConstructor(ExecState*, NonNullPassRefPtr<Structure>, NativeErrorPrototype*);

        static const ClassInfo info;

        ErrorInstance* construct(ExecState*, const ArgList&);

    private:
        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);

        virtual const ClassInfo* classInfo() const { return &info; }

        RefPtr<Structure> m_errorStructure;
    };

} // namespace JSC

#endif // NativeErrorConstructor_h
