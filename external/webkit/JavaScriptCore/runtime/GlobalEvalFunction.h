

#ifndef GlobalEvalFunction_h
#define GlobalEvalFunction_h

#include "PrototypeFunction.h"

namespace JSC {

    class JSGlobalObject;

    class GlobalEvalFunction : public PrototypeFunction {
    public:
        GlobalEvalFunction(ExecState*, NonNullPassRefPtr<Structure>, int len, const Identifier&, NativeFunction, JSGlobalObject* expectedThisObject);
        JSGlobalObject* cachedGlobalObject() const { return m_cachedGlobalObject; }

        static PassRefPtr<Structure> createStructure(JSValue prototype) 
        { 
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

    protected:
        static const unsigned StructureFlags = ImplementsHasInstance | OverridesMarkChildren | OverridesGetPropertyNames | PrototypeFunction::StructureFlags;

    private:
        virtual void markChildren(MarkStack&);

        JSGlobalObject* m_cachedGlobalObject;
    };

} // namespace JSC

#endif // GlobalEvalFunction_h
