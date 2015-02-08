
 
#ifndef JSActivation_h
#define JSActivation_h

#include "CodeBlock.h"
#include "JSVariableObject.h"
#include "RegisterFile.h"
#include "SymbolTable.h"
#include "Nodes.h"

namespace JSC {

    class Arguments;
    class Register;
    
    class JSActivation : public JSVariableObject {
        typedef JSVariableObject Base;
    public:
        JSActivation(CallFrame*, NonNullPassRefPtr<FunctionExecutable>);
        virtual ~JSActivation();

        virtual void markChildren(MarkStack&);

        virtual bool isDynamicScope() const;

        virtual bool isActivationObject() const { return true; }

        virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);

        virtual void put(ExecState*, const Identifier&, JSValue, PutPropertySlot&);

        virtual void putWithAttributes(ExecState*, const Identifier&, JSValue, unsigned attributes);
        virtual bool deleteProperty(ExecState*, const Identifier& propertyName);

        virtual JSObject* toThisObject(ExecState*) const;

        void copyRegisters(Arguments* arguments);
        
        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;

        static PassRefPtr<Structure> createStructure(JSValue proto) { return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); }

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | NeedsThisConversion | OverridesMarkChildren | OverridesGetPropertyNames | JSVariableObject::StructureFlags;

    private:
        struct JSActivationData : public JSVariableObjectData {
            JSActivationData(NonNullPassRefPtr<FunctionExecutable> _functionExecutable, Register* registers)
                : JSVariableObjectData(_functionExecutable->generatedBytecode().symbolTable(), registers)
                , functionExecutable(_functionExecutable)
            {
                // We have to manually ref and deref the symbol table as JSVariableObjectData
                // doesn't know about SharedSymbolTable
                functionExecutable->generatedBytecode().sharedSymbolTable()->ref();
            }
            ~JSActivationData()
            {
                static_cast<SharedSymbolTable*>(symbolTable)->deref();
            }

            RefPtr<FunctionExecutable> functionExecutable;
        };
        
        static JSValue argumentsGetter(ExecState*, const Identifier&, const PropertySlot&);
        NEVER_INLINE PropertySlot::GetValueFunc getArgumentsGetter();

        JSActivationData* d() const { return static_cast<JSActivationData*>(JSVariableObject::d); }
    };

    JSActivation* asActivation(JSValue);

    inline JSActivation* asActivation(JSValue value)
    {
        ASSERT(asObject(value)->inherits(&JSActivation::info));
        return static_cast<JSActivation*>(asObject(value));
    }

} // namespace JSC

#endif // JSActivation_h
