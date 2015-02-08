

#ifndef JSStaticScopeObject_h
#define JSStaticScopeObject_h

#include "JSVariableObject.h"

namespace JSC{
    
    class JSStaticScopeObject : public JSVariableObject {
    protected:
        using JSVariableObject::JSVariableObjectData;
        struct JSStaticScopeObjectData : public JSVariableObjectData {
            JSStaticScopeObjectData()
                : JSVariableObjectData(&symbolTable, &registerStore + 1)
            {
            }
            SymbolTable symbolTable;
            Register registerStore;
        };
        
    public:
        JSStaticScopeObject(ExecState* exec, const Identifier& ident, JSValue value, unsigned attributes)
            : JSVariableObject(exec->globalData().staticScopeStructure, new JSStaticScopeObjectData())
        {
            d()->registerStore = value;
            symbolTable().add(ident.ustring().rep(), SymbolTableEntry(-1, attributes));
        }
        virtual ~JSStaticScopeObject();
        virtual void markChildren(MarkStack&);
        bool isDynamicScope() const;
        virtual JSObject* toThisObject(ExecState*) const;
        virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
        virtual void put(ExecState*, const Identifier&, JSValue, PutPropertySlot&);
        void putWithAttributes(ExecState*, const Identifier&, JSValue, unsigned attributes);

        static PassRefPtr<Structure> createStructure(JSValue proto) { return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); }

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | NeedsThisConversion | OverridesMarkChildren | OverridesGetPropertyNames | JSVariableObject::StructureFlags;

    private:
        JSStaticScopeObjectData* d() { return static_cast<JSStaticScopeObjectData*>(JSVariableObject::d); }
    };

}

#endif // JSStaticScopeObject_h
