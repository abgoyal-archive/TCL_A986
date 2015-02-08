

#ifndef JSFunction_h
#define JSFunction_h

#include "InternalFunction.h"

namespace JSC {

    class ExecutableBase;
    class FunctionExecutable;
    class FunctionPrototype;
    class JSActivation;
    class JSGlobalObject;

    class JSFunction : public InternalFunction {
        friend class JIT;
        friend class JSGlobalData;

        typedef InternalFunction Base;

    public:
        JSFunction(ExecState*, NonNullPassRefPtr<Structure>, int length, const Identifier&, NativeFunction);
        JSFunction(ExecState*, NonNullPassRefPtr<FunctionExecutable>, ScopeChainNode*);
        virtual ~JSFunction();

        JSObject* construct(ExecState*, const ArgList&);
        JSValue call(ExecState*, JSValue thisValue, const ArgList&);

        void setScope(const ScopeChain& scopeChain) { setScopeChain(scopeChain); }
        ScopeChain& scope() { return scopeChain(); }

        ExecutableBase* executable() const { return m_executable.get(); }

        // To call either of these methods include Executable.h
        inline bool isHostFunction() const;
        FunctionExecutable* jsExecutable() const;

        static JS_EXPORTDATA const ClassInfo info;

        static PassRefPtr<Structure> createStructure(JSValue prototype) 
        { 
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); 
        }

        NativeFunction nativeFunction()
        {
            return *WTF::bitwise_cast<NativeFunction*>(m_data);
        }

        virtual ConstructType getConstructData(ConstructData&);
        virtual CallType getCallData(CallData&);

    protected:
        const static unsigned StructureFlags = OverridesGetOwnPropertySlot | ImplementsHasInstance | OverridesMarkChildren | OverridesGetPropertyNames | InternalFunction::StructureFlags;

    private:
        JSFunction(NonNullPassRefPtr<Structure>);

        bool isHostFunctionNonInline() const;

        virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
        virtual void getOwnPropertyNames(ExecState*, PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);
        virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);
        virtual bool deleteProperty(ExecState*, const Identifier& propertyName);

        virtual void markChildren(MarkStack&);

        virtual const ClassInfo* classInfo() const { return &info; }

        static JSValue argumentsGetter(ExecState*, const Identifier&, const PropertySlot&);
        static JSValue callerGetter(ExecState*, const Identifier&, const PropertySlot&);
        static JSValue lengthGetter(ExecState*, const Identifier&, const PropertySlot&);

        RefPtr<ExecutableBase> m_executable;
        ScopeChain& scopeChain()
        {
            ASSERT(!isHostFunctionNonInline());
            return *WTF::bitwise_cast<ScopeChain*>(m_data);
        }
        void clearScopeChain()
        {
            ASSERT(!isHostFunctionNonInline());
            new (m_data) ScopeChain(NoScopeChain());
        }
        void setScopeChain(ScopeChainNode* sc)
        {
            ASSERT(!isHostFunctionNonInline());
            new (m_data) ScopeChain(sc);
        }
        void setScopeChain(const ScopeChain& sc)
        {
            ASSERT(!isHostFunctionNonInline());
            *WTF::bitwise_cast<ScopeChain*>(m_data) = sc;
        }
        void setNativeFunction(NativeFunction func)
        {
            *WTF::bitwise_cast<NativeFunction*>(m_data) = func;
        }
        unsigned char m_data[sizeof(void*)];
    };

    JSFunction* asFunction(JSValue);

    inline JSFunction* asFunction(JSValue value)
    {
        ASSERT(asObject(value)->inherits(&JSFunction::info));
        return static_cast<JSFunction*>(asObject(value));
    }

} // namespace JSC

#endif // JSFunction_h
