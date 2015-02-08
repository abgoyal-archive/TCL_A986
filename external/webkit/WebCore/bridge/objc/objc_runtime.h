

#ifndef KJS_BINDINGS_OBJC_RUNTIME_H
#define KJS_BINDINGS_OBJC_RUNTIME_H

#include "Bridge.h"
#include "objc_header.h"
#include <runtime/JSGlobalObject.h>
#include <wtf/RetainPtr.h>

namespace JSC {
namespace Bindings {

ClassStructPtr webScriptObjectClass();
ClassStructPtr webUndefinedClass();

class ObjcInstance;

class ObjcField : public Field {
public:
    ObjcField(IvarStructPtr);
    ObjcField(CFStringRef name);
    
    virtual JSValue valueFromInstance(ExecState*, const Instance*) const;
    virtual void setValueToInstance(ExecState*, const Instance*, JSValue) const;

private:
    IvarStructPtr _ivar;
    RetainPtr<CFStringRef> _name;
};

class ObjcMethod : public Method {
public:
    ObjcMethod() : _objcClass(0), _selector(0), _javaScriptName(0) {}
    ObjcMethod(ClassStructPtr aClass, SEL _selector);

    virtual int numParameters() const;

    NSMethodSignature *getMethodSignature() const;
    
    bool isFallbackMethod() const { return _selector == @selector(invokeUndefinedMethodFromWebScript:withArguments:); }
    void setJavaScriptName(CFStringRef n) { _javaScriptName = n; }
    CFStringRef javaScriptName() const { return _javaScriptName.get(); }
    
    SEL selector() const { return _selector; }

private:
    ClassStructPtr _objcClass;
    SEL _selector;
    RetainPtr<CFStringRef> _javaScriptName;
};

class ObjcArray : public Array {
public:
    ObjcArray(ObjectStructPtr, PassRefPtr<RootObject>);

    virtual void setValueAt(ExecState *exec, unsigned int index, JSValue aValue) const;
    virtual JSValue valueAt(ExecState *exec, unsigned int index) const;
    virtual unsigned int getLength() const;
    
    ObjectStructPtr getObjcArray() const { return _array.get(); }

    static JSValue convertObjcArrayToArray(ExecState *exec, ObjectStructPtr anObject);

private:
    RetainPtr<ObjectStructPtr> _array;
};

class ObjcFallbackObjectImp : public JSObject {
public:
    ObjcFallbackObjectImp(ExecState*, ObjcInstance*, const Identifier& propertyName);

    static const ClassInfo s_info;

    const Identifier& propertyName() const { return _item; }

    static ObjectPrototype* createPrototype(ExecState*, JSGlobalObject* globalObject)
    {
        return globalObject->objectPrototype();
    }

    static PassRefPtr<Structure> createStructure(JSValue prototype)
    {
        return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
    }

private:
    static const unsigned StructureFlags = OverridesGetOwnPropertySlot | JSObject::StructureFlags;
    virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
    virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
    virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);
    virtual CallType getCallData(CallData&);
    virtual bool deleteProperty(ExecState*, const Identifier& propertyName);
    virtual JSValue defaultValue(ExecState*, PreferredPrimitiveType) const;

    virtual bool toBoolean(ExecState*) const;

    virtual const ClassInfo* classInfo() const { return &s_info; }

    RefPtr<ObjcInstance> _instance;
    Identifier _item;
};

} // namespace Bindings
} // namespace JSC

#endif
