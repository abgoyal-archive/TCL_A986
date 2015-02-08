

#ifndef BINDINGS_OBJC_INSTANCE_H_
#define BINDINGS_OBJC_INSTANCE_H_

#include "objc_class.h"
#include "objc_utility.h"

namespace JSC {

namespace Bindings {

class ObjcClass;

class ObjcInstance : public Instance {
public:
    static PassRefPtr<ObjcInstance> create(ObjectStructPtr, PassRefPtr<RootObject>);
    virtual ~ObjcInstance();
    
    static void setGlobalException(NSString*, JSGlobalObject* exceptionEnvironment = 0); // A null exceptionEnvironment means the exception should propogate to any execution environment.

    virtual Class* getClass() const;
        
    virtual JSValue valueOf(ExecState*) const;
    virtual JSValue defaultValue(ExecState*, PreferredPrimitiveType) const;
    
    virtual JSValue invokeMethod(ExecState*, const MethodList&, const ArgList&);
    virtual bool supportsInvokeDefaultMethod() const;
    virtual JSValue invokeDefaultMethod(ExecState*, const ArgList&);

    JSValue getValueOfUndefinedField(ExecState*, const Identifier&) const;
    virtual bool setValueOfUndefinedField(ExecState*, const Identifier&, JSValue);

    ObjectStructPtr getObject() const { return _instance.get(); }
    
    JSValue stringValue(ExecState*) const;
    JSValue numberValue(ExecState*) const;
    JSValue booleanValue() const;

protected:
    virtual void virtualBegin();
    virtual void virtualEnd();

private:
    static void moveGlobalExceptionToExecState(ExecState*);

    ObjcInstance(ObjectStructPtr, PassRefPtr<RootObject>);
    
    RetainPtr<ObjectStructPtr> _instance;
    mutable ObjcClass *_class;
    ObjectStructPtr _pool;
    int _beginCount;
};

} // namespace Bindings

} // namespace JSC

#endif // BINDINGS_OBJC_INSTANCE_H_
