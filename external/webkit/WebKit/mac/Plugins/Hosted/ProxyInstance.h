

#if USE(PLUGIN_HOST_PROCESS)

#ifndef ProxyInstance_h
#define ProxyInstance_h

#import "NetscapePluginInstanceProxy.h"
#import "WebKitPluginHostTypes.h"
#import <WebCore/Bridge.h>
#import <WebCore/runtime_root.h>
#import <wtf/OwnPtr.h>

namespace WebKit {

class ProxyClass;
    
class ProxyInstance : public JSC::Bindings::Instance {
public:
    static PassRefPtr<ProxyInstance> create(PassRefPtr<JSC::Bindings::RootObject> rootObject, NetscapePluginInstanceProxy* instanceProxy, uint32_t objectID)
    {
        return adoptRef(new ProxyInstance(rootObject, instanceProxy, objectID));
    }
    ~ProxyInstance();

    JSC::Bindings::MethodList methodsNamed(const JSC::Identifier&);
    JSC::Bindings::Field* fieldNamed(const JSC::Identifier&);

    JSC::JSValue fieldValue(JSC::ExecState*, const JSC::Bindings::Field*) const;
    void setFieldValue(JSC::ExecState*, const JSC::Bindings::Field*, JSC::JSValue) const;
    
    void invalidate();
    
    uint32_t objectID() const { return m_objectID; }
    
private:
    ProxyInstance(PassRefPtr<JSC::Bindings::RootObject>, NetscapePluginInstanceProxy*, uint32_t objectID);
    
    virtual JSC::Bindings::Class *getClass() const;

    virtual JSC::JSValue invokeMethod(JSC::ExecState*, const JSC::Bindings::MethodList&, const JSC::ArgList& args);

    virtual bool supportsInvokeDefaultMethod() const;
    virtual JSC::JSValue invokeDefaultMethod(JSC::ExecState*, const JSC::ArgList&);

    virtual bool supportsConstruct() const;
    virtual JSC::JSValue invokeConstruct(JSC::ExecState*, const JSC::ArgList&);

    virtual JSC::JSValue defaultValue(JSC::ExecState*, JSC::PreferredPrimitiveType) const;
    virtual JSC::JSValue valueOf(JSC::ExecState*) const;
    
    virtual void getPropertyNames(JSC::ExecState*, JSC::PropertyNameArray&);

    JSC::JSValue stringValue(JSC::ExecState*) const;
    JSC::JSValue numberValue(JSC::ExecState*) const;
    JSC::JSValue booleanValue() const;
    
    JSC::JSValue invoke(JSC::ExecState*, InvokeType, uint64_t identifier, const JSC::ArgList& args);
    
    template <typename T>
    std::auto_ptr<T> waitForReply(uint32_t requestID) const {
        std::auto_ptr<T> reply = m_instanceProxy->waitForReply<T>(requestID);
        
        // If the instance proxy was invalidated, just return a null reply.
        if (!m_instanceProxy)
            return std::auto_ptr<T>();
        
        return reply;
    }

    NetscapePluginInstanceProxy* m_instanceProxy;
    uint32_t m_objectID;
    JSC::Bindings::FieldMap m_fields;
    JSC::Bindings::MethodMap m_methods;
};
    
}

#endif // ProxyInstance_h
#endif // USE(PLUGIN_HOST_PROCESS)
