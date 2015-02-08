

#ifndef WebKitClassFactory_H
#define WebKitClassFactory_H

class WebKitClassFactory : public IClassFactory
{
public:
    WebKitClassFactory(CLSID targetClass);
    ~WebKitClassFactory();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IClassFactory
    virtual HRESULT STDMETHODCALLTYPE CreateInstance(IUnknown* pUnkOuter, REFIID riid, void** ppvObject);
    virtual HRESULT STDMETHODCALLTYPE LockServer(BOOL fLock);

protected:
    ULONG m_refCount;
    CLSID m_targetClass;
};

#endif
