

#ifndef WebWorkersPrivate_h
#define WebWorkersPrivate_h

#include "WebKit.h"

class WebWorkersPrivate : public IWebWorkersPrivate {
public:
    static WebWorkersPrivate* createInstance();
private:
    WebWorkersPrivate();
    ~WebWorkersPrivate();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebWorkersPrivate
    virtual HRESULT STDMETHODCALLTYPE workerThreadCount(UINT*);

private:
    ULONG m_refCount;
};

#endif // WebWorkersPrivate_h
