

#ifndef WebScriptWorld_h
#define WebScriptWorld_h

#include <WebCore/COMPtr.h>

namespace WebCore {
    class DOMWrapperWorld;
}

class WebScriptWorld : public Noncopyable, public IWebScriptWorld {
public:
    static WebScriptWorld* standardWorld();
    static COMPtr<WebScriptWorld> createInstance();

    static COMPtr<WebScriptWorld> findOrCreateWorld(WebCore::DOMWrapperWorld*);

    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    WebCore::DOMWrapperWorld* world() const { return m_world.get(); }

private:
    static COMPtr<WebScriptWorld> createInstance(PassRefPtr<WebCore::DOMWrapperWorld>);

    WebScriptWorld(PassRefPtr<WebCore::DOMWrapperWorld>);
    ~WebScriptWorld();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** ppvObject);
    virtual HRESULT STDMETHODCALLTYPE standardWorld(IWebScriptWorld**);
    virtual HRESULT STDMETHODCALLTYPE scriptWorldForGlobalContext(JSGlobalContextRef, IWebScriptWorld**);

    ULONG m_refCount;
    RefPtr<WebCore::DOMWrapperWorld> m_world;
};

#endif // WebScriptWorld_h
