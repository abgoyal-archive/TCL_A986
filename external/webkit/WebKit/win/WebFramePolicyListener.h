

#ifndef WebFramePolicyListener_h
#define WebFramePolicyListener_h

#include "WebKit.h"

#include <WTF/PassRefPtr.h>
#include <WTF/RefPtr.h>

#pragma warning(push, 0)
#include <WebCore/FrameLoaderTypes.h>
#pragma warning(pop)

namespace WebCore {
    class Frame;
}

class WebFramePolicyListener : public IWebPolicyDecisionListener, public IWebFormSubmissionListener {
public:
    static WebFramePolicyListener* createInstance(PassRefPtr<WebCore::Frame>);
protected:
    WebFramePolicyListener(PassRefPtr<WebCore::Frame>);
    ~WebFramePolicyListener();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebPolicyDecisionListener
    virtual HRESULT STDMETHODCALLTYPE use(void);
    virtual HRESULT STDMETHODCALLTYPE download(void);
    virtual HRESULT STDMETHODCALLTYPE ignore(void);

    // IWebFormSubmissionListener
    virtual HRESULT STDMETHODCALLTYPE continueSubmit(void);

    // WebFramePolicyListener
    void receivedPolicyDecision(WebCore::PolicyAction);
    void invalidate();
private:
    ULONG m_refCount;
    RefPtr<WebCore::Frame> m_frame;
};

#endif
