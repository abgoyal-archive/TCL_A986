

#ifndef WebDataSourceImpl_h
#define WebDataSourceImpl_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebDataSource.h"

#include "DocumentLoader.h"
#include "KURL.h"

#include "WebPluginLoadObserver.h"
#include "WrappedResourceRequest.h"
#include "WrappedResourceResponse.h"

#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/Vector.h>

namespace WebKit {

class WebPluginLoadObserver;

class WebDataSourceImpl : public WebCore::DocumentLoader, public WebDataSource {
public:
    static PassRefPtr<WebDataSourceImpl> create(const WebCore::ResourceRequest&,
                                                const WebCore::SubstituteData&);

    static WebDataSourceImpl* fromDocumentLoader(WebCore::DocumentLoader* loader)
    {
        return static_cast<WebDataSourceImpl*>(loader);
    }

    // WebDataSource methods:
    virtual const WebURLRequest& originalRequest() const;
    virtual const WebURLRequest& request() const;
    virtual const WebURLResponse& response() const;
    virtual bool hasUnreachableURL() const;
    virtual WebURL unreachableURL() const;
    virtual void redirectChain(WebVector<WebURL>&) const;
    virtual WebString pageTitle() const;
    virtual WebNavigationType navigationType() const;
    virtual double triggeringEventTime() const;
    virtual ExtraData* extraData() const;
    virtual void setExtraData(ExtraData*);
    virtual WebApplicationCacheHost* applicationCacheHost();

    static WebNavigationType toWebNavigationType(WebCore::NavigationType type);

    bool hasRedirectChain() const { return !m_redirectChain.isEmpty(); }
    const WebCore::KURL& endOfRedirectChain() const;
    void clearRedirectChain();
    void appendRedirect(const WebCore::KURL& url);

    PassOwnPtr<WebPluginLoadObserver> releasePluginLoadObserver() { return m_pluginLoadObserver.release(); }
    static void setNextPluginLoadObserver(PassOwnPtr<WebPluginLoadObserver>);

private:
    WebDataSourceImpl(const WebCore::ResourceRequest&, const WebCore::SubstituteData&);
    ~WebDataSourceImpl();

    // Mutable because the const getters will magically sync these to the
    // latest version from WebKit.
    mutable WrappedResourceRequest m_originalRequestWrapper;
    mutable WrappedResourceRequest m_requestWrapper;
    mutable WrappedResourceResponse m_responseWrapper;

    // Lists all intermediate URLs that have redirected for the current provisional load.
    // See WebFrameLoaderClient::dispatchDidReceiveServerRedirectForProvisionalLoad for a
    // description of who modifies this when to keep it up to date.
    Vector<WebCore::KURL> m_redirectChain;

    OwnPtr<ExtraData> m_extraData;
    OwnPtr<WebPluginLoadObserver> m_pluginLoadObserver;

    static WebPluginLoadObserver* m_nextPluginLoadObserver;
};

} // namespace WebKit

#endif  // WebDataSourceImpl_h
