

#import <WebCore/DocumentLoader.h>
#import <wtf/RetainPtr.h>
#import <wtf/HashSet.h>

@class WebDataSource;
@class WebView;

namespace WebCore {
    class ResourceRequest;
}

class WebDocumentLoaderMac : public WebCore::DocumentLoader {
public:
    static PassRefPtr<WebDocumentLoaderMac> create(const WebCore::ResourceRequest& request, const WebCore::SubstituteData& data)
    {
        return adoptRef(new WebDocumentLoaderMac(request, data));
    }

    void setDataSource(WebDataSource *, WebView*);
    void detachDataSource();
    WebDataSource *dataSource() const;

    void increaseLoadCount(unsigned long identifier);
    void decreaseLoadCount(unsigned long identifier);

private:
    WebDocumentLoaderMac(const WebCore::ResourceRequest&, const WebCore::SubstituteData&);

    virtual void attachToFrame();
    virtual void detachFromFrame();

    void retainDataSource();
    void releaseDataSource();

    WebDataSource *m_dataSource;
    bool m_isDataSourceRetained;
    RetainPtr<id> m_resourceLoadDelegate;
    RetainPtr<id> m_downloadDelegate;
    HashSet<unsigned long> m_loadingResources;
};
