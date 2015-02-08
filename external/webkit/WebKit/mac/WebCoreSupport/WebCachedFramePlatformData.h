

#import <objc/objc-runtime.h>
#import <WebCore/CachedFramePlatformData.h>
#import <wtf/RetainPtr.h>

class WebCachedFramePlatformData : public WebCore::CachedFramePlatformData {
public:
    WebCachedFramePlatformData(id webDocumentView) : m_webDocumentView(webDocumentView) { }
    
    virtual void clear() { objc_msgSend(m_webDocumentView.get(), @selector(closeIfNotCurrentView)); }

    id webDocumentView() { return m_webDocumentView.get(); }
private:
    RetainPtr<id> m_webDocumentView;
};

