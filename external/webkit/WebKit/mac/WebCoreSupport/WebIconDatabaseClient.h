

#import <WebCore/IconDatabaseClient.h>

namespace WebCore {
    class String;
}

class WebIconDatabaseClient : public WebCore::IconDatabaseClient {
public:
    virtual bool performImport();
    virtual void dispatchDidRemoveAllIcons();
    virtual void dispatchDidAddIconForPageURL(const WebCore::String& pageURL);
};
