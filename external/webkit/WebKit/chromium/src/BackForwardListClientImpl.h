

#ifndef BackForwardListClientImpl_h
#define BackForwardListClientImpl_h

#include "BackForwardList.h"

namespace WebKit {
class WebViewImpl;

extern const char backForwardNavigationScheme[];

class BackForwardListClientImpl : public WebCore::BackForwardListClient {
public:
    BackForwardListClientImpl(WebViewImpl* webview);
    ~BackForwardListClientImpl();

    void setCurrentHistoryItem(WebCore::HistoryItem* item);
    WebCore::HistoryItem* previousHistoryItem() const;

private:
    // WebCore::BackForwardListClient methods:
    virtual void addItem(PassRefPtr<WebCore::HistoryItem>);
    virtual void goToItem(WebCore::HistoryItem*);
    virtual WebCore::HistoryItem* currentItem();
    virtual WebCore::HistoryItem* itemAtIndex(int index);
    virtual int backListCount();
    virtual int forwardListCount();
    virtual void close();

    WebViewImpl* m_webView;

    RefPtr<WebCore::HistoryItem> m_previousItem;
    RefPtr<WebCore::HistoryItem> m_currentItem;

    // The last history item that was accessed via itemAtIndex().  We keep track
    // of this until goToItem() is called, so we can track the navigation.
    RefPtr<WebCore::HistoryItem> m_pendingHistoryItem;
};

} // namespace WebKit

#endif
