

#include "config.h"
#include "BackForwardListClientImpl.h"

#include "HistoryItem.h"
#include "WebViewClient.h"
#include "WebViewImpl.h"

using namespace WebCore;

namespace WebKit {

const char backForwardNavigationScheme[] = "chrome-back-forward";

BackForwardListClientImpl::BackForwardListClientImpl(WebViewImpl* webView)
    : m_webView(webView)
{
}

BackForwardListClientImpl::~BackForwardListClientImpl()
{
}

void BackForwardListClientImpl::setCurrentHistoryItem(HistoryItem* item)
{
    m_previousItem = m_currentItem;
    m_currentItem = item;
}

HistoryItem* BackForwardListClientImpl::previousHistoryItem() const
{
    return m_previousItem.get();
}

void BackForwardListClientImpl::addItem(PassRefPtr<HistoryItem> item)
{
    m_previousItem = m_currentItem;
    m_currentItem = item;

    // If WebCore adds a new HistoryItem, it means this is a new navigation (ie,
    // not a reload or back/forward).
    m_webView->observeNewNavigation();

    if (m_webView->client())
        m_webView->client()->didAddHistoryItem();
}

void BackForwardListClientImpl::goToItem(HistoryItem* item)
{
    m_previousItem = m_currentItem;
    m_currentItem = item;

    if (m_pendingHistoryItem == item)
        m_pendingHistoryItem = 0;
}

HistoryItem* BackForwardListClientImpl::currentItem()
{
    return m_currentItem.get();
}

HistoryItem* BackForwardListClientImpl::itemAtIndex(int index)
{
    if (!m_webView->client() || index > forwardListCount() || -index > backListCount())
        return 0;

    // Since we don't keep the entire back/forward list, we have no way to
    // properly implement this method.  We return a dummy entry instead that we
    // intercept in our FrameLoaderClient implementation in case WebCore asks
    // to navigate to this HistoryItem.

    // FIXME: We should change WebCore to handle history.{back,forward,go}
    // differently.  It should perhaps just ask the FrameLoaderClient to
    // perform those navigations.

    String url_string = String::format(
        "%s://go/%d", backForwardNavigationScheme, index);

    m_pendingHistoryItem =
        HistoryItem::create(url_string, String(), 0.0);
    return m_pendingHistoryItem.get();
}

int BackForwardListClientImpl::backListCount()
{
    if (!m_webView->client())
        return 0;

    return m_webView->client()->historyBackListCount();
}

int BackForwardListClientImpl::forwardListCount()
{
    if (!m_webView->client())
        return 0;

    return m_webView->client()->historyForwardListCount();
}

void BackForwardListClientImpl::close()
{
    m_currentItem = 0;
    m_previousItem = 0;
    m_pendingHistoryItem = 0;
}

} // namespace WebKit
