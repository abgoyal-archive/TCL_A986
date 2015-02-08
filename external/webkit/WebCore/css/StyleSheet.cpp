
#include "config.h"
#include "StyleSheet.h"

#include "MediaList.h"

namespace WebCore {

StyleSheet::StyleSheet(StyleSheet* parentSheet, const String& originalURL, const KURL& finalURL)
    : StyleList(parentSheet)
    , m_parentNode(0)
    , m_originalURL(originalURL)
    , m_finalURL(finalURL)
    , m_disabled(false)
{
}

StyleSheet::StyleSheet(Node* parentNode, const String& originalURL, const KURL& finalURL)
    : StyleList(0)
    , m_parentNode(parentNode)
    , m_originalURL(originalURL)
    , m_finalURL(finalURL)
    , m_disabled(false)
{
}

StyleSheet::StyleSheet(StyleBase* owner, const String& originalURL, const KURL& finalURL)
    : StyleList(owner)
    , m_parentNode(0)
    , m_originalURL(originalURL)
    , m_finalURL(finalURL)
    , m_disabled(false)
{
}

StyleSheet::~StyleSheet()
{
    if (m_media)
        m_media->setParent(0);
}

StyleSheet* StyleSheet::parentStyleSheet() const
{
    return (parent() && parent()->isStyleSheet()) ? static_cast<StyleSheet*>(parent()) : 0;
}

void StyleSheet::setMedia(PassRefPtr<MediaList> media)
{
    if (m_media)
        m_media->setParent(0);

    m_media = media;
    m_media->setParent(this);
}

KURL StyleSheet::completeURL(const String& url) const
{
    // Always return a null URL when passed a null string.
    // FIXME: Should we change the KURL constructor to have this behavior?
    // See also Document::completeURL(const String&)
    if (url.isNull())
        return KURL();
    return KURL(baseURL(), url);
}

}
