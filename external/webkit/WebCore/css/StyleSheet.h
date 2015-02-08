

#ifndef StyleSheet_h
#define StyleSheet_h

#include "KURLHash.h"
#include "PlatformString.h"
#include "StyleList.h"
#include <wtf/ListHashSet.h>

namespace WebCore {

class CachedCSSStyleSheet;
class MediaList;
class Node;

class StyleSheet : public StyleList {
public:
    virtual ~StyleSheet();

    bool disabled() const { return m_disabled; }
    void setDisabled(bool disabled) { m_disabled = disabled; styleSheetChanged(); }

    Node* ownerNode() const { return m_parentNode; }
    StyleSheet *parentStyleSheet() const;

    // Note that href is the URL that started the redirect chain that led to
    // this style sheet. This property probably isn't useful for much except
    // the JavaScript binding (which needs to use this value for security).
    const String& href() const { return m_originalURL; }

    void setFinalURL(const KURL& finalURL) { m_finalURL = finalURL; }
    const KURL& finalURL() const { return m_finalURL; }

    const String& title() const { return m_strTitle; }
    void setTitle(const String& s) { m_strTitle = s; }
    MediaList* media() const { return m_media.get(); }
    void setMedia(PassRefPtr<MediaList>);

    virtual String type() const = 0;
    virtual bool isLoading() = 0;
    virtual void styleSheetChanged() { }

    virtual KURL completeURL(const String& url) const;
    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&) { }

    virtual bool parseString(const String&, bool strict = true) = 0;

protected:
    StyleSheet(Node* ownerNode, const String& href, const KURL& finalURL);
    StyleSheet(StyleSheet* parentSheet, const String& href, const KURL& finalURL);
    StyleSheet(StyleBase* owner, const String& href, const KURL& finalURL);

private:
    virtual bool isStyleSheet() const { return true; }

    Node* m_parentNode;
    String m_originalURL;
    KURL m_finalURL;
    String m_strTitle;
    RefPtr<MediaList> m_media;
    bool m_disabled;
};

} // namespace

#endif
