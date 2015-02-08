

#ifndef HTMLLinkElement_h
#define HTMLLinkElement_h

#include "CSSStyleSheet.h"
#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include "HTMLElement.h"

namespace WebCore {

class CachedCSSStyleSheet;
class KURL;

class HTMLLinkElement : public HTMLElement, public CachedResourceClient {
public:
    HTMLLinkElement(const QualifiedName&, Document*, bool createdByParser);
    ~HTMLLinkElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    bool disabled() const;
    void setDisabled(bool);

    String charset() const;
    void setCharset(const String&);

    KURL href() const;
    void setHref(const String&);

    String hreflang() const;
    void setHreflang(const String&);

    String media() const;
    void setMedia(const String&);

    String rel() const;
    void setRel(const String&);

    String rev() const;
    void setRev(const String&);

    virtual String target() const;
    void setTarget(const String&);

    String type() const;
    void setType(const String&);

    StyleSheet* sheet() const;

    // overload from HTMLElement
    virtual void parseMappedAttribute(MappedAttribute*);

    void process();

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    // from CachedResourceClient
    virtual void setCSSStyleSheet(const String& href, const KURL& baseURL, const String& charset, const CachedCSSStyleSheet* sheet);
    bool isLoading() const;
    virtual bool sheetLoaded();

    bool isAlternate() const { return m_disabledState == 0 && m_alternate; }
    bool isDisabled() const { return m_disabledState == 2; }
    bool isEnabledViaScript() const { return m_disabledState == 1; }
    bool isIcon() const { return m_isIcon; }
    
    int disabledState() { return m_disabledState; }
    void setDisabledState(bool _disabled);

    virtual bool isURLAttribute(Attribute*) const;
    
#ifdef ANDROID_APPLE_TOUCH_ICON
    static void tokenizeRelAttribute(const AtomicString& value, bool& stylesheet, bool& alternate, bool& icon, bool& touchIcon, bool& precomposedTouchIcon, bool& dnsPrefetch);
#else
    static void tokenizeRelAttribute(const AtomicString& value, bool& stylesheet, bool& alternate, bool& icon, bool& dnsPrefetch);
#endif

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    virtual void finishParsingChildren();

#ifdef ANDROID_INSTRUMENT
    // Overridden to resolve the ambiguous
    void* operator new(size_t size);
    void* operator new[](size_t size);
    void operator delete(void* p, size_t size);
    void operator delete[](void* p, size_t size);
#endif

protected:
    CachedResourceHandle<CachedCSSStyleSheet> m_cachedSheet;
    RefPtr<CSSStyleSheet> m_sheet;
    KURL m_url;
    String m_type;
    String m_media;
    int m_disabledState; // 0=unset(default), 1=enabled via script, 2=disabled
    bool m_loading;
    bool m_alternate;
    bool m_isStyleSheet;
    bool m_isIcon;
#ifdef ANDROID_APPLE_TOUCH_ICON
    bool m_isTouchIcon;
    bool m_isPrecomposedTouchIcon;
#endif
    bool m_isDNSPrefetch;
    bool m_createdByParser;
};

} //namespace

#endif
