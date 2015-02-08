

#ifndef CSSImportRule_h
#define CSSImportRule_h

#include "CSSRule.h"
#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include "MediaList.h"
#include "PlatformString.h"

namespace WebCore {

class CachedCSSStyleSheet;
class MediaList;

class CSSImportRule : public CSSRule, private CachedResourceClient {
public:
    static PassRefPtr<CSSImportRule> create(CSSStyleSheet* parent, const String& href, PassRefPtr<MediaList> media)
    {
        return adoptRef(new CSSImportRule(parent, href, media));
    }

    virtual ~CSSImportRule();

    String href() const { return m_strHref; }
    MediaList* media() const { return m_lstMedia.get(); }
    CSSStyleSheet* styleSheet() const { return m_styleSheet.get(); }

    virtual String cssText() const;

    // Not part of the CSSOM
    bool isLoading() const;

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>& urls);

private:
    CSSImportRule(CSSStyleSheet* parent, const String& href, PassRefPtr<MediaList>);

    virtual bool isImportRule() { return true; }
    virtual void insertedIntoParent();

    // from CSSRule
    virtual unsigned short type() const { return IMPORT_RULE; }

    // from CachedResourceClient
    virtual void setCSSStyleSheet(const String& href, const KURL& baseURL, const String& charset, const CachedCSSStyleSheet*);

#ifdef ANDROID_INSTRUMENT
    // Overridden to resolve the ambiguous
    void* operator new(size_t size);
    void* operator new[](size_t size);
    void operator delete(void* p, size_t size);
    void operator delete[](void* p, size_t size);
#endif

    String m_strHref;
    RefPtr<MediaList> m_lstMedia;
    RefPtr<CSSStyleSheet> m_styleSheet;
    CachedResourceHandle<CachedCSSStyleSheet> m_cachedSheet;
    bool m_loading;
};

} // namespace WebCore

#endif // CSSImportRule_h
