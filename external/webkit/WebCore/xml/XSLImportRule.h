

#ifndef XSLImportRule_h
#define XSLImportRule_h

#if ENABLE(XSLT)

#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include "StyleBase.h"
#include "XSLStyleSheet.h"

namespace WebCore {

class CachedXSLStyleSheet;

class XSLImportRule : public StyleBase, private CachedResourceClient {
public:
    static PassRefPtr<XSLImportRule> create(XSLStyleSheet* parentSheet, const String& href)
    {
        return adoptRef(new XSLImportRule(parentSheet, href));
    }

    virtual ~XSLImportRule();
    
    const String& href() const { return m_strHref; }
    XSLStyleSheet* styleSheet() const { return m_styleSheet.get(); }

    XSLStyleSheet* parentStyleSheet() const;

    bool isLoading();
    void loadSheet();
    
private:
    XSLImportRule(XSLStyleSheet* parentSheet, const String& href);

    virtual bool isImportRule() { return true; }

    // from CachedResourceClient
    virtual void setXSLStyleSheet(const String& href, const KURL& baseURL, const String& sheet);
    
    String m_strHref;
    RefPtr<XSLStyleSheet> m_styleSheet;
    CachedResourceHandle<CachedXSLStyleSheet> m_cachedSheet;
    bool m_loading;
};

} // namespace WebCore

#endif // ENABLE(XSLT)

#endif // XSLImportRule_h
