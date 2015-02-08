

#ifndef CSSStyleSheet_h
#define CSSStyleSheet_h

#include "CSSRuleList.h"
#include "StyleSheet.h"

namespace WebCore {

class CSSNamespace;
class CSSParser;
class CSSRule;
class DocLoader;
class Document;

typedef int ExceptionCode;

class CSSStyleSheet : public StyleSheet {
public:
    static PassRefPtr<CSSStyleSheet> create()
    {
        return adoptRef(new CSSStyleSheet(static_cast<CSSStyleSheet*>(0), String(), KURL(), String()));
    }
    static PassRefPtr<CSSStyleSheet> create(Node* ownerNode)
    {
        return adoptRef(new CSSStyleSheet(ownerNode, String(), KURL(), String()));
    }
    static PassRefPtr<CSSStyleSheet> create(Node* ownerNode, const String& originalURL, const KURL& finalURL)
    {
        return adoptRef(new CSSStyleSheet(ownerNode, originalURL, finalURL, String()));
    }
    static PassRefPtr<CSSStyleSheet> create(Node* ownerNode, const String& originalURL, const KURL& finalURL, const String& charset)
    {
        return adoptRef(new CSSStyleSheet(ownerNode, originalURL, finalURL, charset));
    }
    static PassRefPtr<CSSStyleSheet> create(CSSRule* ownerRule, const String& originalURL, const KURL& finalURL, const String& charset)
    {
        return adoptRef(new CSSStyleSheet(ownerRule, originalURL, finalURL, charset));
    }
    static PassRefPtr<CSSStyleSheet> createInline(Node* ownerNode, const KURL& finalURL)
    {
        return adoptRef(new CSSStyleSheet(ownerNode, finalURL.string(), finalURL, String()));
    }

    virtual ~CSSStyleSheet();

    CSSRule* ownerRule() const;
    PassRefPtr<CSSRuleList> cssRules(bool omitCharsetRules = false);
    unsigned insertRule(const String& rule, unsigned index, ExceptionCode&);
    void deleteRule(unsigned index, ExceptionCode&);

    // IE Extensions
    PassRefPtr<CSSRuleList> rules() { return cssRules(true); }
    int addRule(const String& selector, const String& style, int index, ExceptionCode&);
    int addRule(const String& selector, const String& style, ExceptionCode&);
    void removeRule(unsigned index, ExceptionCode& ec) { deleteRule(index, ec); }

    void addNamespace(CSSParser*, const AtomicString& prefix, const AtomicString& uri);
    const AtomicString& determineNamespace(const AtomicString& prefix);

    virtual void styleSheetChanged();

    virtual bool parseString(const String&, bool strict = true);

    virtual bool isLoading();

    virtual void checkLoaded();

    Document* doc() { return m_doc; }

    const String& charset() const { return m_charset; }

    bool loadCompleted() const { return m_loadCompleted; }

    virtual KURL completeURL(const String& url) const;
    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&);

    void setStrictParsing(bool b) { m_strictParsing = b; }
    bool useStrictParsing() const { return m_strictParsing; }

    void setIsUserStyleSheet(bool b) { m_isUserStyleSheet = b; }
    bool isUserStyleSheet() const { return m_isUserStyleSheet; }
    void setHasSyntacticallyValidCSSHeader(bool b) { m_hasSyntacticallyValidCSSHeader = b; }
    bool hasSyntacticallyValidCSSHeader() const { return m_hasSyntacticallyValidCSSHeader; }

private:
    CSSStyleSheet(Node* ownerNode, const String& originalURL, const KURL& finalURL, const String& charset);
    CSSStyleSheet(CSSStyleSheet* parentSheet, const String& originalURL, const KURL& finalURL, const String& charset);
    CSSStyleSheet(CSSRule* ownerRule, const String& originalURL, const KURL& finalURL, const String& charset);

    virtual bool isCSSStyleSheet() const { return true; }
    virtual String type() const { return "text/css"; }

    Document* m_doc;
    CSSNamespace* m_namespaces;
    String m_charset;
    bool m_loadCompleted : 1;
    bool m_strictParsing : 1;
    bool m_isUserStyleSheet : 1;
    bool m_hasSyntacticallyValidCSSHeader : 1;
};

} // namespace

#endif
