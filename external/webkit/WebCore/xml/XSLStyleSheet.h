

#ifndef XSLStyleSheet_h
#define XSLStyleSheet_h

#if ENABLE(XSLT)

#include "StyleSheet.h"

#if !USE(QXMLQUERY)
#include <libxml/parser.h>
#include <libxslt/transform.h>
#endif

#include <wtf/PassRefPtr.h>

namespace WebCore {

class DocLoader;
class Document;
class XSLImportRule;
    
class XSLStyleSheet : public StyleSheet {
public:
#if !USE(QXMLQUERY)
    static PassRefPtr<XSLStyleSheet> create(XSLImportRule* parentImport, const String& originalURL, const KURL& finalURL)
    {
        return adoptRef(new XSLStyleSheet(parentImport, originalURL, finalURL));
    }
#endif
    static PassRefPtr<XSLStyleSheet> create(Node* parentNode, const String& originalURL, const KURL& finalURL)
    {
        return adoptRef(new XSLStyleSheet(parentNode, originalURL, finalURL, false));
    }
    static PassRefPtr<XSLStyleSheet> createInline(Node* parentNode, const KURL& finalURL)
    {
        return adoptRef(new XSLStyleSheet(parentNode, finalURL.string(), finalURL, true));
    }

    virtual ~XSLStyleSheet();
    
    virtual bool isXSLStyleSheet() const { return true; }

    virtual String type() const { return "text/xml"; }

    virtual bool parseString(const String &string, bool strict = true);
    
    virtual bool isLoading();
    virtual void checkLoaded();

    void loadChildSheets();
    void loadChildSheet(const String& href);

    DocLoader* docLoader();

    Document* ownerDocument() { return m_ownerDocument; }
    void setParentStyleSheet(XSLStyleSheet* parent);

#if USE(QXMLQUERY)
    String sheetString() const { return m_sheetString; }
#else
    xmlDocPtr document();
    xsltStylesheetPtr compileStyleSheet();
    xmlDocPtr locateStylesheetSubResource(xmlDocPtr parentDoc, const xmlChar* uri);
#endif

    void clearDocuments();

    void markAsProcessed();
    bool processed() const { return m_processed; }

private:
    XSLStyleSheet(Node* parentNode, const String& originalURL, const KURL& finalURL, bool embedded);
#if !USE(QXMLQUERY)
    XSLStyleSheet(XSLImportRule* parentImport, const String& originalURL, const KURL& finalURL);
#endif

    Document* m_ownerDocument;
    bool m_embedded;
    bool m_processed;

#if USE(QXMLQUERY)
    String m_sheetString;
#else
    xmlDocPtr m_stylesheetDoc;
    bool m_stylesheetDocTaken;
#endif
    
    XSLStyleSheet* m_parentStyleSheet;
};

} // namespace WebCore

#endif // ENABLE(XSLT)

#endif // XSLStyleSheet_h
