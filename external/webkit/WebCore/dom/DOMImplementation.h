

#ifndef DOMImplementation_h
#define DOMImplementation_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class CSSStyleSheet;
class Document;
class DocumentType;
class Frame;
class HTMLDocument;
class String;

typedef int ExceptionCode;

class DOMImplementation : public RefCounted<DOMImplementation> {
public:
    static PassRefPtr<DOMImplementation> create() { return adoptRef(new DOMImplementation); }

    // DOM methods & attributes for DOMImplementation
    static bool hasFeature(const String& feature, const String& version);
    static PassRefPtr<DocumentType> createDocumentType(const String& qualifiedName, const String& publicId, const String &systemId, ExceptionCode&);
    static PassRefPtr<Document> createDocument(const String& namespaceURI, const String& qualifiedName, DocumentType*, ExceptionCode&);

    DOMImplementation* getInterface(const String& feature);

    // From the DOMImplementationCSS interface
    static PassRefPtr<CSSStyleSheet> createCSSStyleSheet(const String& title, const String& media, ExceptionCode&);

    // From the HTMLDOMImplementation interface
    static PassRefPtr<HTMLDocument> createHTMLDocument(const String& title);

    // Other methods (not part of DOM)
    static PassRefPtr<Document> createDocument(const String& MIMEType, Frame*, bool inViewSourceMode);
    static PassRefPtr<Document> createDocument(Frame*);
    static PassRefPtr<HTMLDocument> createHTMLDocument(Frame*);

    static bool isXMLMIMEType(const String& MIMEType);
    static bool isTextMIMEType(const String& MIMEType);

private:
    DOMImplementation() { }
};

} //namespace

#endif
