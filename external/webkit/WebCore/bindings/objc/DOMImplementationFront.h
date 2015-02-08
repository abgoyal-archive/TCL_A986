

#ifndef DOMImplementationFront_h
#define DOMImplementationFront_h

// FIXME: This source file exists to work around a problem that occurs trying
// to mix DOMImplementation and WebCore::DOMImplementation in DOM.mm.
// It seems to affect only older versions of gcc. Once the buildbot is upgraded,
// we should consider increasing the minimum required version of gcc to build
// WebCore, and rolling the change that introduced this file back.

#include <wtf/Forward.h>

namespace WebCore {

class CSSStyleSheet;
class Document;
class DocumentType;
class HTMLDocument;
class JSDOMImplementation;
class String;

typedef int ExceptionCode;

class DOMImplementationFront {
public:
    void ref();
    void deref();
    bool hasFeature(const String& feature, const String& version) const;
    PassRefPtr<DocumentType> createDocumentType(const String& qualifiedName, const String& publicId, const String& systemId, ExceptionCode&);
    PassRefPtr<Document> createDocument(const String& namespaceURI, const String& qualifiedName, DocumentType*, ExceptionCode&);
    DOMImplementationFront* getInterface(const String& feature);
    PassRefPtr<CSSStyleSheet> createCSSStyleSheet(const String& title, const String& media, ExceptionCode&);
    PassRefPtr<HTMLDocument> createHTMLDocument(const String& title);
};

DOMImplementationFront* implementationFront(Document*);
DOMImplementationFront* implementationFront(JSDOMImplementation*);

} // namespace WebCore

#endif // DOMImplementationFront_h
