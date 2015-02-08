

#ifndef XSLTProcessor_h
#define XSLTProcessor_h

#if ENABLE(XSLT)

#include "Node.h"
#include "StringHash.h"
#include "XSLStyleSheet.h"
#include <wtf/HashMap.h>

#if !USE(QXMLQUERY)
#include <libxml/parserInternals.h>
#include <libxslt/documents.h>
#endif

namespace WebCore {

class Frame;
class Document;
class DocumentFragment;

class XSLTProcessor : public RefCounted<XSLTProcessor> {
public:
    static PassRefPtr<XSLTProcessor> create() { return adoptRef(new XSLTProcessor); }

    void setXSLStyleSheet(PassRefPtr<XSLStyleSheet> styleSheet) { m_stylesheet = styleSheet; }
    bool transformToString(Node* source, String& resultMIMEType, String& resultString, String& resultEncoding);
    PassRefPtr<Document> createDocumentFromSource(const String& source, const String& sourceEncoding, const String& sourceMIMEType, Node* sourceNode, Frame* frame);
    
    // DOM methods
    void importStylesheet(PassRefPtr<Node> style) { m_stylesheetRootNode = style; }
    PassRefPtr<DocumentFragment> transformToFragment(Node* source, Document* ouputDoc);
    PassRefPtr<Document> transformToDocument(Node* source);
    
    void setParameter(const String& namespaceURI, const String& localName, const String& value);
    String getParameter(const String& namespaceURI, const String& localName) const;
    void removeParameter(const String& namespaceURI, const String& localName);
    void clearParameters() { m_parameters.clear(); }

    void reset();

#if !USE(QXMLQUERY)
    static void parseErrorFunc(void* userData, xmlError*);
    static void genericErrorFunc(void* userData, const char* msg, ...);
    
    // Only for libXSLT callbacks
    XSLStyleSheet* xslStylesheet() const { return m_stylesheet.get(); }
#endif

    typedef HashMap<String, String> ParameterMap;

private:
    XSLTProcessor() { }

    RefPtr<XSLStyleSheet> m_stylesheet;
    RefPtr<Node> m_stylesheetRootNode;
    ParameterMap m_parameters;
};

}

#endif
#endif
