

#include "config.h"
#include "XSLStyleSheet.h"

#if ENABLE(XSLT)

#include "DOMWindow.h"
#include "Document.h"
#include "Node.h"
#include "NotImplemented.h"
#include "XSLTProcessor.h"
#include "loader.h"

namespace WebCore {

XSLStyleSheet::XSLStyleSheet(Node* parentNode, const String& originalURL, const KURL& finalURL,  bool embedded)
    : StyleSheet(parentNode, originalURL, finalURL)
    , m_ownerDocument(parentNode->document())
    , m_embedded(embedded)
{
}

XSLStyleSheet::~XSLStyleSheet()
{
}

bool XSLStyleSheet::isLoading()
{
    notImplemented();
    return false;
}

void XSLStyleSheet::checkLoaded()
{
    if (ownerNode())
        ownerNode()->sheetLoaded();
}

void XSLStyleSheet::clearDocuments()
{
    notImplemented();
}

DocLoader* XSLStyleSheet::docLoader()
{
    if (!m_ownerDocument)
        return 0;
    return m_ownerDocument->docLoader();
}

bool XSLStyleSheet::parseString(const String& string, bool)
{
    // FIXME: Fix QXmlQuery so that it allows compiling the stylesheet before setting the document
    // to be transformed. This way we could not only check if the stylesheet is correct before using it
    // but also turn XSLStyleSheet::sheetString() into XSLStyleSheet::query() that returns a QXmlQuery.

    m_sheetString = string;
    return !m_sheetString.isEmpty();
}

void XSLStyleSheet::loadChildSheets()
{
    notImplemented();
}

void XSLStyleSheet::loadChildSheet(const String&)
{
    notImplemented();
}

void XSLStyleSheet::setParentStyleSheet(XSLStyleSheet*)
{
    notImplemented();
}

void XSLStyleSheet::markAsProcessed()
{
    notImplemented();
}

} // namespace WebCore

#endif // ENABLE(XSLT)
