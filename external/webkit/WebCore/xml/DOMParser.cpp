

#include "config.h"
#include "DOMParser.h"

#include "DOMImplementation.h"
#include "PlatformString.h"

namespace WebCore {
    
PassRefPtr<Document> DOMParser::parseFromString(const String& str, const String& contentType)
{
    if (!DOMImplementation::isXMLMIMEType(contentType))
        return 0;

    RefPtr<Document> doc = DOMImplementation::createDocument(contentType, 0, false);

    doc->open();
    doc->write(str);
    doc->finishParsing();
    doc->close();
        
    return doc.release();
}

}
