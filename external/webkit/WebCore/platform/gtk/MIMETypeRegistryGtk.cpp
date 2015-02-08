

#include "config.h"
#include "MIMETypeRegistry.h"

namespace WebCore {

struct ExtensionMap {
    const char* extension;
    const char* mimeType;
};

static const ExtensionMap extensionMap [] = {
    { "bmp", "image/bmp" },
    { "css", "text/css" },
    { "gif", "image/gif" },
    { "htm", "text/html" },
    { "html", "text/html" },
    { "ico", "image/x-icon" },
    { "jpeg", "image/jpeg" },
    { "jpg", "image/jpeg" },
    { "js", "application/x-javascript" },
    { "pdf", "application/pdf" },
    { "png", "image/png" },
    { "rss", "application/rss+xml" },
    { "svg", "image/svg+xml" },
    { "text", "text/plain" },
    { "txt", "text/plain" },
    { "xbm", "image/x-xbitmap" },
    { "xml", "text/xml" },
    { "xsl", "text/xsl" },
    { "xhtml", "application/xhtml+xml" },
    { "wml", "text/vnd.wap.wml" },
    { "wmlc", "application/vnd.wap.wmlc" },
    { 0, 0 }
};

String MIMETypeRegistry::getMIMETypeForExtension(const String &ext)
{
    String s = ext.lower();
    const ExtensionMap *e = extensionMap;
    while (e->extension) {
        if (s == e->extension)
            return e->mimeType;
        ++e;
    }

    return String();
}

}
