

#include "config.h"
#include "MIMETypeRegistry.h"

#include "PlatformString.h"


namespace WebCore {
struct ExtensionMap {
    const char* extension;
    const char* mimeType;
};

static const ExtensionMap extensionMap [] = {
    { "bmp", "image/bmp" },
    { "gif", "image/gif" },
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
    { 0, 0 }
};

// FIXME: Use the Haiku MIME registry
String MIMETypeRegistry::getMIMETypeForExtension(const String &ext)
{
    String str = ext.lower();
    const ExtensionMap *extMap = extensionMap;
    while (extMap->extension) {
        if (str == extMap->extension)
            return extMap->mimeType;
        ++extMap;
    }
    // unknown, let's just assume plain text
    return "text/plain";
}
} // namespace WebCore

