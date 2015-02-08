

#include "config.h"
#include "MIMETypeRegistry.h"

#include <wtf/HashMap.h>
#include <windows.h>
#include <winreg.h>

namespace WebCore {

static String mimeTypeForExtension(const String& extension)
{
    String ext = "." + extension;
    WCHAR contentTypeStr[256];
    DWORD contentTypeStrLen = sizeof(contentTypeStr);
    DWORD valueType;

    HKEY key;
    String result;
    if (ERROR_SUCCESS != RegOpenKeyEx(HKEY_CLASSES_ROOT, ext.charactersWithNullTermination(), 0, 0, &key))
        return result;

    if (ERROR_SUCCESS == RegQueryValueEx(key, L"Content Type", 0, &valueType, (LPBYTE)contentTypeStr, &contentTypeStrLen) && valueType == REG_SZ)
        result = String(contentTypeStr, contentTypeStrLen / sizeof(contentTypeStr[0]) - 1);

    RegCloseKey(key);

    return result;
}

static HashMap<String, String> mimetypeMap;

static void initMIMETypeEntensionMap()
{
    if (mimetypeMap.isEmpty()) {
        //fill with initial values
        mimetypeMap.add("txt", "text/plain");
        mimetypeMap.add("pdf", "application/pdf");
        mimetypeMap.add("ps", "application/postscript");
        mimetypeMap.add("html", "text/html");
        mimetypeMap.add("htm", "text/html");
        mimetypeMap.add("xml", "text/xml");
        mimetypeMap.add("xsl", "text/xsl");
        mimetypeMap.add("js", "application/x-javascript");
        mimetypeMap.add("xhtml", "application/xhtml+xml");
        mimetypeMap.add("rss", "application/rss+xml");
        mimetypeMap.add("webarchive", "application/x-webarchive");
        mimetypeMap.add("svg", "image/svg+xml");
        mimetypeMap.add("svgz", "image/svg+xml");
        mimetypeMap.add("jpg", "image/jpeg");
        mimetypeMap.add("jpeg", "image/jpeg");
        mimetypeMap.add("png", "image/png");
        mimetypeMap.add("tif", "image/tiff");
        mimetypeMap.add("tiff", "image/tiff");
        mimetypeMap.add("ico", "image/ico");
        mimetypeMap.add("cur", "image/ico");
        mimetypeMap.add("bmp", "image/bmp");
        mimetypeMap.add("css", "text/css");
        // FIXME: Custom font works only when MIME is "text/plain"
        mimetypeMap.add("ttf", "text/plain"); // "font/ttf"
        mimetypeMap.add("otf", "text/plain"); // "font/otf"
#if ENABLE(WML)
        mimetypeMap.add("wml", "text/vnd.wap.wml");
#endif
#if ENABLE(WBXML)
        mimetypeMap.add("wbxml", "application/vnd.wap.wmlc");
#endif
    }
}

String MIMETypeRegistry::getPreferredExtensionForMIMEType(const String& type)
{
    if (type.isEmpty())
        return String();

    // Avoid conflicts with "ttf" and "otf"
    if (equalIgnoringCase(type, "text/plain"))
        return "txt";

    initMIMETypeEntensionMap();

    for (HashMap<String, String>::iterator i = mimetypeMap.begin(); i != mimetypeMap.end(); ++i) {
        if (equalIgnoringCase(i->second, type))
            return i->first;
    }

#if ENABLE(XHTMLMP)
    if (equalIgnoringCase("application/vnd.wap.xhtml+xml", type))
        return String("xml");
#endif

    return String();
}

String MIMETypeRegistry::getMIMETypeForExtension(const String &ext)
{
    if (ext.isEmpty())
        return String();

    initMIMETypeEntensionMap();

    String result = mimetypeMap.get(ext.lower());
    if (result.isEmpty()) {
        result = mimeTypeForExtension(ext);
        if (!result.isEmpty())
            mimetypeMap.add(ext, result);
    }
    return result.isEmpty() ? "unknown/unknown" : result;
}

}
