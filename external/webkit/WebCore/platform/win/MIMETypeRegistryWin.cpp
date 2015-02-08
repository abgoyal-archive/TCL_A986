

#include "config.h"
#include "MIMETypeRegistry.h"

#include <shlwapi.h>
#include <wtf/HashMap.h>

namespace WebCore 
{

static String mimeTypeForExtension(const String& extension)
{
    String ext = "." + extension;
    WCHAR contentTypeStr[256];
    DWORD contentTypeStrLen = sizeof(contentTypeStr);
    DWORD keyType;

    HRESULT result = SHGetValue(HKEY_CLASSES_ROOT, ext.charactersWithNullTermination(), L"Content Type", &keyType, (LPVOID)contentTypeStr, &contentTypeStrLen);

    if (result == ERROR_SUCCESS && keyType == REG_SZ) 
        return String(contentTypeStr, contentTypeStrLen / sizeof(contentTypeStr[0]) - 1);

    return String();
}
   
String MIMETypeRegistry::getPreferredExtensionForMIMEType(const String& type)
{
    String path = "MIME\\Database\\Content Type\\" + type;
    WCHAR extStr[MAX_PATH];
    DWORD extStrLen = sizeof(extStr);
    DWORD keyType;

    HRESULT result = SHGetValueW(HKEY_CLASSES_ROOT, path.charactersWithNullTermination(), L"Extension", &keyType, (LPVOID)extStr, &extStrLen);

    if (result == ERROR_SUCCESS && keyType == REG_SZ) 
        return String(extStr + 1, extStrLen / sizeof(extStr[0]) - 2);

    return String();
}

String MIMETypeRegistry::getMIMETypeForExtension(const String &ext)
{
    if (ext.isEmpty())
        return String();

    static HashMap<String, String> mimetypeMap;
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
        mimetypeMap.add("wml", "text/vnd.wap.wml");
        mimetypeMap.add("wmlc", "application/vnd.wap.wmlc");
    }
    String result = mimetypeMap.get(ext);
    if (result.isEmpty()) {
        result = mimeTypeForExtension(ext);
        if (!result.isEmpty())
            mimetypeMap.add(ext, result);
    }
    return result;
}

}
