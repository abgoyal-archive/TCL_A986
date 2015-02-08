

#include "config.h"
#include "ImageSourceCG.h"

#include "StdLibExtras.h" 
#include "StringHash.h"
#include <wtf/HashMap.h>

namespace WebCore {

String MIMETypeForImageSourceType(const String& type)
{
    String mimeType;
    // FIXME: This approach of taking a UTI like public.type and giving back 
    // a MIME type like image/type will work for common image UTIs like jpeg, 
    // png, tiff, gif but won't work for UTIs like: public.jpeg-2000,
    // public.xbitmap-image, com.apple.quicktime-image, and others.
    if (int dotLocation = type.reverseFind('.'))
        mimeType = "image/" + type.substring(dotLocation + 1);
    return mimeType;
}

String preferredExtensionForImageSourceType(const String& type)
{
    if (type.isEmpty())
        return String();

    typedef HashMap<String, String> StringMap;
    DEFINE_STATIC_LOCAL(StringMap, UTIMap, ());
    if (UTIMap.isEmpty()) {
        UTIMap.add("public.html", "html");
        UTIMap.add("public.jpeg", "jpeg");
        UTIMap.add("public.jpeg-2000", "jp2");
        UTIMap.add("public.plain-text", "txt");
        UTIMap.add("public.png", "png");
        UTIMap.add("public.tiff", "tiff");
        UTIMap.add("public.xbitmap-image", "xbm");
        UTIMap.add("public.xml", "xml");
        UTIMap.add("com.adobe.illustrator.ai-image", "ai");
        UTIMap.add("com.adobe.pdf", "pdf");
        UTIMap.add("com.adobe.photoshop-image", "psd");
        UTIMap.add("com.adobe.postscript", "ps");
        UTIMap.add("com.apple.icns", "icns");
        UTIMap.add("com.apple.macpaint-image", "pntg");
        UTIMap.add("com.apple.pict", "pict");
        UTIMap.add("com.apple.quicktime-image", "qtif");
        UTIMap.add("com.apple.webarchive", "webarchive");
        UTIMap.add("com.compuserve.gif", "gif");        
        UTIMap.add("com.ilm.openexr-image", "exr");
        UTIMap.add("com.kodak.flashpix-image", "fpx");
        UTIMap.add("com.microsoft.bmp", "bmp");
        UTIMap.add("com.microsoft.ico", "ico");
        UTIMap.add("com.netscape.javascript-source", "js");
        UTIMap.add("com.sgi.sgi-image", "sgi");
        UTIMap.add("com.truevision.tga-image", "tga");
    }
    return UTIMap.get(type);
}

} // namespace WebCore
