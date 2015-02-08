

#ifndef MIMETypeRegistry_h
#define MIMETypeRegistry_h

#include "PlatformString.h"
#include "StringHash.h"
#include <wtf/HashSet.h>
#include <wtf/Vector.h>

namespace WebCore {

class MIMETypeRegistry {
public:
    static String getMIMETypeForExtension(const String& ext);
    static Vector<String> getExtensionsForMIMEType(const String& type);
    static String getPreferredExtensionForMIMEType(const String& type);
    static String getMediaMIMETypeForExtension(const String& ext);

    static String getMIMETypeForPath(const String& path);

    // Check to see if a mime type is suitable for being loaded inline as an
    // image (e.g., <img> tags).
    static bool isSupportedImageMIMEType(const String& mimeType);

    // Check to see if a mime type is suitable for being loaded as an image
    // document in a frame.
    static bool isSupportedImageResourceMIMEType(const String& mimeType);

    // Check to see if a mime type is suitable for being encoded.
    static bool isSupportedImageMIMETypeForEncoding(const String& mimeType);

    // Check to see if a mime type is suitable for being loaded as a JavaScript
    // resource.
    static bool isSupportedJavaScriptMIMEType(const String& mimeType);    

    // Check to see if a non-image mime type is suitable for being loaded as a
    // document in a frame.  Includes supported JavaScript MIME types.
    static bool isSupportedNonImageMIMEType(const String& mimeType);

    // Check to see if a mime type is suitable for being loaded using <video> and <audio>
    static bool isSupportedMediaMIMEType(const String& mimeType); 

    // Check to see if a mime type is suitable for being downloaded.
    static bool isSupportedDownloadMIMEType(const String & mimeType);

    // Check to see if a mime type is a valid Java applet mime type
    static bool isJavaAppletMIMEType(const String& mimeType);

    static HashSet<String>& getSupportedImageMIMETypes();
    static HashSet<String>& getSupportedImageResourceMIMETypes();
    static HashSet<String>& getSupportedImageMIMETypesForEncoding();
    static HashSet<String>& getSupportedNonImageMIMETypes();
    static HashSet<String>& getSupportedMediaMIMETypes();
    static HashSet<String>& getSupportedDownloadMIMETypes();
};

const String& defaultMIMEType();

} // namespace WebCore

#endif // MIMETypeRegistry_h
