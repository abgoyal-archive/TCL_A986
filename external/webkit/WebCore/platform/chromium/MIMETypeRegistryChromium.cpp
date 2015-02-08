

#include "config.h"
#include "MIMETypeRegistry.h"

#include "ChromiumBridge.h"
#include "CString.h"
#include "MediaPlayer.h"
#include "PluginDataChromium.h"

// NOTE: Unlike other ports, we don't use the shared implementation bits in
// MIMETypeRegistry.cpp.  Instead, we need to route most functions via the
// ChromiumBridge to the embedder.

namespace WebCore {

String MIMETypeRegistry::getMIMETypeForExtension(const String &ext)
{
    return ChromiumBridge::mimeTypeForExtension(ext);
}

// Returns the file extension if one is found.  Does not include the dot in the
// filename.  E.g., 'html'.
String MIMETypeRegistry::getPreferredExtensionForMIMEType(const String& type)
{
    // Prune out any parameters in case they happen to have snuck in there...
    // FIXME: Is this really necessary??
    String mimeType = type.substring(0, static_cast<unsigned>(type.find(';')));

    String ext = ChromiumBridge::preferredExtensionForMIMEType(type);
    if (!ext.isEmpty() && ext[0] == '.')
        ext = ext.substring(1);

    return ext;
}

String MIMETypeRegistry::getMIMETypeForPath(const String& path)
{
    int pos = path.reverseFind('.');
    if (pos < 0)
        return "application/octet-stream";
    String extension = path.substring(pos + 1);
    String mimeType = getMIMETypeForExtension(extension);
    if (mimeType.isEmpty()) {
        // If there's no mimetype registered for the extension, check to see
        // if a plugin can handle the extension.
        mimeType = getPluginMimeTypeFromExtension(extension);
    }
    if (mimeType.isEmpty())
        return "application/octet-stream";
    return mimeType;
}

bool MIMETypeRegistry::isSupportedImageMIMEType(const String& mimeType)
{ 
    return ChromiumBridge::isSupportedImageMIMEType(mimeType);
}

bool MIMETypeRegistry::isSupportedImageResourceMIMEType(const String& mimeType)
{ 
    return isSupportedImageMIMEType(mimeType); 
}

bool MIMETypeRegistry::isSupportedImageMIMETypeForEncoding(const String& mimeType)
{
    // FIXME: Fill this out. See: http://trac.webkit.org/changeset/30888
    return isSupportedImageMIMEType(mimeType);
}

bool MIMETypeRegistry::isSupportedJavaScriptMIMEType(const String& mimeType)
{
    return ChromiumBridge::isSupportedJavaScriptMIMEType(mimeType);
}
    
bool MIMETypeRegistry::isSupportedNonImageMIMEType(const String& mimeType)
{
    return ChromiumBridge::isSupportedNonImageMIMEType(mimeType);
}

bool MIMETypeRegistry::isSupportedMediaMIMEType(const String& mimeType)
{
    HashSet<String> supportedMediaMIMETypes;
#if ENABLE(VIDEO)
    MediaPlayer::getSupportedTypes(supportedMediaMIMETypes);
#endif
    return !mimeType.isEmpty() && supportedMediaMIMETypes.contains(mimeType);
}

bool MIMETypeRegistry::isJavaAppletMIMEType(const String& mimeType)
{
    // Since this set is very limited and is likely to remain so we won't bother with the overhead
    // of using a hash set.
    // Any of the MIME types below may be followed by any number of specific versions of the JVM,
    // which is why we use startsWith()
    return mimeType.startsWith("application/x-java-applet", false) 
        || mimeType.startsWith("application/x-java-bean", false) 
        || mimeType.startsWith("application/x-java-vm", false);
}

String MIMETypeRegistry::getMediaMIMETypeForExtension(const String&)
{
    return String();
}

static HashSet<String>& dummyHashSet()
{
    ASSERT_NOT_REACHED();
    static HashSet<String> dummy;
    return dummy;
}

// NOTE: the following methods should never be reached
HashSet<String>& MIMETypeRegistry::getSupportedImageMIMETypes() { return dummyHashSet(); }
HashSet<String>& MIMETypeRegistry::getSupportedImageResourceMIMETypes() { return dummyHashSet(); }
HashSet<String>& MIMETypeRegistry::getSupportedImageMIMETypesForEncoding() { return dummyHashSet(); }
HashSet<String>& MIMETypeRegistry::getSupportedNonImageMIMETypes() { return dummyHashSet(); }
HashSet<String>& MIMETypeRegistry::getSupportedMediaMIMETypes() { return dummyHashSet(); }

} // namespace WebCore
