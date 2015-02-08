
#import "config.h"
#import "FileSystem.h"

#import "CString.h"
#import "PlatformString.h"
#import <wtf/RetainPtr.h>

namespace WebCore {

CString fileSystemRepresentation(const String& path)
{
    RetainPtr<CFStringRef> cfString(AdoptCF, path.createCFString());

    if (!cfString)
        return CString();

    CFIndex size = CFStringGetMaximumSizeOfFileSystemRepresentation(cfString.get());

    char* buffer;
    CString string = CString::newUninitialized(size, buffer);

    if (!CFStringGetFileSystemRepresentation(cfString.get(), buffer, size)) {
        LOG_ERROR("Failed to get filesystem representation to create CString from cfString");
        return CString();
    }

    return string;
}

} // namespace WebCore
