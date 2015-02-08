

#include "config.h"
#include "KURL.h"

#include <wtf/RetainPtr.h>
#include <CoreFoundation/CFURL.h>

using namespace std;

namespace WebCore {

KURL::KURL(CFURLRef url)
{
    if (!url) {
        parse(0, 0);
        return;
    }

    CFIndex bytesLength = CFURLGetBytes(url, 0, 0);
    Vector<char, 512> buffer(bytesLength + 6); // 5 for "file:", 1 for null character to end C string
    char* bytes = &buffer[5];
    CFURLGetBytes(url, reinterpret_cast<UInt8*>(bytes), bytesLength);
    bytes[bytesLength] = '\0';
    if (bytes[0] != '/') {
        parse(bytes, 0);
        return;
    }

    buffer[0] = 'f';
    buffer[1] = 'i';
    buffer[2] = 'l';
    buffer[3] = 'e';
    buffer[4] = ':';

    parse(buffer.data(), 0);
}

CFURLRef KURL::createCFURL() const
{
    // FIXME: What should this return for invalid URLs?
    // Currently it throws away the high bytes of the characters in the string in that case,
    // which is clearly wrong.

    Vector<char, 512> buffer;
    copyToBuffer(buffer);

    // NOTE: We use UTF-8 here since this encoding is used when computing strings when returning URL components
    // (e.g calls to NSURL -path). However, this function is not tolerant of illegal UTF-8 sequences, which
    // could either be a malformed string or bytes in a different encoding, like Shift-JIS, so we fall back
    // onto using ISO Latin-1 in those cases.
    CFURLRef result = CFURLCreateAbsoluteURLWithBytes(0, reinterpret_cast<const UInt8*>(buffer.data()), buffer.size(), kCFStringEncodingUTF8, 0, true);
    if (!result)
        result = CFURLCreateAbsoluteURLWithBytes(0, reinterpret_cast<const UInt8*>(buffer.data()), buffer.size(), kCFStringEncodingISOLatin1, 0, true);
    return result;
}

String KURL::fileSystemPath() const
{
    RetainPtr<CFURLRef> cfURL(AdoptCF, createCFURL());
    if (!cfURL)
        return String();

#if PLATFORM(WIN)
    CFURLPathStyle pathStyle = kCFURLWindowsPathStyle;
#else
    CFURLPathStyle pathStyle = kCFURLPOSIXPathStyle;
#endif
    return RetainPtr<CFStringRef>(AdoptCF, CFURLCopyFileSystemPath(cfURL.get(), pathStyle)).get();
}

}
