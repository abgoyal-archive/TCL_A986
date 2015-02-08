

#include "config.h"
#include "PlatformString.h"

#if PLATFORM(CF)

#include <CoreFoundation/CoreFoundation.h>

namespace WebCore {

String::String(CFStringRef str)
{
    if (!str)
        return;

    CFIndex size = CFStringGetLength(str);
    if (size == 0)
        m_impl = StringImpl::empty();
    else {
        Vector<UChar, 1024> buffer(size);
        CFStringGetCharacters(str, CFRangeMake(0, size), (UniChar*)buffer.data());
        m_impl = StringImpl::create(buffer.data(), size);
    }
}

CFStringRef String::createCFString() const
{
    if (!m_impl)
        return static_cast<CFStringRef>(CFRetain(CFSTR("")));

    return m_impl->createCFString();
}

}

#endif // PLATFORM(CF)
