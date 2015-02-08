

#include "config.h"
#include "KURL.h"
#include "ResourceError.h"

#if USE(CFNETWORK)

// FIXME: Once <rdar://problem/5050881> is fixed in open source we
// can remove this extern "C"
extern "C" { 
#include <CFNetwork/CFNetworkErrors.h>
}

#include <CoreFoundation/CFError.h>
#include <WTF/RetainPtr.h>

namespace WebCore {

const CFStringRef failingURLStringKey = CFSTR("NSErrorFailingURLStringKey");
const CFStringRef failingURLKey = CFSTR("NSErrorFailingURLKey");

// FIXME: Once <rdar://problem/5050841> is fixed we can remove this constructor.
ResourceError::ResourceError(CFStreamError error)
    : m_dataIsUpToDate(true)
{
    m_isNull = false;
    m_errorCode = error.error;

    switch(error.domain) {
    case kCFStreamErrorDomainCustom:
        m_domain ="NSCustomErrorDomain";
        break;
    case kCFStreamErrorDomainPOSIX:
        m_domain = "NSPOSIXErrorDomain";
        break;
    case kCFStreamErrorDomainMacOSStatus:
        m_domain = "NSOSStatusErrorDomain";
        break;
    }
}

void ResourceError::platformLazyInit()
{
    if (m_dataIsUpToDate)
        return;

    if (!m_platformError)
        return;

    CFStringRef domain = CFErrorGetDomain(m_platformError.get());
    if (domain == kCFErrorDomainMach || domain == kCFErrorDomainCocoa)
        m_domain ="NSCustomErrorDomain";
    else if (domain == kCFErrorDomainCFNetwork)
        m_domain = "CFURLErrorDomain";
    else if (domain == kCFErrorDomainPOSIX)
        m_domain = "NSPOSIXErrorDomain";
    else if (domain == kCFErrorDomainOSStatus)
        m_domain = "NSOSStatusErrorDomain";
    else if (domain == kCFErrorDomainWinSock)
        m_domain = "kCFErrorDomainWinSock";

    m_errorCode = CFErrorGetCode(m_platformError.get());

    RetainPtr<CFDictionaryRef> userInfo(AdoptCF, CFErrorCopyUserInfo(m_platformError.get()));
    if (userInfo.get()) {
        CFStringRef failingURLString = (CFStringRef) CFDictionaryGetValue(userInfo.get(), failingURLStringKey);
        if (failingURLString)
            m_failingURL = String(failingURLString);
        else {
            CFURLRef failingURL = (CFURLRef) CFDictionaryGetValue(userInfo.get(), failingURLKey);
            if (failingURL) {
                RetainPtr<CFURLRef> absoluteURLRef(AdoptCF, CFURLCopyAbsoluteURL(failingURL));
                if (absoluteURLRef.get()) {
                    failingURLString = CFURLGetString(absoluteURLRef.get());
                    m_failingURL = String(failingURLString);
                }
            }
        }
        m_localizedDescription = (CFStringRef) CFDictionaryGetValue(userInfo.get(), kCFErrorLocalizedDescriptionKey);
    }

    m_dataIsUpToDate = true;
}

bool ResourceError::platformCompare(const ResourceError& a, const ResourceError& b)
{
    return (CFErrorRef)a == (CFErrorRef)b;
}

ResourceError::operator CFErrorRef() const
{
    if (m_isNull) {
        ASSERT(!m_platformError);
        return nil;
    }
    
    if (!m_platformError) {
        RetainPtr<CFMutableDictionaryRef> userInfo(AdoptCF, CFDictionaryCreateMutable(0, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));

        if (!m_localizedDescription.isEmpty()) {
            RetainPtr<CFStringRef> localizedDescriptionString(AdoptCF, m_localizedDescription.createCFString());
            CFDictionarySetValue(userInfo.get(), kCFErrorLocalizedDescriptionKey, localizedDescriptionString.get());
        }

        if (!m_failingURL.isEmpty()) {
            RetainPtr<CFStringRef> failingURLString(AdoptCF, m_failingURL.createCFString());
            CFDictionarySetValue(userInfo.get(), failingURLStringKey, failingURLString.get());
            RetainPtr<CFURLRef> url(AdoptCF, KURL(ParsedURLString, m_failingURL).createCFURL());
            CFDictionarySetValue(userInfo.get(), failingURLKey, url.get());
        }

        RetainPtr<CFStringRef> domainString(AdoptCF, m_domain.createCFString());
        m_platformError.adoptCF(CFErrorCreate(0, domainString.get(), m_errorCode, userInfo.get()));
    }

    return m_platformError.get();
}

ResourceError::operator CFStreamError() const
{
    lazyInit();

    CFStreamError result;
    result.error = m_errorCode;

    if (m_domain == "NSCustomErrorDomain")
        result.domain = kCFStreamErrorDomainCustom;
    else if (m_domain == "NSPOSIXErrorDomain")
        result.domain = kCFStreamErrorDomainPOSIX;
    else if (m_domain == "NSOSStatusErrorDomain")
        result.domain = kCFStreamErrorDomainMacOSStatus;
    else
        ASSERT_NOT_REACHED();

    return result;
}

} // namespace WebCore

#endif // USE(CFNETWORK)
