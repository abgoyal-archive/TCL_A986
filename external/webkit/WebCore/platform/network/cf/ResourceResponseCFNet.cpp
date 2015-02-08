

#include "config.h"
#include "ResourceResponse.h"

#include "HTTPParsers.h"
#include "MIMETypeRegistry.h"
#include <CFNetwork/CFURLResponsePriv.h>
#include <wtf/RetainPtr.h>

using namespace std;

// We would like a better value for a maximum time_t,
// but there is no way to do that in C with any certainty.
// INT_MAX should work well enough for our purposes.
#define MAX_TIME_T ((time_t)INT_MAX)    

namespace WebCore {

CFURLResponseRef ResourceResponse::cfURLResponse() const
{  
    return m_cfResponse.get();
}

static inline bool filenameHasSaneExtension(const String& filename)
{
    int dot = filename.find('.');

    // The dot can't be the first or last character in the filename.
    int length = filename.length();
    return dot > 0 && dot < length - 1;
}

static time_t toTimeT(CFAbsoluteTime time)
{
    static const double maxTimeAsDouble = std::numeric_limits<time_t>::max();
    static const double minTimeAsDouble = std::numeric_limits<time_t>::min();
    return min(max(minTimeAsDouble, time + kCFAbsoluteTimeIntervalSince1970), maxTimeAsDouble);
}

void ResourceResponse::platformLazyInit()
{
    if (m_isUpToDate)
        return;
    m_isUpToDate = true;

    if (m_isNull) {
        ASSERT(!m_cfResponse.get());
        return;
    }

    // FIXME: We may need to do MIME type sniffing here (unless that is done in CFURLResponseGetMIMEType).

    m_url = CFURLResponseGetURL(m_cfResponse.get());
    m_mimeType = CFURLResponseGetMIMEType(m_cfResponse.get());
    m_expectedContentLength = CFURLResponseGetExpectedContentLength(m_cfResponse.get());
    m_textEncodingName = CFURLResponseGetTextEncodingName(m_cfResponse.get());

    m_lastModifiedDate = toTimeT(CFURLResponseGetLastModifiedDate(m_cfResponse.get()));

    RetainPtr<CFStringRef> suggestedFilename(AdoptCF, CFURLResponseCopySuggestedFilename(m_cfResponse.get()));
    m_suggestedFilename = suggestedFilename.get();

    CFHTTPMessageRef httpResponse = CFURLResponseGetHTTPResponse(m_cfResponse.get());
    if (httpResponse) {
        m_httpStatusCode = CFHTTPMessageGetResponseStatusCode(httpResponse);

        RetainPtr<CFStringRef> statusLine(AdoptCF, CFHTTPMessageCopyResponseStatusLine(httpResponse));
        String statusText(statusLine.get());
        int spacePos = statusText.find(' ');
        // Remove the status code from the status text.
        spacePos = statusText.find(' ', spacePos + 1);
        statusText = statusText.substring(spacePos + 1);      

        m_httpStatusText = statusText;

        RetainPtr<CFDictionaryRef> headers(AdoptCF, CFHTTPMessageCopyAllHeaderFields(httpResponse));
        CFIndex headerCount = CFDictionaryGetCount(headers.get());
        Vector<const void*, 128> keys(headerCount);
        Vector<const void*, 128> values(headerCount);
        CFDictionaryGetKeysAndValues(headers.get(), keys.data(), values.data());
        for (int i = 0; i < headerCount; ++i)
            m_httpHeaderFields.set((CFStringRef)keys[i], (CFStringRef)values[i]);
    } else
        m_httpStatusCode = 0;
}

bool ResourceResponse::platformCompare(const ResourceResponse& a, const ResourceResponse& b)
{
    return CFEqual(a.cfURLResponse(), b.cfURLResponse());
}


}
