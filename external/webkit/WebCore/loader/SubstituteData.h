

#ifndef SubstituteData_h
#define SubstituteData_h

#include "KURL.h"
#include "SharedBuffer.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class SubstituteData {
    public:
        SubstituteData() { }

        SubstituteData(PassRefPtr<SharedBuffer> content, const String& mimeType, const String& textEncoding, const KURL& failingURL, const KURL& responseURL = KURL())
            : m_content(content)
            , m_mimeType(mimeType)
            , m_textEncoding(textEncoding)
            , m_failingURL(failingURL)
            , m_responseURL(responseURL)
        {
        }

        bool isValid() const { return m_content != 0; }

        const SharedBuffer* content() const { return m_content.get(); }
        const String& mimeType() const { return m_mimeType; }
        const String& textEncoding() const { return m_textEncoding; }
        const KURL& failingURL() const { return m_failingURL; }
        const KURL& responseURL() const { return m_responseURL; }
        
    private:
        RefPtr<SharedBuffer> m_content;
        String m_mimeType;
        String m_textEncoding;
        KURL m_failingURL;
        KURL m_responseURL;
    };

}

#endif // SubstituteData_h

