

#ifndef SourceCode_h
#define SourceCode_h

#include "SourceProvider.h"
#include <wtf/RefPtr.h>

namespace JSC {

    class SourceCode {
    public:
        SourceCode()
            : m_provider(0)
            , m_startChar(0)
            , m_endChar(0)
            , m_firstLine(0)
        {
        }

        SourceCode(PassRefPtr<SourceProvider> provider, int firstLine = 1)
            : m_provider(provider)
            , m_startChar(0)
            , m_endChar(m_provider->length())
            , m_firstLine(std::max(firstLine, 1))
        {
        }

        SourceCode(PassRefPtr<SourceProvider> provider, int start, int end, int firstLine)
            : m_provider(provider)
            , m_startChar(start)
            , m_endChar(end)
            , m_firstLine(std::max(firstLine, 1))
        {
        }

        UString toString() const
        {
            if (!m_provider)
                return UString();
            return m_provider->getRange(m_startChar, m_endChar);
        }
        
        bool isNull() const { return !m_provider; }
        SourceProvider* provider() const { return m_provider.get(); }
        int firstLine() const { return m_firstLine; }
        int startOffset() const { return m_startChar; }
        int endOffset() const { return m_endChar; }
        const UChar* data() const { return m_provider->data() + m_startChar; }
        int length() const { return m_endChar - m_startChar; }

    private:
        RefPtr<SourceProvider> m_provider;
        int m_startChar;
        int m_endChar;
        int m_firstLine;
    };

    inline SourceCode makeSource(const UString& source, const UString& url = UString(), int firstLine = 1)
    {
        return SourceCode(UStringSourceProvider::create(source, url), firstLine);
    }

} // namespace JSC

#endif // SourceCode_h
