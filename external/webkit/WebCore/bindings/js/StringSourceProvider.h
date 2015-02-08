

#ifndef StringSourceProvider_h
#define StringSourceProvider_h

#include "ScriptSourceProvider.h"
#include <parser/SourceCode.h>

namespace WebCore {

    class StringSourceProvider : public ScriptSourceProvider {
    public:
        static PassRefPtr<StringSourceProvider> create(const String& source, const String& url) { return adoptRef(new StringSourceProvider(source, url)); }

        JSC::UString getRange(int start, int end) const { return JSC::UString(m_source.characters() + start, end - start); }
        const UChar* data() const { return m_source.characters(); }
        int length() const { return m_source.length(); }
        const String& source() const { return m_source; }

    private:
        StringSourceProvider(const String& source, const String& url)
            : ScriptSourceProvider(url)
            , m_source(source)
        {
        }
        
        String m_source;
    };

    inline JSC::SourceCode makeSource(const String& source, const String& url = String(), int firstLine = 1)
    {
        return JSC::SourceCode(StringSourceProvider::create(source, url), firstLine);
    }

} // namespace WebCore

#endif // StringSourceProvider_h
