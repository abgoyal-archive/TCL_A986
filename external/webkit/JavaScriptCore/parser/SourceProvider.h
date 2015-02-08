

#ifndef SourceProvider_h
#define SourceProvider_h

#include "UString.h"
#include <wtf/RefCounted.h>

namespace JSC {

    enum SourceBOMPresence { SourceHasNoBOMs, SourceCouldHaveBOMs };

    class SourceProvider : public RefCounted<SourceProvider> {
    public:
        SourceProvider(const UString& url, SourceBOMPresence hasBOMs = SourceCouldHaveBOMs)
            : m_url(url)
            , m_hasBOMs(hasBOMs)
        {
        }
        virtual ~SourceProvider() { }

        virtual UString getRange(int start, int end) const = 0;
        virtual const UChar* data() const = 0;
        virtual int length() const = 0;
        
        const UString& url() { return m_url; }
        intptr_t asID() { return reinterpret_cast<intptr_t>(this); }

        SourceBOMPresence hasBOMs() const { return m_hasBOMs; }

    private:
        UString m_url;
        SourceBOMPresence m_hasBOMs;
    };

    class UStringSourceProvider : public SourceProvider {
    public:
        static PassRefPtr<UStringSourceProvider> create(const UString& source, const UString& url)
        {
            return adoptRef(new UStringSourceProvider(source, url));
        }

        UString getRange(int start, int end) const { return m_source.substr(start, end - start); }
        const UChar* data() const { return m_source.data(); }
        int length() const { return m_source.size(); }

    private:
        UStringSourceProvider(const UString& source, const UString& url)
            : SourceProvider(url)
            , m_source(source)
        {
        }

        UString m_source;
    };
    
} // namespace JSC

#endif // SourceProvider_h
