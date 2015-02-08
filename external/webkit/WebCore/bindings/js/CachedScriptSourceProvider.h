

#ifndef CachedScriptSourceProvider_h
#define CachedScriptSourceProvider_h

#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include "CachedScript.h"
#include "ScriptSourceProvider.h"
#include <parser/SourceCode.h>

namespace WebCore {

    class CachedScriptSourceProvider : public ScriptSourceProvider, public CachedResourceClient {
    public:
        static PassRefPtr<CachedScriptSourceProvider> create(CachedScript* cachedScript) { return adoptRef(new CachedScriptSourceProvider(cachedScript)); }

        virtual ~CachedScriptSourceProvider()
        {
            m_cachedScript->removeClient(this);
        }

        JSC::UString getRange(int start, int end) const { return JSC::UString(m_cachedScript->script().characters() + start, end - start); }
        const UChar* data() const { return m_cachedScript->script().characters(); }
        int length() const { return m_cachedScript->script().length(); }
        const String& source() const { return m_cachedScript->script(); }

    private:
        CachedScriptSourceProvider(CachedScript* cachedScript)
            : ScriptSourceProvider(cachedScript->url())
            , m_cachedScript(cachedScript)
        {
            m_cachedScript->addClient(this);
        }

        CachedResourceHandle<CachedScript> m_cachedScript;
    };

    inline JSC::SourceCode makeSource(CachedScript* cachedScript)
    {
        return JSC::SourceCode(CachedScriptSourceProvider::create(cachedScript));
    }

} // namespace WebCore

#endif // CachedScriptSourceProvider_h
