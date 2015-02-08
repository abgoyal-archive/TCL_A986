

#ifndef ScriptSourceCode_h
#define ScriptSourceCode_h

#include "CachedScriptSourceProvider.h"
#include "ScriptSourceProvider.h"
#include "StringSourceProvider.h"
#include "KURL.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class ScriptSourceCode {
public:
    ScriptSourceCode(const String& source, const KURL& url = KURL(), int startLine = 1)
        : m_provider(StringSourceProvider::create(source, url.isNull() ? String() : url.string()))
        , m_code(m_provider, startLine)
        , m_url(url)
    {
    }

    ScriptSourceCode(CachedScript* cs)
        : m_provider(CachedScriptSourceProvider::create(cs))
        , m_code(m_provider)
    {
    }

    bool isEmpty() const { return m_code.length() == 0; }

    const JSC::SourceCode& jsSourceCode() const { return m_code; }

    const String& source() const { return m_provider->source(); }

    int startLine() const { return m_code.firstLine(); }

    const KURL& url() const { return m_url; }
    
private:
    RefPtr<ScriptSourceProvider> m_provider;
    
    JSC::SourceCode m_code;
    
    KURL m_url;

};

} // namespace WebCore

#endif // ScriptSourceCode_h
