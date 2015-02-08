

#ifndef ScriptSourceCode_h
#define ScriptSourceCode_h

#include "CachedScript.h"
#include "KURL.h"
#include "PlatformString.h"

namespace WebCore {

class ScriptSourceCode {
public:
    ScriptSourceCode(const String& source, const KURL& url = KURL(), int startLine = 1)
        : m_source(source)
        , m_url(url)
        , m_startLine(startLine)
    {
    }

    // We lose the encoding information from CachedScript.
    // Not sure if that matters.
    ScriptSourceCode(CachedScript* cs)
        : m_source(cs->script())
        , m_url(ParsedURLString, cs->url())
        , m_startLine(1)
    {
    }

    bool isEmpty() const { return m_source.isEmpty(); }

    const String& source() const { return m_source; }
    const KURL& url() const { return m_url; }
    int startLine() const { return m_startLine; }

private:
    String m_source;
    KURL m_url;
    int m_startLine;
};

} // namespace WebCore

#endif // ScriptSourceCode_h
