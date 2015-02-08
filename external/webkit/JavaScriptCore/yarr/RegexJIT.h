

#ifndef RegexJIT_h
#define RegexJIT_h

#include <wtf/Platform.h>

#if ENABLE(YARR_JIT)

#include "MacroAssembler.h"
#include "RegexPattern.h"
#include <UString.h>

#include <pcre.h>
struct JSRegExp; // temporary, remove when fallback is removed.

#if CPU(X86) && !COMPILER(MSVC)
#define YARR_CALL __attribute__ ((regparm (3)))
#else
#define YARR_CALL
#endif

namespace JSC {

class JSGlobalData;
class ExecutablePool;

namespace Yarr {

class RegexCodeBlock {
    typedef int (*RegexJITCode)(const UChar* input, unsigned start, unsigned length, int* output) YARR_CALL;

public:
    RegexCodeBlock()
        : m_fallback(0)
    {
    }

    ~RegexCodeBlock()
    {
        if (m_fallback)
            jsRegExpFree(m_fallback);
    }

    JSRegExp* getFallback() { return m_fallback; }
    void setFallback(JSRegExp* fallback) { m_fallback = fallback; }

    bool operator!() { return !m_ref.m_code.executableAddress(); }
    void set(MacroAssembler::CodeRef ref) { m_ref = ref; }

    int execute(const UChar* input, unsigned start, unsigned length, int* output)
    {
        return reinterpret_cast<RegexJITCode>(m_ref.m_code.executableAddress())(input, start, length, output);
    }

private:
    MacroAssembler::CodeRef m_ref;
    JSRegExp* m_fallback;
};

void jitCompileRegex(JSGlobalData* globalData, RegexCodeBlock& jitObject, const UString& pattern, unsigned& numSubpatterns, const char*& error, bool ignoreCase = false, bool multiline = false);

inline int executeRegex(RegexCodeBlock& jitObject, const UChar* input, unsigned start, unsigned length, int* output, int outputArraySize)
{
    if (JSRegExp* fallback = jitObject.getFallback())
        return (jsRegExpExecute(fallback, input, length, start, output, outputArraySize) < 0) ? -1 : output[0];

    return jitObject.execute(input, start, length, output);
}

} } // namespace JSC::Yarr

#endif

#endif // RegexJIT_h
