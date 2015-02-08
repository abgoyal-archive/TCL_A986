

#ifndef RegExp_h
#define RegExp_h

#include "UString.h"
#include "WREC.h"
#include "ExecutableAllocator.h"
#include <wtf/Forward.h>
#include <wtf/RefCounted.h>
#include "yarr/RegexJIT.h"
#include "yarr/RegexInterpreter.h"

struct JSRegExp;

namespace JSC {

    class JSGlobalData;

    class RegExp : public RefCounted<RegExp> {
    public:
        static PassRefPtr<RegExp> create(JSGlobalData* globalData, const UString& pattern);
        static PassRefPtr<RegExp> create(JSGlobalData* globalData, const UString& pattern, const UString& flags);
#if !ENABLE(YARR)
        ~RegExp();
#endif

        bool global() const { return m_flagBits & Global; }
        bool ignoreCase() const { return m_flagBits & IgnoreCase; }
        bool multiline() const { return m_flagBits & Multiline; }

        const UString& pattern() const { return m_pattern; }

        bool isValid() const { return !m_constructionError; }
        const char* errorMessage() const { return m_constructionError; }

        int match(const UString&, int startOffset, Vector<int, 32>* ovector = 0);
        unsigned numSubpatterns() const { return m_numSubpatterns; }

    private:
        RegExp(JSGlobalData* globalData, const UString& pattern);
        RegExp(JSGlobalData* globalData, const UString& pattern, const UString& flags);

        void compile(JSGlobalData*);

        enum FlagBits { Global = 1, IgnoreCase = 2, Multiline = 4 };

        UString m_pattern; // FIXME: Just decompile m_regExp instead of storing this.
        int m_flagBits;
        const char* m_constructionError;
        unsigned m_numSubpatterns;

#if ENABLE(YARR_JIT)
        Yarr::RegexCodeBlock m_regExpJITCode;
#elif ENABLE(YARR)
        OwnPtr<Yarr::BytecodePattern> m_regExpBytecode;
#else
#if ENABLE(WREC)
        WREC::CompiledRegExp m_wrecFunction;
        RefPtr<ExecutablePool> m_executablePool;
#endif
        JSRegExp* m_regExp;
#endif
    };

} // namespace JSC

#endif // RegExp_h
