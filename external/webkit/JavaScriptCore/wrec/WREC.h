

#ifndef WREC_h
#define WREC_h

#include <wtf/Platform.h>

#if ENABLE(WREC)

#include <wtf/unicode/Unicode.h>

#if COMPILER(GCC) && CPU(X86)
#define WREC_CALL __attribute__ ((regparm (3)))
#else
#define WREC_CALL
#endif

namespace JSC {
    class Interpreter;
    class UString;
}

namespace JSC { namespace WREC {

    typedef int (*CompiledRegExp)(const UChar* input, unsigned start, unsigned length, int* output) WREC_CALL;

} } // namespace JSC::WREC

#endif // ENABLE(WREC)

#endif // WREC_h
