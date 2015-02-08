

#ifndef RegexCompiler_h
#define RegexCompiler_h

#include <wtf/Platform.h>

#if ENABLE(YARR)

#include <wtf/unicode/Unicode.h>
#include "RegexParser.h"
#include "RegexPattern.h"

namespace JSC { namespace Yarr {

const char* compileRegex(const UString& patternString, RegexPattern& pattern);

} } // namespace JSC::Yarr

#endif

#endif // RegexCompiler_h
