

#include "config.h"
#include "WREC.h"

#if ENABLE(WREC)

#include "CharacterClassConstructor.h"
#include "Interpreter.h"
#include "JSGlobalObject.h"
#include "RegisterFile.h"
#include "WRECFunctors.h"
#include "WRECParser.h"
#include "pcre_internal.h"

using namespace WTF;

namespace JSC { namespace WREC {

CompiledRegExp Generator::compileRegExp(JSGlobalData* globalData, const UString& pattern, unsigned* numSubpatterns_ptr, const char** error_ptr, RefPtr<ExecutablePool>& pool, bool ignoreCase, bool multiline)
{
    if (pattern.size() > MAX_PATTERN_SIZE) {
        *error_ptr = "regular expression too large";
        return 0;
    }

    Parser parser(pattern, ignoreCase, multiline);
    Generator& generator = parser.generator();
    MacroAssembler::JumpList failures;
    MacroAssembler::Jump endOfInput;

    generator.generateEnter();
    generator.generateSaveIndex();

    Label beginPattern(&generator);
    parser.parsePattern(failures);
    generator.generateReturnSuccess();

    failures.link(&generator);
    generator.generateIncrementIndex(&endOfInput);
    parser.parsePattern(failures);
    generator.generateReturnSuccess();

    failures.link(&generator);
    generator.generateIncrementIndex();
    generator.generateJumpIfNotEndOfInput(beginPattern);
    
    endOfInput.link(&generator);
    generator.generateReturnFailure();

    if (parser.error()) {
        *error_ptr = parser.syntaxError(); // NULL in the case of patterns that WREC doesn't support yet.
        return 0;
    }

    *numSubpatterns_ptr = parser.numSubpatterns();
    pool = globalData->executableAllocator.poolForSize(generator.size());
    return reinterpret_cast<CompiledRegExp>(generator.copyCode(pool.get()));
}

} } // namespace JSC::WREC

#endif // ENABLE(WREC)
