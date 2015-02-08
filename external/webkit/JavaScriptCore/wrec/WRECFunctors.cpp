

#include "config.h"
#include "WRECFunctors.h"

#if ENABLE(WREC)

#include "WRECGenerator.h"

using namespace WTF;

namespace JSC { namespace WREC {

void GeneratePatternCharacterFunctor::generateAtom(Generator* generator, Generator::JumpList& failures)
{
    generator->generatePatternCharacter(failures, m_ch);
}

void GeneratePatternCharacterFunctor::backtrack(Generator* generator)
{
    generator->generateBacktrack1();
}

void GenerateCharacterClassFunctor::generateAtom(Generator* generator, Generator::JumpList& failures)
{
    generator->generateCharacterClass(failures, *m_charClass, m_invert);
}

void GenerateCharacterClassFunctor::backtrack(Generator* generator)
{
    generator->generateBacktrack1();
}

void GenerateBackreferenceFunctor::generateAtom(Generator* generator, Generator::JumpList& failures)
{
    generator->generateBackreference(failures, m_subpatternId);
}

void GenerateBackreferenceFunctor::backtrack(Generator* generator)
{
    generator->generateBacktrackBackreference(m_subpatternId);
}

void GenerateParenthesesNonGreedyFunctor::generateAtom(Generator* generator, Generator::JumpList& failures)
{
    generator->generateParenthesesNonGreedy(failures, m_start, m_success, m_fail);
}

void GenerateParenthesesNonGreedyFunctor::backtrack(Generator*)
{
    // FIXME: do something about this.
    CRASH();
}

} } // namespace JSC::WREC

#endif // ENABLE(WREC)
