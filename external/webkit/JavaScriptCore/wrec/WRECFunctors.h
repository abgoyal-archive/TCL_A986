

#include <wtf/Platform.h>

#if ENABLE(WREC)

#include "WRECGenerator.h"
#include <wtf/unicode/Unicode.h>

namespace JSC { namespace WREC {

    struct CharacterClass;

    class GenerateAtomFunctor {
    public:
        virtual ~GenerateAtomFunctor() {}

        virtual void generateAtom(Generator*, Generator::JumpList&) = 0;
        virtual void backtrack(Generator*) = 0;
    };

    class GeneratePatternCharacterFunctor : public GenerateAtomFunctor {
    public:
        GeneratePatternCharacterFunctor(const UChar ch)
            : m_ch(ch)
        {
        }

        virtual void generateAtom(Generator*, Generator::JumpList&);
        virtual void backtrack(Generator*);

    private:
        const UChar m_ch;
    };

    class GenerateCharacterClassFunctor : public GenerateAtomFunctor {
    public:
        GenerateCharacterClassFunctor(const CharacterClass* charClass, bool invert)
            : m_charClass(charClass)
            , m_invert(invert)
        {
        }

        virtual void generateAtom(Generator*, Generator::JumpList&);
        virtual void backtrack(Generator*);

    private:
        const CharacterClass* m_charClass;
        bool m_invert;
    };

    class GenerateBackreferenceFunctor : public GenerateAtomFunctor {
    public:
        GenerateBackreferenceFunctor(unsigned subpatternId)
            : m_subpatternId(subpatternId)
        {
        }

        virtual void generateAtom(Generator*, Generator::JumpList&);
        virtual void backtrack(Generator*);

    private:
        unsigned m_subpatternId;
    };

    class GenerateParenthesesNonGreedyFunctor : public GenerateAtomFunctor {
    public:
        GenerateParenthesesNonGreedyFunctor(Generator::Label start, Generator::Jump success, Generator::Jump fail)
            : m_start(start)
            , m_success(success)
            , m_fail(fail)
        {
        }

        virtual void generateAtom(Generator*, Generator::JumpList&);
        virtual void backtrack(Generator*);

    private:
        Generator::Label m_start;
        Generator::Jump m_success;
        Generator::Jump m_fail;
    };

} } // namespace JSC::WREC

#endif // ENABLE(WREC)
