

#ifndef Escapes_h
#define Escapes_h

#include <wtf/Platform.h>

#if ENABLE(WREC)

#include <wtf/Assertions.h>

namespace JSC { namespace WREC {

    class CharacterClass;

    class Escape {
    public:
        enum Type {
            PatternCharacter,
            CharacterClass,
            Backreference,
            WordBoundaryAssertion,
            Error,
        };
        
        Escape(Type type)
            : m_type(type)
        {
        }

        Type type() const { return m_type; }

    private:
        Type m_type;
        
    protected:
        // Used by subclasses to store data.
        union {
            int i;
            const WREC::CharacterClass* c;
        } m_u;
        bool m_invert;
    };

    class PatternCharacterEscape : public Escape {
    public:
        static const PatternCharacterEscape& cast(const Escape& escape)
        {
            ASSERT(escape.type() == PatternCharacter);
            return static_cast<const PatternCharacterEscape&>(escape);
        }
        
        PatternCharacterEscape(int character)
            : Escape(PatternCharacter)
        {
            m_u.i = character;
        }
        
        operator Escape() const { return *this; }
        
        int character() const { return m_u.i; }
    };

    class CharacterClassEscape : public Escape {
    public:
        static const CharacterClassEscape& cast(const Escape& escape)
        {
            ASSERT(escape.type() == CharacterClass);
            return static_cast<const CharacterClassEscape&>(escape);
        }
        
        CharacterClassEscape(const WREC::CharacterClass& characterClass, bool invert)
            : Escape(CharacterClass)
        {
            m_u.c = &characterClass;
            m_invert = invert;
        }
        
        operator Escape() { return *this; }
        
        const WREC::CharacterClass& characterClass() const { return *m_u.c; }
        bool invert() const { return m_invert; }
    };

    class BackreferenceEscape : public Escape {
    public:
        static const BackreferenceEscape& cast(const Escape& escape)
        {
            ASSERT(escape.type() == Backreference);
            return static_cast<const BackreferenceEscape&>(escape);
        }
        
        BackreferenceEscape(int subpatternId)
            : Escape(Backreference)
        {
            m_u.i = subpatternId;
        }
        
        operator Escape() const { return *this; }
        
        int subpatternId() const { return m_u.i; }
    };

    class WordBoundaryAssertionEscape : public Escape {
    public:
        static const WordBoundaryAssertionEscape& cast(const Escape& escape)
        {
            ASSERT(escape.type() == WordBoundaryAssertion);
            return static_cast<const WordBoundaryAssertionEscape&>(escape);
        }
        
        WordBoundaryAssertionEscape(bool invert)
            : Escape(WordBoundaryAssertion)
        {
            m_invert = invert;
        }
        
        operator Escape() const { return *this; }
        
        bool invert() const { return m_invert; }
    };

} } // namespace JSC::WREC

#endif // ENABLE(WREC)

#endif // Escapes_h
