

#ifndef IdentifierRep_h
#define IdentifierRep_h

#include <wtf/Assertions.h>
#include <wtf/FastAllocBase.h>
#include <wtf/StringExtras.h>
#include <string.h>

namespace WebCore {
    
class IdentifierRep : public FastAllocBase {
public:
    static IdentifierRep* get(int);
    static IdentifierRep* get(const char*);

    static bool isValid(IdentifierRep*);
    
    bool isString() const { return m_isString; }

    int number() const { return m_isString ? 0 : m_value.m_number; }
    const char* string() const { return m_isString ? m_value.m_string : 0; }

private:
    IdentifierRep(int number) 
        : m_isString(false)
    {
        m_value.m_number = number;
    }
    
    IdentifierRep(const char* name)
        : m_isString(true)
    {
        m_value.m_string = fastStrDup(name);
    }
    
    ~IdentifierRep()
    {
        // IdentifierReps should never be deleted.
        ASSERT_NOT_REACHED();
    }
    
    union {
        const char* m_string;
        int m_number;
    } m_value;
    bool m_isString;
};

} // namespace WebCore

#endif // IdentifierRep_h
