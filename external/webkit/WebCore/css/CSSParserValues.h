

#ifndef CSSParserValues_h
#define CSSParserValues_h

#include "AtomicString.h"

namespace WebCore {

class CSSValue;

struct CSSParserString {
    UChar* characters;
    int length;

    void lower();

    operator String() const { return String(characters, length); }
    operator AtomicString() const { return AtomicString(characters, length); }
};

struct CSSParserFunction;

struct CSSParserValue {
    int id;
    bool isInt;
    union {
        double fValue;
        int iValue;
        CSSParserString string;
        CSSParserFunction* function;
    };
    enum {
        Operator = 0x100000,
        Function = 0x100001,
        Q_EMS    = 0x100002
    };
    int unit;
    
    bool isVariable() const;
    
    PassRefPtr<CSSValue> createCSSValue();
};

class CSSParserValueList : public FastAllocBase {
public:
    CSSParserValueList()
        : m_current(0)
        , m_variablesCount(0)
    {
    }
    ~CSSParserValueList();
    
    void addValue(const CSSParserValue&);
    void deleteValueAt(unsigned);

    unsigned size() const { return m_values.size(); }
    CSSParserValue* current() { return m_current < m_values.size() ? &m_values[m_current] : 0; }
    CSSParserValue* next() { ++m_current; return current(); }

    CSSParserValue* valueAt(unsigned i) { return i < m_values.size() ? &m_values[i] : 0; }
        
    void clear() { m_values.clear(); }

    bool containsVariables() const { return m_variablesCount; }

private:
    Vector<CSSParserValue, 4> m_values;
    unsigned m_current;
    unsigned m_variablesCount;
};

struct CSSParserFunction : FastAllocBase {
    CSSParserString name;
    CSSParserValueList* args;

    ~CSSParserFunction() { delete args; }
};

}

#endif
