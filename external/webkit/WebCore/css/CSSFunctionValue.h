

#ifndef CSSFunctionValue_h
#define CSSFunctionValue_h

#include "CSSValue.h"

namespace WebCore {

class CSSValueList;
struct CSSParserFunction;

class CSSFunctionValue : public CSSValue {
public:
    static PassRefPtr<CSSFunctionValue> create(CSSParserFunction* function)
    {
        return adoptRef(new CSSFunctionValue(function));
    }

    ~CSSFunctionValue();
    
    virtual String cssText() const;

    virtual CSSParserValue parserValue() const;

private:
    CSSFunctionValue(CSSParserFunction*);
    
    String m_name;
    RefPtr<CSSValueList> m_args;
};

}
#endif

