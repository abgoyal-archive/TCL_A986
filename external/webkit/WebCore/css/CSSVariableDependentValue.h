

#ifndef CSSVariableDependentValue_h
#define CSSVariableDependentValue_h

#include "CSSValue.h"

namespace WebCore {

class CSSValueList;

class CSSVariableDependentValue : public CSSValue {
public:
    static PassRefPtr<CSSVariableDependentValue> create(PassRefPtr<CSSValueList> valueList)
    {
        return adoptRef(new CSSVariableDependentValue(valueList));
    }
    virtual ~CSSVariableDependentValue();
    
    virtual String cssText() const;

    bool isVariableDependentValue() const { return true; }

    CSSValueList* valueList() const { return m_list.get(); }

private:
    CSSVariableDependentValue(PassRefPtr<CSSValueList>);
    
    RefPtr<CSSValueList> m_list;
};

}
#endif

