

#ifndef CSSVariablesDeclaration_h
#define CSSVariablesDeclaration_h

#include "PlatformString.h"
#include "StringHash.h"
#include "StyleBase.h"
#include <wtf/HashMap.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

typedef int ExceptionCode;

class CSSMutableStyleDeclaration;
class CSSRule;
class CSSValue;
class CSSValueList;

class CSSVariablesDeclaration : public StyleBase {
public:
    static PassRefPtr<CSSVariablesDeclaration> create(StyleBase* owningRule, const Vector<String>& names, const Vector<RefPtr<CSSValue> >& values)
    {
        return adoptRef(new CSSVariablesDeclaration(owningRule, names, values));
    }
    virtual ~CSSVariablesDeclaration();

    String getVariableValue(const String&);
    String removeVariable(const String&, ExceptionCode&);
    void setVariable(const String&, const String&, ExceptionCode&);

    unsigned length() const;
    String item(unsigned index);

    CSSRule* parentRule();

    String cssText() const;
    void setCssText(const String&); // FIXME: The spec contradicts itself regarding whether or not cssText is settable.

    void addParsedVariable(const String& variableName, PassRefPtr<CSSValue> variableValue, bool updateNamesList = true);
    
    CSSValueList* getParsedVariable(const String& variableName);
    CSSMutableStyleDeclaration* getParsedVariableDeclarationBlock(const String& variableName);

private:
    CSSVariablesDeclaration(StyleBase* owningRule, const Vector<String>& names, const Vector<RefPtr<CSSValue> >& values);

    void setNeedsStyleRecalc();

protected:
    Vector<String> m_variableNames;
    HashMap<String, RefPtr<CSSValue> > m_variablesMap;
};

} // namespace WebCore

#endif // CSSVariablesDeclaration_h
