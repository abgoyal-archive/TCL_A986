

#ifndef CSSStyleDeclaration_h
#define CSSStyleDeclaration_h

#include "StyleBase.h"
#include <wtf/Forward.h>

namespace WebCore {

class CSSMutableStyleDeclaration;
class CSSProperty;
class CSSRule;
class CSSValue;

typedef int ExceptionCode;

class CSSStyleDeclaration : public StyleBase {
public:
    static bool isPropertyName(const String&);

    CSSRule* parentRule() const;

    virtual String cssText() const = 0;
    virtual void setCssText(const String&, ExceptionCode&) = 0;

    virtual unsigned length() const = 0;
    bool isEmpty() const { return !length(); }
    virtual String item(unsigned index) const = 0;

    PassRefPtr<CSSValue> getPropertyCSSValue(const String& propertyName);
    String getPropertyValue(const String& propertyName);
    String getPropertyPriority(const String& propertyName);
    String getPropertyShorthand(const String& propertyName);
    bool isPropertyImplicit(const String& propertyName);
    
    virtual PassRefPtr<CSSValue> getPropertyCSSValue(int propertyID) const = 0;
    virtual String getPropertyValue(int propertyID) const = 0;
    virtual bool getPropertyPriority(int propertyID) const = 0;
    virtual int getPropertyShorthand(int propertyID) const = 0;
    virtual bool isPropertyImplicit(int propertyID) const = 0;

    void setProperty(const String& propertyName, const String& value, ExceptionCode&);
    void setProperty(const String& propertyName, const String& value, const String& priority, ExceptionCode&);
    String removeProperty(const String& propertyName, ExceptionCode&);
    virtual void setProperty(int propertyId, const String& value, bool important, ExceptionCode&) = 0;
    virtual String removeProperty(int propertyID, ExceptionCode&) = 0;

    virtual PassRefPtr<CSSMutableStyleDeclaration> copy() const = 0;
    virtual PassRefPtr<CSSMutableStyleDeclaration> makeMutable() = 0;
 
    void diff(CSSMutableStyleDeclaration*) const;

    PassRefPtr<CSSMutableStyleDeclaration> copyPropertiesInSet(const int* set, unsigned length) const;

protected:
    CSSStyleDeclaration(CSSRule* parentRule = 0);

    virtual bool cssPropertyMatches(const CSSProperty*) const;

};

} // namespace WebCore

#endif // CSSStyleDeclaration_h
