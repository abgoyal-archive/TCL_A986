

#ifndef CSSComputedStyleDeclaration_h
#define CSSComputedStyleDeclaration_h

#include "CSSStyleDeclaration.h"
#include "Node.h"

namespace WebCore {

class CSSMutableStyleDeclaration;
class ShadowData;

enum EUpdateLayout { DoNotUpdateLayout = false, UpdateLayout = true };

class CSSComputedStyleDeclaration : public CSSStyleDeclaration {
public:
    friend PassRefPtr<CSSComputedStyleDeclaration> computedStyle(PassRefPtr<Node>);
    virtual ~CSSComputedStyleDeclaration();

    virtual String cssText() const;

    virtual unsigned length() const;
    virtual String item(unsigned index) const;

    virtual PassRefPtr<CSSValue> getPropertyCSSValue(int propertyID) const;
    virtual String getPropertyValue(int propertyID) const;
    virtual bool getPropertyPriority(int propertyID) const;
    virtual int getPropertyShorthand(int /*propertyID*/) const { return -1; }
    virtual bool isPropertyImplicit(int /*propertyID*/) const { return false; }

    virtual PassRefPtr<CSSMutableStyleDeclaration> copy() const;
    virtual PassRefPtr<CSSMutableStyleDeclaration> makeMutable();

    PassRefPtr<CSSValue> getPropertyCSSValue(int propertyID, EUpdateLayout) const;
    PassRefPtr<CSSValue> getFontSizeCSSValuePreferringKeyword() const;
#if ENABLE(SVG)
    PassRefPtr<CSSValue> getSVGPropertyCSSValue(int propertyID, EUpdateLayout) const;
#endif

protected:
    virtual bool cssPropertyMatches(const CSSProperty*) const;

private:
    CSSComputedStyleDeclaration(PassRefPtr<Node>);

    virtual void setCssText(const String&, ExceptionCode&);

    virtual String removeProperty(int propertyID, ExceptionCode&);
    virtual void setProperty(int propertyId, const String& value, bool important, ExceptionCode&);

    PassRefPtr<CSSValue> valueForShadow(const ShadowData*, int) const;

    RefPtr<Node> m_node;
};

inline PassRefPtr<CSSComputedStyleDeclaration> computedStyle(PassRefPtr<Node> node)
{
    return adoptRef(new CSSComputedStyleDeclaration(node));
}

} // namespace WebCore

#endif // CSSComputedStyleDeclaration_h
