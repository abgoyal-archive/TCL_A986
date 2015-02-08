

#ifndef MappedAttribute_h
#define MappedAttribute_h

#include "Attribute.h"
#include "CSSMappedAttributeDeclaration.h"

namespace WebCore {

class MappedAttribute : public Attribute {
public:
    static PassRefPtr<MappedAttribute> create(const QualifiedName& name, const AtomicString& value)
    {
        return adoptRef(new MappedAttribute(name, value, 0));
    }
    static PassRefPtr<MappedAttribute> create(const AtomicString& name, const AtomicString& value)
    {
        return adoptRef(new MappedAttribute(name, value, 0));
    }

    virtual PassRefPtr<Attribute> clone() const;

    virtual CSSStyleDeclaration* style() const { return m_styleDecl.get(); }

    virtual bool isMappedAttribute() { return true; }

    CSSMappedAttributeDeclaration* decl() const { return m_styleDecl.get(); }
    void setDecl(PassRefPtr<CSSMappedAttributeDeclaration> decl) { m_styleDecl = decl; }

private:
    MappedAttribute(const QualifiedName& name, const AtomicString& value, CSSMappedAttributeDeclaration* declaration)
        : Attribute(name, value), m_styleDecl(declaration)
    {
    }
    MappedAttribute(const AtomicString& name, const AtomicString& value, CSSMappedAttributeDeclaration* declaration)
        : Attribute(name, value), m_styleDecl(declaration)
    {
    }

    RefPtr<CSSMappedAttributeDeclaration> m_styleDecl;
};

} //namespace

#endif
