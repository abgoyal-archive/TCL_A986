

#ifndef CSSMappedAttributeDeclaration_h
#define CSSMappedAttributeDeclaration_h

#include "CSSMutableStyleDeclaration.h"
#include "MappedAttributeEntry.h"
#include "QualifiedName.h"

namespace WebCore {

class CSSMappedAttributeDeclaration : public CSSMutableStyleDeclaration {
public:
    static PassRefPtr<CSSMappedAttributeDeclaration> create()
    {
        return adoptRef(new CSSMappedAttributeDeclaration(0));
    }

    virtual ~CSSMappedAttributeDeclaration();

    void setMappedState(MappedAttributeEntry type, const QualifiedName& name, const AtomicString& val)
    {
        m_entryType = type;
        m_attrName = name;
        m_attrValue = val;
    }

private:
    CSSMappedAttributeDeclaration(CSSRule* parentRule)
        : CSSMutableStyleDeclaration(parentRule)
        , m_entryType(eNone)
        , m_attrName(anyQName())
    {
    }
    
    MappedAttributeEntry m_entryType;
    QualifiedName m_attrName;
    AtomicString m_attrValue;
};

} //namespace

#endif
