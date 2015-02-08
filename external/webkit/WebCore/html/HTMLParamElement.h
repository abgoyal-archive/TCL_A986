

#ifndef HTMLParamElement_h
#define HTMLParamElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLParamElement : public HTMLElement {
    friend class HTMLAppletElement;
public:
    HTMLParamElement(const QualifiedName&, Document*);
    ~HTMLParamElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual void parseMappedAttribute(MappedAttribute*);

    virtual bool isURLAttribute(Attribute*) const;

    String name() const { return m_name; }
    void setName(const String&);

    String type() const;
    void setType(const String&);

    String value() const { return m_value; }
    void setValue(const String&);

    String valueType() const;
    void setValueType(const String&);

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

 protected:
    AtomicString m_name;
    AtomicString m_value;
};


}

#endif
