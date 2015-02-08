

#ifndef HTMLKeygenElement_h
#define HTMLKeygenElement_h

#include "HTMLSelectElement.h"

namespace WebCore {

class HTMLKeygenElement : public HTMLSelectElement {
public:
    HTMLKeygenElement(const QualifiedName&, Document*, HTMLFormElement* = 0);

    virtual int tagPriority() const { return 0; }
    virtual const AtomicString& formControlType() const;
    virtual bool isEnumeratable() const { return false; }
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual bool appendFormData(FormDataList&, bool);

private:
    AtomicString m_challenge;
    AtomicString m_keyType;
};

} //namespace

#endif
