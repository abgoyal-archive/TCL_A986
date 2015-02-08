

#ifndef HTMLBaseElement_h
#define HTMLBaseElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLBaseElement : public HTMLElement {
public:
    HTMLBaseElement(const QualifiedName&, Document*);

private:
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual String target() const { return m_target; }

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    void process();
    
    void setHref(const String&);
    void setTarget(const String&);

    String m_hrefAttrValue;
    String m_href;
    String m_target;
};

} //namespace

#endif
