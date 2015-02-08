

#ifndef HTMLOListElement_h
#define HTMLOListElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLOListElement : public HTMLElement {
public:
    HTMLOListElement(const QualifiedName&, Document*);
        
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    bool compact() const;
    void setCompact(bool);

    int start() const { return m_start; }
    void setStart(int);

    String type() const;
    void setType(const String&);

private:
    int m_start;
};


} //namespace

#endif
