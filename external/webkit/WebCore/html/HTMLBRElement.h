

#ifndef HTMLBRElement_h
#define HTMLBRElement_h

#include "HTMLElement.h"

namespace WebCore {

class String;

class HTMLBRElement : public HTMLElement {
public:
    HTMLBRElement(const QualifiedName&, Document*);

private:
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);
    
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
};

} //namespace

#endif
