

#ifndef HTMLIFrameElement_h
#define HTMLIFrameElement_h

#include "HTMLFrameElementBase.h"

namespace WebCore {

class HTMLIFrameElement : public HTMLFrameElementBase {
public:
    static PassRefPtr<HTMLIFrameElement> create(const QualifiedName&, Document*);

private:
    HTMLIFrameElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    
    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    
    virtual bool isURLAttribute(Attribute*) const;

    AtomicString m_name;
};

} // namespace WebCore

#endif // HTMLIFrameElement_h
