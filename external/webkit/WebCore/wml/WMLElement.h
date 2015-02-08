

#ifndef WMLElement_h
#define WMLElement_h

#if ENABLE(WML)
#include "StyledElement.h"
#include "WMLErrorHandling.h"

namespace WebCore {

class WMLElement : public StyledElement {
public:
    static PassRefPtr<WMLElement> create(const QualifiedName& tagName, Document*);

    virtual bool isWMLElement() const { return true; }
    virtual bool isWMLTaskElement() const { return false; }

    virtual bool mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual String title() const;

    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

protected:
    WMLElement(const QualifiedName& tagName, Document*);

    // Helper function for derived classes
    String parseValueSubstitutingVariableReferences(const AtomicString&, WMLErrorCode defaultErrorCode = WMLErrorInvalidVariableReference) const;
    String parseValueForbiddingVariableReferences(const AtomicString&) const;
};

}

#endif
#endif
