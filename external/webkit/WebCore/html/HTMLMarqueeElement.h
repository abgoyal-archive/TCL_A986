

#ifndef HTMLMarqueeElement_h
#define HTMLMarqueeElement_h

#include "ActiveDOMObject.h"
#include "HTMLElement.h"

namespace WebCore {

class RenderMarquee;

class HTMLMarqueeElement : public HTMLElement, private ActiveDOMObject {
public:
    HTMLMarqueeElement(const QualifiedName&, Document*);
    
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 3; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    int minimumDelay() const { return m_minimumDelay; }

    // DOM Functions

    void start();
    void stop();
    
private:
    // ActiveDOMObject
    virtual bool canSuspend() const;
    virtual void suspend();
    virtual void resume();

    RenderMarquee* renderMarquee() const;

    int m_minimumDelay;
};

} // namespace WebCore

#endif // HTMLMarqueeElement_h
