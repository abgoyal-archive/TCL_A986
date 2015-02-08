

#ifndef WMLDoElement_h
#define WMLDoElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLTaskElement;

class WMLDoElement : public WMLElement {
public:
    WMLDoElement(const QualifiedName& tagName, Document*);

    virtual void defaultEventHandler(Event*);
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    virtual void attach();
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual void recalcStyle(StyleChange);

    void registerTask(WMLTaskElement*);
    void deregisterTask(WMLTaskElement*);

    bool isActive() const { return m_isActive; }
    String label() const;
    String name() const { return m_name; }

    void setActive(bool active) { m_isActive = active; }
    void setNoop(bool noop) { m_isNoop = noop;}

private:
    WMLTaskElement* m_task;
    bool m_isActive;
    bool m_isNoop;
    bool m_isOptional;
    String m_name;
    String m_type;
};

}

#endif
#endif
