

#ifndef HTMLFrameElementBase_h
#define HTMLFrameElementBase_h

#include "HTMLFrameOwnerElement.h"
#include "ScrollTypes.h"

namespace WebCore {

class HTMLFrameElementBase : public HTMLFrameOwnerElement {
public:
    KURL location() const;
    void setLocation(const String&);

    virtual ScrollbarMode scrollingMode() const { return m_scrolling; }
    
    int getMarginWidth() const { return m_marginWidth; }
    int getMarginHeight() const { return m_marginHeight; }

    int width() const;
    int height() const;

    void setRemainsAliveOnRemovalFromTree(bool);

protected:
    HTMLFrameElementBase(const QualifiedName&, Document*);

    bool isURLAllowed() const;

    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    virtual void attach();

private:
    virtual bool canLazyAttach() { return false; }

    virtual bool supportsFocus() const;
    virtual void setFocus(bool);
    
    virtual bool isURLAttribute(Attribute*) const;

    virtual void willRemove();
    void checkAttachedTimerFired(Timer<HTMLFrameElementBase>*);

    bool viewSourceMode() const { return m_viewSource; }

    void setNameAndOpenURL();
    void openURL();

    static void setNameAndOpenURLCallback(Node*);

    AtomicString m_URL;
    AtomicString m_frameName;

    ScrollbarMode m_scrolling;

    int m_marginWidth;
    int m_marginHeight;

    Timer<HTMLFrameElementBase> m_checkAttachedTimer;

    bool m_viewSource;

    bool m_shouldOpenURLAfterAttach;

    bool m_remainsAliveOnRemovalFromTree;
};

} // namespace WebCore

#endif // HTMLFrameElementBase_h
