

#ifndef WMLTimerElement_h
#define WMLTimerElement_h

#if ENABLE(WML)
#include "Timer.h"
#include "WMLElement.h"

namespace WebCore {

class WMLCardElement;

class WMLTimerElement : public WMLElement {
public:
    WMLTimerElement(const QualifiedName& tagName, Document*);

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    void timerFired(Timer<WMLTimerElement>*);

    void start(int interval = -1);
    void stop();
    void storeIntervalToPageState();

    String value() const;

private:
    WMLCardElement* m_card;
    String m_name;
    Timer<WMLTimerElement> m_timer;
};

}

#endif
#endif
