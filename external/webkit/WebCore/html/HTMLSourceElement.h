

#ifndef HTMLSourceElement_h
#define HTMLSourceElement_h

#if ENABLE(VIDEO)

#include "HTMLElement.h"
#include "Timer.h"
#include <limits>

namespace WebCore {

class KURL;

class HTMLSourceElement : public HTMLElement {
public:
    HTMLSourceElement(const QualifiedName&, Document*);
    virtual ~HTMLSourceElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
    
    virtual void insertedIntoDocument();
    
    KURL src() const;
    String media() const;
    String type() const;
    void setSrc(const String&);    
    void setMedia(const String&);
    void setType(const String&);
    
    void scheduleErrorEvent();
    void cancelPendingErrorEvent();

private:
    void errorEventTimerFired(Timer<HTMLSourceElement>*);

    Timer<HTMLSourceElement> m_errorEventTimer;
};

} //namespace

#endif
#endif
