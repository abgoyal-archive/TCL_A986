

#ifndef HTMLAudioElement_h
#define HTMLAudioElement_h

#if ENABLE(VIDEO)

#include "HTMLMediaElement.h"

namespace WebCore {

class HTMLAudioElement : public HTMLMediaElement {
public:
    HTMLAudioElement(const QualifiedName&, Document*);
    
    virtual int tagPriority() const { return 5; }
};

} //namespace

#endif
#endif
