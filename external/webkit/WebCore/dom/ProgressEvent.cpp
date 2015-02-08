

#include "config.h"
#include "ProgressEvent.h"

namespace WebCore {
    
ProgressEvent::ProgressEvent()
    : m_lengthComputable(false)
    , m_loaded(0)
    , m_total(0)
{
}

ProgressEvent::ProgressEvent(const AtomicString& type, bool lengthComputable, unsigned loaded, unsigned total)
    : Event(type, false, true)
    , m_lengthComputable(lengthComputable)
    , m_loaded(loaded)
    , m_total(total)
{
}
    
void ProgressEvent::initProgressEvent(const AtomicString& typeArg, 
                                      bool canBubbleArg,
                                      bool cancelableArg,
                                      bool lengthComputableArg,
                                      unsigned loadedArg,
                                      unsigned totalArg)
{    
    if (dispatched())
        return;

    initEvent(typeArg, canBubbleArg, cancelableArg);

    m_lengthComputable = lengthComputableArg;
    m_loaded = loadedArg;
    m_total = totalArg;
}

}
