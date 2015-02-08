

#include "config.h"
#include "PageTransitionEvent.h"

#include "EventNames.h"

namespace WebCore {
        
PageTransitionEvent::PageTransitionEvent()
    : m_persisted(false)
{
}

PageTransitionEvent::PageTransitionEvent(const AtomicString& type, bool persisted)
    : Event(type, true, true)
    , m_persisted(persisted)
{
}

PageTransitionEvent::~PageTransitionEvent()
{
}

void PageTransitionEvent::initPageTransitionEvent(const AtomicString& type, 
                                            bool canBubbleArg,
                                            bool cancelableArg,
                                            bool persisted)
{
    if (dispatched())
        return;
    
    initEvent(type, canBubbleArg, cancelableArg);
    
    m_persisted = persisted;
}

} // namespace WebCore
