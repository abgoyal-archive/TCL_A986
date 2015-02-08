

#include "config.h"
#include "DocumentLoader.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameTree.h"
#include "Page.h"

namespace WebCore {

void Page::addSchedulePair(PassRefPtr<SchedulePair> prpPair)
{
    RefPtr<SchedulePair> pair = prpPair;

    if (!m_scheduledRunLoopPairs)
        m_scheduledRunLoopPairs.set(new SchedulePairHashSet);
    m_scheduledRunLoopPairs->add(pair);

#ifndef BUILDING_ON_TIGER
    for (Frame* frame = m_mainFrame.get(); frame; frame = frame->tree()->traverseNext()) {
        if (DocumentLoader* documentLoader = frame->loader()->documentLoader())
            documentLoader->schedule(pair.get());
        if (DocumentLoader* documentLoader = frame->loader()->provisionalDocumentLoader())
            documentLoader->schedule(pair.get());
    }
#endif

    // FIXME: make SharedTimerMac use these SchedulePairs.
}

void Page::removeSchedulePair(PassRefPtr<SchedulePair> prpPair)
{
    ASSERT(m_scheduledRunLoopPairs);
    if (!m_scheduledRunLoopPairs)
        return;

    RefPtr<SchedulePair> pair = prpPair;
    m_scheduledRunLoopPairs->remove(pair);

#ifndef BUILDING_ON_TIGER
    for (Frame* frame = m_mainFrame.get(); frame; frame = frame->tree()->traverseNext()) {
        if (DocumentLoader* documentLoader = frame->loader()->documentLoader())
            documentLoader->unschedule(pair.get());
        if (DocumentLoader* documentLoader = frame->loader()->provisionalDocumentLoader())
            documentLoader->unschedule(pair.get());
    }
#endif
}

} // namespace
