

#include "config.h"
#include "StorageEventDispatcherImpl.h"

#if ENABLE(DOM_STORAGE)

#include "DOMWindow.h"
#include "EventNames.h"
#include "Frame.h"
#include "KURL.h"
#include "Page.h"
#include "PageGroup.h"
#include "SecurityOrigin.h"
#include "StorageEvent.h"

namespace WebCore {

StorageEventDispatcherImpl::StorageEventDispatcherImpl(const String& groupName)
    : m_pageGroup(PageGroup::pageGroup(groupName))
{
    ASSERT(m_pageGroup);
}

void StorageEventDispatcherImpl::dispatchStorageEvent(const String& key, const String& oldValue,
                                                      const String& newValue, SecurityOrigin* securityOrigin,
                                                      const KURL& url, StorageType storageType)
{
    // FIXME: Implement
    if (storageType == SessionStorage)
        return;

    // We need to copy all relevant frames from every page to a vector since sending the event to one frame might mutate the frame tree
    // of any given page in the group or mutate the page group itself.
    Vector<RefPtr<Frame> > frames;

    const HashSet<Page*>& pages = m_pageGroup->pages();
    HashSet<Page*>::const_iterator end = pages.end();
    for (HashSet<Page*>::const_iterator it = pages.begin(); it != end; ++it) {
        for (Frame* frame = (*it)->mainFrame(); frame; frame = frame->tree()->traverseNext()) {
            if (frame->document()->securityOrigin()->equal(securityOrigin))
                frames.append(frame);
        }
    }

    // FIXME: Figure out how to pass in the document URI.
    for (unsigned i = 0; i < frames.size(); ++i) {
        frames[i]->document()->dispatchWindowEvent(StorageEvent::create(eventNames().storageEvent, key, oldValue, newValue,
                                                                        url, frames[i]->domWindow()->localStorage()));
    }
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
