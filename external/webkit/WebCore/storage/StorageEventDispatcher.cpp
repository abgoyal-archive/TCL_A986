

#include "config.h"
#include "StorageEventDispatcher.h"

#if ENABLE(DOM_STORAGE)

#include "DOMWindow.h"
#include "EventNames.h"
#include "Frame.h"
#include "Page.h"
#include "PageGroup.h"
#include "SecurityOrigin.h"
#include "StorageEvent.h"

namespace WebCore {

void StorageEventDispatcher::dispatch(const String& key, const String& oldValue, const String& newValue, StorageType storageType, SecurityOrigin* securityOrigin, Frame* sourceFrame)
{
#ifdef ANDROID
    if (!sourceFrame)
        return;
#endif
    Page* page = sourceFrame->page();
    if (!page)
        return;

    // We need to copy all relevant frames from every page to a vector since sending the event to one frame might mutate the frame tree
    // of any given page in the group or mutate the page group itself.
    Vector<RefPtr<Frame> > frames;
    if (storageType == SessionStorage) {
        // Send events only to our page.
        for (Frame* frame = page->mainFrame(); frame; frame = frame->tree()->traverseNext()) {
            if (sourceFrame != frame && frame->document()->securityOrigin()->equal(securityOrigin))
                frames.append(frame);
        }

        for (unsigned i = 0; i < frames.size(); ++i)
            frames[i]->document()->enqueueStorageEvent(StorageEvent::create(eventNames().storageEvent, key, oldValue, newValue, sourceFrame->document()->url(), frames[i]->domWindow()->sessionStorage()));
    } else {
        // Send events to every page.
        const HashSet<Page*>& pages = page->group().pages();
        HashSet<Page*>::const_iterator end = pages.end();
        for (HashSet<Page*>::const_iterator it = pages.begin(); it != end; ++it) {
            for (Frame* frame = (*it)->mainFrame(); frame; frame = frame->tree()->traverseNext()) {
                if (sourceFrame != frame && frame->document()->securityOrigin()->equal(securityOrigin))
                    frames.append(frame);
            }
        }

        for (unsigned i = 0; i < frames.size(); ++i)
            frames[i]->document()->enqueueStorageEvent(StorageEvent::create(eventNames().storageEvent, key, oldValue, newValue, sourceFrame->document()->url(), frames[i]->domWindow()->localStorage()));
    }
}

}

#endif // ENABLE(DOM_STORAGE)
