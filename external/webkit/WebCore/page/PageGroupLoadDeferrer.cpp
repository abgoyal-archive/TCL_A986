

#include "config.h"
#include "PageGroupLoadDeferrer.h"

#include "Frame.h"
#include "Page.h"
#include "PageGroup.h"
#include <wtf/HashSet.h>

namespace WebCore {

using namespace std;

PageGroupLoadDeferrer::PageGroupLoadDeferrer(Page* page, bool deferSelf)
{
    const HashSet<Page*>& pages = page->group().pages();

    HashSet<Page*>::const_iterator end = pages.end();
    for (HashSet<Page*>::const_iterator it = pages.begin(); it != end; ++it) {
        Page* otherPage = *it;
        if ((deferSelf || otherPage != page)) {
            if (!otherPage->defersLoading())
                m_deferredFrames.append(otherPage->mainFrame());

            // This code is not logically part of load deferring, but we do not want JS code executed beneath modal
            // windows or sheets, which is exactly when PageGroupLoadDeferrer is used.
            for (Frame* frame = otherPage->mainFrame(); frame; frame = frame->tree()->traverseNext())
                frame->document()->suspendActiveDOMObjects();
        }
    }

    size_t count = m_deferredFrames.size();
    for (size_t i = 0; i < count; ++i)
        if (Page* page = m_deferredFrames[i]->page())
            page->setDefersLoading(true);
}

PageGroupLoadDeferrer::~PageGroupLoadDeferrer()
{
    for (size_t i = 0; i < m_deferredFrames.size(); ++i) {
        if (Page* page = m_deferredFrames[i]->page()) {
            page->setDefersLoading(false);

            for (Frame* frame = page->mainFrame(); frame; frame = frame->tree()->traverseNext())
                frame->document()->resumeActiveDOMObjects();
        }
    }
}


} // namespace WebCore
