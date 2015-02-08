

#include "config.h"
#include "CachedPage.h"

#include "FocusController.h"
#include "Frame.h"
#include "FrameView.h"
#include "Page.h"
#include <wtf/CurrentTime.h>
#include <wtf/RefCountedLeakCounter.h>

using namespace JSC;

namespace WebCore {

#ifndef NDEBUG
static WTF::RefCountedLeakCounter cachedPageCounter("CachedPage");
#endif

PassRefPtr<CachedPage> CachedPage::create(Page* page)
{
    return adoptRef(new CachedPage(page));
}

CachedPage::CachedPage(Page* page)
    : m_timeStamp(currentTime())
    , m_cachedMainFrame(CachedFrame::create(page->mainFrame()))
{
#ifndef NDEBUG
    cachedPageCounter.increment();
#endif
}

CachedPage::~CachedPage()
{
#ifndef NDEBUG
    cachedPageCounter.decrement();
#endif

    destroy();
    ASSERT(!m_cachedMainFrame);
}

void CachedPage::restore(Page* page)
{
    ASSERT(m_cachedMainFrame);
    ASSERT(page && page->mainFrame() && page->mainFrame() == m_cachedMainFrame->view()->frame());
    ASSERT(!page->frameCount());

    m_cachedMainFrame->open();
    
    // Restore the focus appearance for the focused element.
    // FIXME: Right now we don't support pages w/ frames in the b/f cache.  This may need to be tweaked when we add support for that.
    Document* focusedDocument = page->focusController()->focusedOrMainFrame()->document();
    if (Node* node = focusedDocument->focusedNode()) {
        if (node->isElementNode())
            static_cast<Element*>(node)->updateFocusAppearance(true);
    }
    
    clear();
}

void CachedPage::clear()
{
    ASSERT(m_cachedMainFrame);
    m_cachedMainFrame->clear();
    m_cachedMainFrame = 0;
}

void CachedPage::destroy()
{
    if (m_cachedMainFrame)
        m_cachedMainFrame->destroy();

    m_cachedMainFrame = 0;
}

} // namespace WebCore
