

#include "config.h"

#include "Media.h"
#include "CSSStyleSelector.h"
#include "Frame.h"
#include "FrameView.h"
#include "MediaList.h"
#include "MediaQueryEvaluator.h"

namespace WebCore {

Media::Media(Frame* frame)
    : m_frame(frame)
{
}

String Media::type() const
{
    FrameView* view = m_frame ? m_frame->view() : 0;
    if (view)
        return view->mediaType();

    return String();
}

bool Media::matchMedium(const String& query) const
{
    if (!m_frame)
        return false;

    Document* document = m_frame->document();
    ASSERT(document);
    Element* documentElement = document->documentElement();
    ASSERT(documentElement);

    CSSStyleSelector* styleSelector = document->styleSelector();
    if (!styleSelector)
        return false;

    RefPtr<RenderStyle> rootStyle = styleSelector->styleForElement(documentElement, 0 /*defaultParent*/, false /*allowSharing*/, true /*resolveForRootDefault*/);
    RefPtr<MediaList> media = MediaList::create();

    ExceptionCode ec = 0;
    media->setMediaText(query, ec);
    if (ec)
        return false;

    MediaQueryEvaluator screenEval(type(), m_frame, rootStyle.get());
    return screenEval.eval(media.get());
}

} // namespace WebCore
