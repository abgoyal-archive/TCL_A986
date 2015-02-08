

#include "config.h"
#include "RenderPart.h"

#include "Frame.h"
#include "FrameView.h"

namespace WebCore {

RenderPart::RenderPart(Element* node)
    : RenderWidget(node)
    , m_hasFallbackContent(false)
{
    // init RenderObject attributes
    setInline(false);
}

RenderPart::~RenderPart()
{
    clearWidget();
}

void RenderPart::setWidget(PassRefPtr<Widget> widget)
{
    if (widget == this->widget())
        return;

    RenderWidget::setWidget(widget);

    // make sure the scrollbars are set correctly for restore
    // ### find better fix
    viewCleared();
}

void RenderPart::viewCleared()
{
}

}
