

#include "config.h"
#include "RenderImageGeneratedContent.h"

#include "RenderBlock.h"
#include "RenderStyle.h"
#include "StyleCachedImage.h"

namespace WebCore {

RenderImageGeneratedContent::RenderImageGeneratedContent(Node* n)
: RenderImage(n)
{}

RenderImageGeneratedContent::~RenderImageGeneratedContent()
{
    m_cachedImage = 0;
    m_styleImage->removeClient(this);
}

void RenderImageGeneratedContent::setStyleImage(StyleImage* image)
{
    if (image->isCachedImage())
        m_cachedImage = static_cast<StyleCachedImage*>(image)->cachedImage();
    m_styleImage = image;
    m_styleImage->addClient(this);
}

}
