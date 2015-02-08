

#include "config.h"
#include "NinePieceImage.h"

namespace WebCore {

bool NinePieceImage::operator==(const NinePieceImage& o) const
{
    return StyleImage::imagesEquivalent(m_image.get(), o.m_image.get()) && m_slices == o.m_slices && m_horizontalRule == o.m_horizontalRule &&
           m_verticalRule == o.m_verticalRule;
}

}
