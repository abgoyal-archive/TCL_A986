

#ifndef NinePieceImage_h
#define NinePieceImage_h

#include "LengthBox.h"
#include "StyleImage.h"

namespace WebCore {

enum ENinePieceImageRule {
    StretchImageRule, RoundImageRule, RepeatImageRule
};

class NinePieceImage {
public:
    NinePieceImage()
        : m_image(0)
        , m_horizontalRule(StretchImageRule)
        , m_verticalRule(StretchImageRule)
    {
    }

    NinePieceImage(StyleImage* image, LengthBox slices, ENinePieceImageRule h, ENinePieceImageRule v) 
      : m_image(image)
      , m_slices(slices)
      , m_horizontalRule(h)
      , m_verticalRule(v)
    {
    }

    bool operator==(const NinePieceImage& o) const;
    bool operator!=(const NinePieceImage& o) const { return !(*this == o); }

    bool hasImage() const { return m_image != 0; }
    StyleImage* image() const { return m_image.get(); }
    
    ENinePieceImageRule horizontalRule() const { return static_cast<ENinePieceImageRule>(m_horizontalRule); }
    ENinePieceImageRule verticalRule() const { return static_cast<ENinePieceImageRule>(m_verticalRule); }
    
    RefPtr<StyleImage> m_image;
    LengthBox m_slices;
    unsigned m_horizontalRule : 2; // ENinePieceImageRule
    unsigned m_verticalRule : 2; // ENinePieceImageRule
};

} // namespace WebCore

#endif // NinePieceImage_h
