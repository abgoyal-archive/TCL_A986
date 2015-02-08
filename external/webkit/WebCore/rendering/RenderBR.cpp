

#include "config.h"
#include "RenderBR.h"

#include "Document.h"
#include "InlineTextBox.h"
#include "VisiblePosition.h"

namespace WebCore {

RenderBR::RenderBR(Node* node)
    : RenderText(node, StringImpl::create("\n"))
    , m_lineHeight(-1)
{
}

RenderBR::~RenderBR()
{
}

int RenderBR::baselinePosition(bool firstLine, bool isRootLineBox) const
{
    if (firstTextBox() && !firstTextBox()->isText())
        return 0;
    return RenderText::baselinePosition(firstLine, isRootLineBox);
}

int RenderBR::lineHeight(bool firstLine, bool /*isRootLineBox*/) const
{
    if (firstTextBox() && !firstTextBox()->isText())
        return 0;

    if (firstLine) {
        RenderStyle* s = style(firstLine);
        Length lh = s->lineHeight();
        if (lh.isNegative()) {
            if (s == style()) {
                if (m_lineHeight == -1)
                    m_lineHeight = RenderObject::lineHeight(false);
                return m_lineHeight;
            }
            return s->font().lineSpacing();
        }
        if (lh.isPercent())
            return lh.calcMinValue(s->fontSize());
        return lh.value();
    }

    if (m_lineHeight == -1)
        m_lineHeight = RenderObject::lineHeight(false);
    return m_lineHeight;
}

void RenderBR::styleDidChange(StyleDifference diff, const RenderStyle* oldStyle)
{
    RenderText::styleDidChange(diff, oldStyle);
    m_lineHeight = -1;
}

int RenderBR::caretMinOffset() const 
{ 
    return 0;
}

int RenderBR::caretMaxOffset() const 
{ 
    return 1;
}

unsigned RenderBR::caretMaxRenderedOffset() const
{
    return 1;
}

VisiblePosition RenderBR::positionForPoint(const IntPoint&)
{
    return createVisiblePosition(0, DOWNSTREAM);
}

} // namespace WebCore
