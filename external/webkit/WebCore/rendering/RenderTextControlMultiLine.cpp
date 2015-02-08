

#include "config.h"
#include "RenderTextControlMultiLine.h"

#include "Event.h"
#include "EventNames.h"
#include "Frame.h"
#include "HTMLNames.h"
#include "HTMLTextAreaElement.h"
#include "HitTestResult.h"
#ifdef ANDROID_LAYOUT
#include "Settings.h"
#endif

namespace WebCore {

RenderTextControlMultiLine::RenderTextControlMultiLine(Node* node, bool placeholderVisible)
    : RenderTextControl(node, placeholderVisible)
{
}

RenderTextControlMultiLine::~RenderTextControlMultiLine()
{
    if (node())
        static_cast<HTMLTextAreaElement*>(node())->rendererWillBeDestroyed();
}

void RenderTextControlMultiLine::subtreeHasChanged()
{
    RenderTextControl::subtreeHasChanged();
    HTMLTextAreaElement* textArea = static_cast<HTMLTextAreaElement*>(node());
    textArea->setFormControlValueMatchesRenderer(false);
    textArea->setNeedsValidityCheck();

    if (!node()->focused())
        return;

    node()->dispatchEvent(Event::create(eventNames().inputEvent, true, false));

    if (Frame* frame = document()->frame())
        frame->textDidChangeInTextArea(textArea);
}

bool RenderTextControlMultiLine::nodeAtPoint(const HitTestRequest& request, HitTestResult& result, int x, int y, int tx, int ty, HitTestAction hitTestAction)
{
    if (!RenderTextControl::nodeAtPoint(request, result, x, y, tx, ty, hitTestAction))
        return false;

    bool resultIsTextValueOrPlaceholder
        = (!m_placeholderVisible && result.innerNode() == innerTextElement())
        || (m_placeholderVisible && result.innerNode()->isDescendantOf(innerTextElement()));
    if (result.innerNode() == node() || resultIsTextValueOrPlaceholder)
        hitInnerTextElement(result, x, y, tx, ty);

    return true;
}

void RenderTextControlMultiLine::forwardEvent(Event* event)
{
    RenderTextControl::forwardEvent(event);
}

int RenderTextControlMultiLine::preferredContentWidth(float charWidth) const
{
    int factor = static_cast<HTMLTextAreaElement*>(node())->cols();
    return static_cast<int>(ceilf(charWidth * factor)) + scrollbarThickness();
}

void RenderTextControlMultiLine::adjustControlHeightBasedOnLineHeight(int lineHeight)
{
    setHeight(height() + lineHeight * static_cast<HTMLTextAreaElement*>(node())->rows());
}

int RenderTextControlMultiLine::baselinePosition(bool, bool) const
{
    return height() + marginTop() + marginBottom();
}

void RenderTextControlMultiLine::updateFromElement()
{
    createSubtreeIfNeeded(0);
    RenderTextControl::updateFromElement();

    HTMLTextAreaElement* textArea = static_cast<HTMLTextAreaElement*>(node());
    if (m_placeholderVisible)
        setInnerTextValue(textArea->getAttribute(HTMLNames::placeholderAttr));
    else
        setInnerTextValue(textArea->value());
}

void RenderTextControlMultiLine::cacheSelection(int start, int end)
{
    static_cast<HTMLTextAreaElement*>(node())->cacheSelection(start, end);
}

PassRefPtr<RenderStyle> RenderTextControlMultiLine::createInnerTextStyle(const RenderStyle* startStyle) const
{
    RefPtr<RenderStyle> textBlockStyle;
    if (m_placeholderVisible) {
        if (RenderStyle* pseudoStyle = getCachedPseudoStyle(INPUT_PLACEHOLDER))
            textBlockStyle = RenderStyle::clone(pseudoStyle);
    }
    if (!textBlockStyle) {
        textBlockStyle = RenderStyle::create();
        textBlockStyle->inheritFrom(startStyle);
    }

    adjustInnerTextStyle(startStyle, textBlockStyle.get());
    textBlockStyle->setDisplay(BLOCK);

    return textBlockStyle.release();
}

RenderStyle* RenderTextControlMultiLine::textBaseStyle() const
{
    return style();
}

}
