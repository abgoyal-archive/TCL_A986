
 
#include "config.h"
#include "TextControlInnerElements.h"

#include "BeforeTextInsertedEvent.h"
#include "Document.h"
#include "EventHandler.h"
#include "EventNames.h"
#include "Frame.h"
#include "HTMLInputElement.h"
#include "HTMLNames.h"
#include "HTMLTextAreaElement.h"
#include "MouseEvent.h"
#include "RenderLayer.h"
#include "RenderTextControlSingleLine.h"

namespace WebCore {

class RenderTextControlInnerBlock : public RenderBlock {
public:
    RenderTextControlInnerBlock(Node* node, bool isMultiLine) : RenderBlock(node), m_multiLine(isMultiLine) { }

    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);
    virtual VisiblePosition positionForPoint(const IntPoint&);
    private:
        bool m_multiLine;
};

bool RenderTextControlInnerBlock::nodeAtPoint(const HitTestRequest& request, HitTestResult& result, int x, int y, int tx, int ty, HitTestAction hitTestAction)
{
    RenderObject* renderer = node()->shadowAncestorNode()->renderer();

    bool placeholderIsVisible = false;
    if (renderer->isTextField())
        placeholderIsVisible = toRenderTextControlSingleLine(renderer)->placeholderIsVisible();

    return RenderBlock::nodeAtPoint(request, result, x, y, tx, ty, placeholderIsVisible ? HitTestBlockBackground : hitTestAction);
}

VisiblePosition RenderTextControlInnerBlock::positionForPoint(const IntPoint& point)
{
    int contentsX = point.x();
    int contentsY = point.y();

    // Multiline text controls have the scroll on shadowAncestorNode, so we need to take that
    // into account here.
    if (m_multiLine) {
        RenderTextControl* renderer = toRenderTextControl(node()->shadowAncestorNode()->renderer());
        if (renderer->hasOverflowClip())
            renderer->layer()->addScrolledContentOffset(contentsX, contentsY);
    }

    return RenderBlock::positionForPoint(IntPoint(contentsX, contentsY));
}

TextControlInnerElement::TextControlInnerElement(Document* doc, Node* shadowParent)
    : HTMLDivElement(HTMLNames::divTag, doc)
    , m_shadowParent(shadowParent)
{
}

void TextControlInnerElement::attachInnerElement(Node* parent, PassRefPtr<RenderStyle> style, RenderArena* arena)
{
    // When adding these elements, create the renderer & style first before adding to the DOM.
    // Otherwise, the render tree will create some anonymous blocks that will mess up our layout.

    // Create the renderer with the specified style
    RenderObject* renderer = createRenderer(arena, style.get());
    if (renderer) {
        setRenderer(renderer);
        renderer->setStyle(style);
    }
    
    // Set these explicitly since this normally happens during an attach()
    setAttached();
    setInDocument(true);
    
    // For elements without a shadow parent, add the node to the DOM normally.
    if (!m_shadowParent)
        parent->addChild(this);
    
    // Add the renderer to the render tree
    if (renderer)
        parent->renderer()->addChild(renderer);
}

TextControlInnerTextElement::TextControlInnerTextElement(Document* doc, Node* shadowParent)
    : TextControlInnerElement(doc, shadowParent)
{
}

void TextControlInnerTextElement::defaultEventHandler(Event* evt)
{
    // FIXME: In the future, we should add a way to have default event listeners.  Then we would add one to the text field's inner div, and we wouldn't need this subclass.
    Node* shadowAncestor = shadowAncestorNode();
    if (shadowAncestor && shadowAncestor->renderer()) {
        ASSERT(shadowAncestor->renderer()->isTextControl());
        if (evt->isBeforeTextInsertedEvent()) {
            if (shadowAncestor->renderer()->isTextField())
                static_cast<HTMLInputElement*>(shadowAncestor)->defaultEventHandler(evt);
            else
                static_cast<HTMLTextAreaElement*>(shadowAncestor)->defaultEventHandler(evt);
        }
        if (evt->type() == eventNames().webkitEditableContentChangedEvent)
            toRenderTextControl(shadowAncestor->renderer())->subtreeHasChanged();
    }
    if (!evt->defaultHandled())
        HTMLDivElement::defaultEventHandler(evt);
}

RenderObject* TextControlInnerTextElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    bool multiLine = false;
    Node* shadowAncestor = shadowAncestorNode();
    if (shadowAncestor && shadowAncestor->renderer()) {
        ASSERT(shadowAncestor->renderer()->isTextField() || shadowAncestor->renderer()->isTextArea());
        multiLine = shadowAncestor->renderer()->isTextArea();
    }
    return new (arena) RenderTextControlInnerBlock(this, multiLine);
}

SearchFieldResultsButtonElement::SearchFieldResultsButtonElement(Document* doc)
    : TextControlInnerElement(doc)
{
}

void SearchFieldResultsButtonElement::defaultEventHandler(Event* evt)
{
    // On mousedown, bring up a menu, if needed
    HTMLInputElement* input = static_cast<HTMLInputElement*>(shadowAncestorNode());
    if (evt->type() == eventNames().mousedownEvent && evt->isMouseEvent() && static_cast<MouseEvent*>(evt)->button() == LeftButton) {
        input->focus();
        input->select();
        RenderTextControlSingleLine* renderer = toRenderTextControlSingleLine(input->renderer());
        if (renderer->popupIsVisible())
            renderer->hidePopup();
        else if (input->maxResults() > 0)
            renderer->showPopup();
        evt->setDefaultHandled();
    }
    if (!evt->defaultHandled())
        HTMLDivElement::defaultEventHandler(evt);
}

SearchFieldCancelButtonElement::SearchFieldCancelButtonElement(Document* doc)
    : TextControlInnerElement(doc)
    , m_capturing(false)
{
}

void SearchFieldCancelButtonElement::detach()
{
    if (m_capturing) {
        if (Frame* frame = document()->frame())
            frame->eventHandler()->setCapturingMouseEventsNode(0);      
    }
    TextControlInnerElement::detach();
}


void SearchFieldCancelButtonElement::defaultEventHandler(Event* evt)
{
    // If the element is visible, on mouseup, clear the value, and set selection
    HTMLInputElement* input = static_cast<HTMLInputElement*>(shadowAncestorNode());
    if (evt->type() == eventNames().mousedownEvent && evt->isMouseEvent() && static_cast<MouseEvent*>(evt)->button() == LeftButton) {
        input->focus();
        input->select();
        evt->setDefaultHandled();
        if (renderer() && renderer()->visibleToHitTesting())
            if (Frame* frame = document()->frame()) {
                frame->eventHandler()->setCapturingMouseEventsNode(this);
                m_capturing = true;
            }
    } else if (evt->type() == eventNames().mouseupEvent && evt->isMouseEvent() && static_cast<MouseEvent*>(evt)->button() == LeftButton) {
        if (m_capturing && renderer() && renderer()->visibleToHitTesting()) {
            if (hovered()) {
                input->setValue("");
                input->onSearch();
                evt->setDefaultHandled();
            }
            if (Frame* frame = document()->frame()) {
                frame->eventHandler()->setCapturingMouseEventsNode(0);
                m_capturing = false;
            }
        }
    }
    if (!evt->defaultHandled())
        HTMLDivElement::defaultEventHandler(evt);
}

}
