

#ifndef RenderTextControl_h
#define RenderTextControl_h

#include "RenderBlock.h"

namespace WebCore {

class VisibleSelection;
class TextControlInnerElement;
class TextControlInnerTextElement;

class RenderTextControl : public RenderBlock {
public:
    virtual ~RenderTextControl();

    bool wasChangedSinceLastChangeEvent() const { return m_wasChangedSinceLastChangeEvent; }
    void setChangedSinceLastChangeEvent(bool wasChangedSinceLastChangeEvent) { m_wasChangedSinceLastChangeEvent = wasChangedSinceLastChangeEvent; }

    bool lastChangeWasUserEdit() const { return m_lastChangeWasUserEdit; }
    void setLastChangeWasUserEdit(bool lastChangeWasUserEdit);

    int selectionStart();
    int selectionEnd();
    void setSelectionStart(int);
    void setSelectionEnd(int);
    void select();
    void setSelectionRange(int start, int end);
    VisibleSelection selection(int start, int end) const;

    virtual void subtreeHasChanged();
    String text();
    String textWithHardLineBreaks();
    void selectionChanged(bool userTriggered);

    VisiblePosition visiblePositionForIndex(int index);
    int indexForVisiblePosition(const VisiblePosition&);

    void updatePlaceholderVisibility(bool, bool);

protected:
    RenderTextControl(Node*, bool);

    int scrollbarThickness() const;
    void adjustInnerTextStyle(const RenderStyle* startStyle, RenderStyle* textBlockStyle) const;
    void setInnerTextValue(const String&);

    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    void createSubtreeIfNeeded(TextControlInnerElement* innerBlock);
    void hitInnerTextElement(HitTestResult&, int x, int y, int tx, int ty);
    void forwardEvent(Event*);

    int textBlockWidth() const;
    int textBlockHeight() const;

    virtual int preferredContentWidth(float charWidth) const = 0;
    virtual void adjustControlHeightBasedOnLineHeight(int lineHeight) = 0;
    virtual void cacheSelection(int start, int end) = 0;
    virtual PassRefPtr<RenderStyle> createInnerTextStyle(const RenderStyle* startStyle) const = 0;
    virtual RenderStyle* textBaseStyle() const = 0;

    virtual void updateFromElement();
    virtual void calcHeight();

    friend class TextIterator;
    HTMLElement* innerTextElement() const;

    bool m_placeholderVisible;

private:
    virtual const char* renderName() const { return "RenderTextControl"; }
    virtual bool isTextControl() const { return true; }
    virtual bool hasControlClip() const { return false; }
    virtual IntRect controlClipRect(int tx, int ty) const;
    virtual void calcPrefWidths();
    virtual void removeLeftoverAnonymousBlock(RenderBlock*) { }
    virtual bool canHaveChildren() const { return false; }
    virtual bool avoidsFloats() const { return true; }
    void setInnerTextStyle(PassRefPtr<RenderStyle>);
    
    virtual void addFocusRingRects(Vector<IntRect>&, int tx, int ty);

    virtual bool canBeProgramaticallyScrolled(bool) const { return true; }

    String finishText(Vector<UChar>&) const;

    bool m_wasChangedSinceLastChangeEvent;
    bool m_lastChangeWasUserEdit;
    RefPtr<TextControlInnerTextElement> m_innerText;
};

inline RenderTextControl* toRenderTextControl(RenderObject* object)
{ 
    ASSERT(!object || object->isTextControl());
    return static_cast<RenderTextControl*>(object);
}

inline const RenderTextControl* toRenderTextControl(const RenderObject* object)
{ 
    ASSERT(!object || object->isTextControl());
    return static_cast<const RenderTextControl*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderTextControl(const RenderTextControl*);

} // namespace WebCore

#endif // RenderTextControl_h
