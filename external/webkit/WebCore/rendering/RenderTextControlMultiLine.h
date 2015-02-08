

#ifndef RenderTextControlMultiLine_h
#define RenderTextControlMultiLine_h

#include "RenderTextControl.h"

namespace WebCore {

class RenderTextControlMultiLine : public RenderTextControl {
public:
    RenderTextControlMultiLine(Node*, bool);
    virtual ~RenderTextControlMultiLine();

    void forwardEvent(Event*);

private:
    virtual bool isTextArea() const { return true; }

    virtual void subtreeHasChanged();

    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);

    virtual int preferredContentWidth(float charWidth) const;
    virtual void adjustControlHeightBasedOnLineHeight(int lineHeight);
    virtual int baselinePosition(bool firstLine, bool isRootLineBox) const;

    virtual void updateFromElement();
    virtual void cacheSelection(int start, int end);

    virtual RenderStyle* textBaseStyle() const;
    virtual PassRefPtr<RenderStyle> createInnerTextStyle(const RenderStyle* startStyle) const;
};

inline RenderTextControlMultiLine* toRenderTextControlMultiLine(RenderObject* object)
{ 
    ASSERT(!object || object->isTextArea());
    return static_cast<RenderTextControlMultiLine*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderTextControlMultiLine(const RenderTextControlMultiLine*);

}

#endif
