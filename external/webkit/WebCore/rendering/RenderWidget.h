

#ifndef RenderWidget_h
#define RenderWidget_h

#include "OverlapTestRequestClient.h"
#include "RenderReplaced.h"

namespace WebCore {

class Widget;

class RenderWidget : public RenderReplaced, private OverlapTestRequestClient {
public:
    virtual ~RenderWidget();

    Widget* widget() const { return m_widget.get(); }
    virtual void setWidget(PassRefPtr<Widget>);

    static RenderWidget* find(const Widget*);

    void updateWidgetPosition();

    void showSubstituteImage(PassRefPtr<Image>);

    static void suspendWidgetHierarchyUpdates();
    static void resumeWidgetHierarchyUpdates();

protected:
    RenderWidget(Node*);

    FrameView* frameView() const { return m_frameView; }

    void clearWidget();

    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);
    virtual void layout();

private:
    virtual bool isWidget() const { return true; }

    virtual void paint(PaintInfo&, int x, int y);
    virtual void destroy();
    virtual void setSelectionState(SelectionState);
    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);
    virtual void setOverlapTestResult(bool);

    bool setWidgetGeometry(const IntRect&);

    friend class RenderWidgetProtector;
    RenderArena* ref() { ++m_refCount; return renderArena(); }
    void deref(RenderArena*);

    RefPtr<Widget> m_widget;
    RefPtr<Image> m_substituteImage;
    FrameView* m_frameView;
    int m_refCount;
};

inline RenderWidget* toRenderWidget(RenderObject* object)
{
    ASSERT(!object || object->isWidget());
    return static_cast<RenderWidget*>(object);
}

inline const RenderWidget* toRenderWidget(const RenderObject* object)
{
    ASSERT(!object || object->isWidget());
    return static_cast<const RenderWidget*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderWidget(const RenderWidget*);

} // namespace WebCore

#endif // RenderWidget_h
