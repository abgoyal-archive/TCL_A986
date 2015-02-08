

#ifndef RenderSlider_h
#define RenderSlider_h

#include "RenderBlock.h"

namespace WebCore {

    class HTMLInputElement;
    class MouseEvent;
    class SliderThumbElement;
    
    class RenderSlider : public RenderBlock {
    public:
        RenderSlider(HTMLInputElement*);
        virtual ~RenderSlider();

        void forwardEvent(Event*);
        bool inDragMode() const;
        IntRect thumbRect();

    private:
        virtual const char* renderName() const { return "RenderSlider"; }
        virtual bool isSlider() const { return true; }

        virtual int baselinePosition(bool, bool) const;
        virtual void calcPrefWidths();
        virtual void layout();
        virtual void updateFromElement();

        bool mouseEventIsInThumb(MouseEvent*);
        FloatPoint mouseEventOffsetToThumb(MouseEvent*);

        void setValueForPosition(int position);
        void setPositionFromValue();
        int positionForOffset(const IntPoint&);

        int currentPosition();

        virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

        PassRefPtr<RenderStyle> createThumbStyle(const RenderStyle* parentStyle);

        int trackSize();

        RefPtr<SliderThumbElement> m_thumb;

        friend class SliderThumbElement;
    };

    inline RenderSlider* toRenderSlider(RenderObject* object)
    {
        ASSERT(!object || object->isSlider());
        return static_cast<RenderSlider*>(object);
    }

    // This will catch anyone doing an unnecessary cast.
    void toRenderSlider(const RenderSlider*);

} // namespace WebCore

#endif // RenderSlider_h
