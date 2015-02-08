

#ifndef ThemeMac_h
#define ThemeMac_h

#include "Theme.h"

namespace WebCore {

class ThemeMac : public Theme {
public:
    ThemeMac() { }
    virtual ~ThemeMac() { }
    
    virtual int baselinePositionAdjustment(ControlPart) const;

    virtual FontDescription controlFont(ControlPart, const Font&, float zoomFactor) const;
    
    virtual LengthSize controlSize(ControlPart, const Font&, const LengthSize&, float zoomFactor) const;
    virtual LengthSize minimumControlSize(ControlPart, const Font&, float zoomFactor) const;

    virtual LengthBox controlPadding(ControlPart, const Font&, const LengthBox& zoomedBox, float zoomFactor) const;
    virtual LengthBox controlBorder(ControlPart, const Font&, const LengthBox& zoomedBox, float zoomFactor) const;

    virtual bool controlRequiresPreWhiteSpace(ControlPart part) const { return part == PushButtonPart; }

    virtual void paint(ControlPart, ControlStates, GraphicsContext*, const IntRect&, float zoomFactor, ScrollView*) const;
    virtual void inflateControlPaintRect(ControlPart, ControlStates, IntRect&, float zoomFactor) const;
};

} // namespace WebCore

#endif // ThemeMac_h
