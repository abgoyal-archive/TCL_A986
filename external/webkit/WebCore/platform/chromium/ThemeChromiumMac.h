

#ifndef ThemeChromiumMac_h
#define ThemeChromiumMac_h

#include "Theme.h"

// This file (and its associated .mm file) is a clone of ThemeMac.h. See
// the .mm file for details.

namespace WebCore {

class ThemeChromiumMac : public Theme {
public:
    ThemeChromiumMac() { }
    virtual ~ThemeChromiumMac() { }
    
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

#endif // ThemeChromiumMac_h
