

#ifndef PopupMenuClient_h
#define PopupMenuClient_h

#include "PopupMenuStyle.h"
#include "ScrollTypes.h"

namespace WebCore {

class Color;
class FontSelector;
class HostWindow;
class Scrollbar;
class ScrollbarClient;
class String;

class PopupMenuClient {
public:
    virtual ~PopupMenuClient() {}
    virtual void valueChanged(unsigned listIndex, bool fireEvents = true) = 0;

    virtual String itemText(unsigned listIndex) const = 0;
    virtual String itemToolTip(unsigned listIndex) const = 0;
    virtual bool itemIsEnabled(unsigned listIndex) const = 0;
    virtual PopupMenuStyle itemStyle(unsigned listIndex) const = 0;
    virtual PopupMenuStyle menuStyle() const = 0;
    virtual int clientInsetLeft() const = 0;
    virtual int clientInsetRight() const = 0;
    virtual int clientPaddingLeft() const = 0;
    virtual int clientPaddingRight() const = 0;
    virtual int listSize() const = 0;
    virtual int selectedIndex() const = 0;
    virtual void popupDidHide() = 0;
    virtual bool itemIsSeparator(unsigned listIndex) const = 0;
    virtual bool itemIsLabel(unsigned listIndex) const = 0;
    virtual bool itemIsSelected(unsigned listIndex) const = 0;
    virtual bool shouldPopOver() const = 0;
    virtual bool valueShouldChangeOnHotTrack() const = 0;
    virtual void setTextFromItem(unsigned listIndex) = 0;
    
    virtual FontSelector* fontSelector() const = 0;
    virtual HostWindow* hostWindow() const = 0;
    
    virtual PassRefPtr<Scrollbar> createScrollbar(ScrollbarClient*, ScrollbarOrientation, ScrollbarControlSize) = 0;
};

}

#endif
