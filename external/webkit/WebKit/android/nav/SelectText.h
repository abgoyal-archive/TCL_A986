

#ifndef SELECT_TEXT_H
#define SELECT_TEXT_H

#include "DrawExtra.h"
#include "IntRect.h"
#include "PlatformString.h"
#include "SkPath.h"
#define DRAW_TOO_LONG 100
#define LOOP_TOO_LONG 400

class SkPicture;
struct SkIRect;
class SkRegion;

namespace android {

struct Cluster{
    SkIRect startRect;
    SkIRect endRect;
};

class CachedRoot;

class SelectText : public DrawExtra {
public:
    SelectText();
    virtual void draw(SkCanvas* , LayerAndroid* );
    void extendSelection(const SkPicture* , int x, int y);
    const String getSelection();
    bool hitSelection(int x, int y) const;
    void moveSelection(const SkPicture* , int x, int y);
    void reset();
    void selectAll(const SkPicture* );
    int selectionX() const;
    int selectionY() const;
    void setDrawPointer(bool drawPointer) { m_drawPointer = drawPointer; }
    void setExtendSelection(bool extend) { m_extendSelection = extend; }
    void setVisibleRect(const IntRect& rect) { m_visibleRect = rect; }
    bool startSelection(int x, int y);
    bool wordSelection(const SkPicture* picture);
public:
    float m_inverseScale; // inverse scale, x, y used for drawing select path
    int m_selectX;
    int m_selectY;
private:
    void drawSelectionPointer(SkCanvas* );
    void drawSelectionRegion(SkCanvas* );
    static void getSelectionArrow(SkPath* );
    void getSelectionCaret(SkPath* );
    bool hitCorner(int cx, int cy, int x, int y) const;
    void swapAsNeeded();
    SkIPoint m_original; // computed start of extend selection
    SkIRect m_selStart;
    SkIRect m_selEnd;
    int m_startBase;
    int m_endBase;
    SkIRect m_visibleRect; // constrains picture computations to visible area
    SkRegion m_selRegion; // computed from sel start, end
    SkPicture m_startControl;
    SkPicture m_endControl;
    const SkPicture* m_picture;
    bool m_drawPointer;
    bool m_extendSelection; // false when trackball is moving pointer
    bool m_flipped;
    bool m_hitTopLeft;
    bool m_startSelection;

    Cluster mStartCluster;
    Cluster mEndCluster;
    
};

}

namespace WebCore {

void ReverseBidi(UChar* chars, int len);

}

#endif
