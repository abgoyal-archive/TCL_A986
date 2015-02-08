

#include "config.h"
#include "RenderSkinCombo.h"

#include "CString.h"
#include "Document.h"
#include "Element.h"
#include "Node.h"
#include "NodeRenderStyle.h"
#include "RenderStyle.h"
#include "SkCanvas.h"
#include "SkNinePatch.h"

namespace WebCore {

// Indicates if the entire asset is being drawn, or if the border is being
// excluded and just the arrow drawn.
enum BorderStyle {
    FullAsset,
    NoBorder
};

// There are 2.5 different concepts of a 'border' here, which results
// in rather a lot of magic constants. In each case, there are 2
// numbers, one for medium res and one for high-res. All sizes are in pixels.

// Firstly, we have the extra padding that webkit needs to know about,
// which defines how much bigger this element is made by the
// asset. This is actually a bit broader than the actual border on the
// asset, to make things look less cramped. The border is the same
// width on all sides, except on the right when it's significantly
// wider to allow for the arrow.
const int RenderSkinCombo::arrowMargin[2] = {22, 34};
const int RenderSkinCombo::padMargin[2] = {2, 5};

// Then we have the borders used for the 9-patch stretch. The
// rectangle at the centre of these borders is entirely below and to
// the left of the arrow in the asset. Hence the border widths are the
// same for the bottom and left, but are different for the top. The
// right hand border width happens to be the same as arrowMargin
// defined above.
static const int        stretchMargin[2] = {3, 5};   // border width for the bottom and left of the 9-patch
static const int        stretchTop[2] = {15, 23};     // border width for the top of the 9-patch

// Finally, if the border is defined by the CSS, we only draw the
// arrow and not the border. We do this by drawing the relevant subset
// of the bitmap, which must now be precisely determined by what's in
// the asset with no extra padding to make things look properly
// spaced. The border to remove at the top, right and bottom of the
// image is the same as stretchMargin above, but we need to know the width
// of the arrow.
static const int arrowWidth[2] = {22, 31};

RenderSkinCombo::Resolution RenderSkinCombo::resolution = MedRes;

const SkIRect RenderSkinCombo::margin[2][2] = {{{ stretchMargin[MedRes], stretchTop[MedRes],
                                          RenderSkinCombo::arrowMargin[MedRes] + stretchMargin[MedRes], stretchMargin[MedRes] },
                                        {0, stretchTop[MedRes], 0, stretchMargin[MedRes]}},
                                       {{ stretchMargin[HighRes], stretchTop[HighRes],
                                          RenderSkinCombo::arrowMargin[HighRes] + stretchMargin[HighRes], stretchMargin[HighRes] },
                                        {0, stretchTop[HighRes], 0, stretchMargin[HighRes]}}};
const SkIRect RenderSkinCombo::margin_small[2][2] = {{{ stretchMargin[MedRes], stretchMargin[MedRes],
                                          RenderSkinCombo::arrowMargin[MedRes] + stretchMargin[MedRes], stretchMargin[MedRes] },
                                        {0, stretchMargin[MedRes], 0, stretchMargin[MedRes]}},
                                       {{ stretchMargin[HighRes], stretchMargin[HighRes],
                                          RenderSkinCombo::arrowMargin[HighRes] + stretchMargin[HighRes], stretchMargin[HighRes] },
                                        {0, stretchMargin[HighRes], 0, stretchMargin[HighRes]}}};
static SkBitmap         bitmaps[2][2]; // Collection of assets for a combo box
static bool             isDecoded;      // True if all assets were decoded

void RenderSkinCombo::Init(android::AssetManager* am, String drawableDirectory)
{
    if (isDecoded)
        return;

    if (drawableDirectory[drawableDirectory.length() - 5] == 'h')
        resolution = HighRes;

    isDecoded = RenderSkinAndroid::DecodeBitmap(am, (drawableDirectory + "combobox_nohighlight.png").utf8().data(), &bitmaps[kNormal][FullAsset]);
    isDecoded &= RenderSkinAndroid::DecodeBitmap(am, (drawableDirectory + "combobox_disabled.png").utf8().data(), &bitmaps[kDisabled][FullAsset]);

    int width = bitmaps[kNormal][FullAsset].width();
    int height = bitmaps[kNormal][FullAsset].height();
    SkIRect  subset;
    subset.set(width - arrowWidth[resolution], 0, width, height);
    bitmaps[kNormal][FullAsset].extractSubset(&bitmaps[kNormal][NoBorder], subset);
    bitmaps[kDisabled][FullAsset].extractSubset(&bitmaps[kDisabled][NoBorder], subset);
}


bool RenderSkinCombo::Draw(SkCanvas* canvas, Node* element, int x, int y, int width, int height)
{
    if (!isDecoded)
        return true;

    State state = (element->isElementNode() && static_cast<Element*>(element)->isEnabledFormControl()) ? kNormal : kDisabled;
    int oldHeight = height;
    height = std::max(height, (stretchMargin[resolution]<<1) + 1);

    SkRect bounds;
    BorderStyle drawBorder = FullAsset;

    bounds.set(SkIntToScalar(x+1), SkIntToScalar(y+1), SkIntToScalar(x + width-1), SkIntToScalar(y + height-1));
    RenderStyle* style = element->renderStyle();
    SkPaint paint;
    paint.setColor(style->backgroundColor().rgb());
    canvas->drawRect(bounds, paint);

    bounds.set(SkIntToScalar(x), SkIntToScalar(y), SkIntToScalar(x + width), SkIntToScalar(y + height));

    if (style->borderLeftColor().isValid() ||
        style->borderRightColor().isValid() ||
        style->borderTopColor().isValid() ||
        style->borderBottomColor().isValid()) {
        bounds.fLeft += SkIntToScalar(width - RenderSkinCombo::extraWidth());
        bounds.fRight -= SkIntToScalar(style->borderRightWidth());
        bounds.fTop += SkIntToScalar(style->borderTopWidth());
        bounds.fBottom -= SkIntToScalar(style->borderBottomWidth());
        drawBorder = NoBorder;
    }
    if (oldHeight > stretchMargin[resolution] + stretchTop[resolution] + 1)
        SkNinePatch::DrawNine(canvas, bounds, bitmaps[state][drawBorder], margin[resolution][drawBorder]);
    else
    	SkNinePatch::DrawNine(canvas, bounds, bitmaps[state][drawBorder], margin_small[resolution][drawBorder]);
    return false;
}

}   //WebCore
