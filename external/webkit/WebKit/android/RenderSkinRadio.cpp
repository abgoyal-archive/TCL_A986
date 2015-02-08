

#include "config.h"
#include "RenderSkinRadio.h"

#include "CString.h"
#include "android_graphics.h"
#include "Document.h"
#include "Element.h"
#include "InputElement.h"
#include "IntRect.h"
#include "Node.h"
#include "RenderSkinAndroid.h"
#include "SkBitmap.h"
#include "SkCanvas.h"
#include "SkRect.h"

static const char* checks[] = { "btn_check_off.png",
                                "btn_check_on.png",
                                "btn_radio_off.png",
                                "btn_radio_on.png"};
// Matches the width of the bitmap
static SkScalar SIZE;

namespace WebCore {

static SkBitmap s_bitmap[4];
static bool     s_decoded;

void RenderSkinRadio::Init(android::AssetManager* am, String drawableDirectory)
{
    if (s_decoded)
        return;
    String path = drawableDirectory + checks[0];
    s_decoded = RenderSkinAndroid::DecodeBitmap(am, path.utf8().data(), &s_bitmap[0]);
    path = drawableDirectory + checks[1];
    s_decoded = RenderSkinAndroid::DecodeBitmap(am, path.utf8().data(), &s_bitmap[1]) && s_decoded;
    path = drawableDirectory + checks[2];
    s_decoded = RenderSkinAndroid::DecodeBitmap(am, path.utf8().data(), &s_bitmap[2]) && s_decoded;
    path = drawableDirectory + checks[3];
    s_decoded = RenderSkinAndroid::DecodeBitmap(am, path.utf8().data(), &s_bitmap[3]) && s_decoded;
    SIZE = SkIntToScalar(s_bitmap[0].width());
}

void RenderSkinRadio::Draw(SkCanvas* canvas, Node* element, const IntRect& ir,
        bool isCheckBox)
{
    if (!s_decoded || !element) {
        return;
    }
    SkRect r(ir);
    // Set up a paint to with filtering to look better.
    SkPaint paint;
    paint.setFlags(SkPaint::kFilterBitmap_Flag);
    int saveScaleCount = 0;

    if (!element->isElementNode() ||
        !static_cast<Element*>(element)->isEnabledFormControl()) {
        paint.setAlpha(0x80);
    }
    SkScalar width = r.width();
    SkScalar scale = SkScalarDiv(width, SIZE);
    saveScaleCount = canvas->save();
    canvas->translate(r.fLeft, r.fTop);
    canvas->scale(scale, scale);

    bool checked = false;
    if (InputElement* inputElement = toInputElement(static_cast<Element*>(element))) {
        checked = inputElement->isChecked();
    }

    canvas->drawBitmap(s_bitmap[checked + 2*(!isCheckBox)],
            0, 0, &paint);
    canvas->restoreToCount(saveScaleCount);
}

} //WebCore
