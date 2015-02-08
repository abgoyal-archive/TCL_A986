

#ifndef SkANP_DEFINED
#define SkANP_DEFINED

#include "android_npapi.h"
#include "SkCanvas.h"
#include "SkMatrix.h"
#include "SkPaint.h"
#include "SkPath.h"
#include "SkTypeface.h"

struct ANPMatrix : SkMatrix {
};

struct ANPPath : SkPath {
};

struct ANPPaint : SkPaint {
};

struct ANPTypeface : SkTypeface {
};

struct ANPCanvas {
    SkCanvas* skcanvas;

    // draw into the specified bitmap
    explicit ANPCanvas(const SkBitmap& bm) {
        skcanvas = new SkCanvas(bm);
    }

    // redirect all drawing to the specific SkCanvas
    explicit ANPCanvas(SkCanvas* other) {
        skcanvas = other;
        skcanvas->ref();
    }

    ~ANPCanvas() {
        skcanvas->unref();
    }
};

class SkANP {
public:
    static SkRect* SetRect(SkRect* dst, const ANPRectF& src);
    static SkIRect* SetRect(SkIRect* dst, const ANPRectI& src);
    static ANPRectI* SetRect(ANPRectI* dst, const SkIRect& src);
    static ANPRectF* SetRect(ANPRectF* dst, const SkRect& src);
    static SkBitmap* SetBitmap(SkBitmap* dst, const ANPBitmap& src);
    static bool SetBitmap(ANPBitmap* dst, const SkBitmap& src);
    
    static void InitEvent(ANPEvent* event, ANPEventType et);
};

#endif
