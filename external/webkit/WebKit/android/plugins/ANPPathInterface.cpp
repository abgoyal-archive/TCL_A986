

// must include config.h first for webkit to fiddle with new/delete
#include "config.h"
#include "SkANP.h"

static ANPPath* anp_newPath() {
    return new ANPPath;
}

static void anp_deletePath(ANPPath* path) {
    delete path;
}

static void anp_copy(ANPPath* dst, const ANPPath* src) {
    *dst = *src;
}

static bool anp_equal(const ANPPath* p0, const ANPPath* p1) {
    return *p0 == *p1;
}

static void anp_reset(ANPPath* path) {
    path->reset();
}

static bool anp_isEmpty(const ANPPath* path) {
    return path->isEmpty();
}

static void anp_getBounds(const ANPPath* path, ANPRectF* bounds) {
    SkANP::SetRect(bounds, path->getBounds());
}

static void anp_moveTo(ANPPath* path, float x, float y) {
    path->moveTo(SkFloatToScalar(x), SkFloatToScalar(y));
}

static void anp_lineTo(ANPPath* path, float x, float y) {
    path->lineTo(SkFloatToScalar(x), SkFloatToScalar(y));
}

static void anp_quadTo(ANPPath* path, float x0, float y0, float x1, float y1) {
    path->quadTo(SkFloatToScalar(x0), SkFloatToScalar(y0),
                 SkFloatToScalar(x1), SkFloatToScalar(y1));
}

static void anp_cubicTo(ANPPath* path, float x0, float y0,
                        float x1, float y1, float x2, float y2) {
    path->cubicTo(SkFloatToScalar(x0), SkFloatToScalar(y0),
                  SkFloatToScalar(x1), SkFloatToScalar(y1),
                  SkFloatToScalar(x2), SkFloatToScalar(y2));
}

static void anp_close(ANPPath* path) {
    path->close();
}

static void anp_offset(ANPPath* path, float dx, float dy, ANPPath* dst) {
    path->offset(SkFloatToScalar(dx), SkFloatToScalar(dy), dst);
}

static void anp_transform(ANPPath* src, const ANPMatrix* matrix,
                          ANPPath* dst) {
    src->transform(*matrix, dst);
}

///////////////////////////////////////////////////////////////////////////////

#define ASSIGN(obj, name)   (obj)->name = anp_##name

void ANPPathInterfaceV0_Init(ANPInterface* value) {
    ANPPathInterfaceV0* i = reinterpret_cast<ANPPathInterfaceV0*>(value);

    ASSIGN(i, newPath);
    ASSIGN(i, deletePath);
    ASSIGN(i, copy);
    ASSIGN(i, equal);
    ASSIGN(i, reset);
    ASSIGN(i, isEmpty);
    ASSIGN(i, getBounds);
    ASSIGN(i, moveTo);
    ASSIGN(i, lineTo);
    ASSIGN(i, quadTo);
    ASSIGN(i, cubicTo);
    ASSIGN(i, close);
    ASSIGN(i, offset);
    ASSIGN(i, transform);
}
