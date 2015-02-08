

#ifndef StrokeStyleApplier_h
#define StrokeStyleApplier_h

namespace WebCore {

    class GraphicsContext;

    class StrokeStyleApplier {
    public:
        virtual void strokeStyle(GraphicsContext*) = 0;

    protected:
        StrokeStyleApplier() {}
        virtual ~StrokeStyleApplier() {}
    };
}

#endif

